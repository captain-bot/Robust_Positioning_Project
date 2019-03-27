/* 
 * File:   myIKFastwrap.cpp
 * Author: Anirban Sinha
 * Email: anirban.sinha.jgec@gmail.com
 * Created on October 7th, 2018, 7:19PM
 */

#include <cstdlib>

using namespace std;
#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IK_VERSION 61
#include "baxter_left_arm_ikfast_solver.cpp" /*use right_arm solver if working with right arm*/
#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()
#include <Python.h>
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>
#define NPY_NO_DEPRICATED_API NPY_1_7_API_VERSION
#include <fstream>

#define IKREAL_TYPE IkReal // for IKFast 56,61
#define W1_MAX 2.094
#define W1_MIN -1.5707
#define PTS 500


bool check_joint_limits(std::vector<IkReal> solvals) {
        /* Joint limits */
        bool jlflg = true;
        double tolval = .01;
        double ul[] = {1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094, 3.059};
        double ll[] = {-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059};
        for( std::size_t j = 0; j < solvals.size(); ++j) {
                if (solvals[j] > ul[j]-tolval || solvals[j] < ll[j]+tolval) {
                        jlflg = false;
                        return jlflg;
                }
        }
        return jlflg;
}

extern "C"{
    // Function wrapping Fibonacci function in C++
    static PyObject* compIKs_wrap(PyObject* /*self*/, PyObject* args)
    {
	int numListElement;
	int config_num;
	PyObject* ee_pose;
        if (!PyArg_ParseTuple(args, "iO!", &config_num, &PyList_Type, &ee_pose)) {
            printf("Error parsing input argument(s)\n");
            return NULL;
        }
	numListElement = PyList_Size(ee_pose);
	if (numListElement < 12) {
	    Py_INCREF(Py_None);
	    printf("There is no 12 elements given in the ee_pose list\n");
	    return Py_None;
	}
        IkSolutionList<IkReal> solutions;
        std::vector<IkReal> vfree(GetNumFreeParameters());
        IkReal eerot[9],eetrans[3];
        /*Create a vector to hold all solutions*/
        std::vector<double> solVec;
        std::string fl_name = "ik_sol_config" + std::to_string(config_num) + ".txt";
	if (std::fstream{fl_name}) std::cout << "file exists\n";
	if (std::remove(fl_name.c_str())==0) std::cout << "file deleted\n";
        std::ofstream joint_sol;
        joint_sol.open(fl_name.c_str(), std::ios::app);
	std::cout << "new file created\n";

	int eetrans_idx = 0;
	int eerot_idx = 0;
	for(int i = 0; i < numListElement; i++) {
	    if (i == 3 || i == 7 || i == 11) {
            	eetrans[eetrans_idx] = PyFloat_AsDouble(PyList_GetItem(ee_pose, i));
		eetrans_idx += 1;
	     }
	    else {
		eerot[eerot_idx] = PyFloat_AsDouble(PyList_GetItem(ee_pose, i));
		eerot_idx += 1;
	    }
	}

        double w1_max = W1_MAX;
        double w1_min = W1_MIN;
        double discrete_pts = PTS;
        double dang = (w1_max - w1_min)/discrete_pts;
        double w1_ang = w1_min;
       
        int num_of_valid_solutions = 0;

        for (int ii = 0; ii < discrete_pts; ii++) {
		for(std::size_t i = 0; i < vfree.size(); ++i)
		                vfree[i] = w1_ang;
		// call ComputeIk method
		bool got_sol = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
                
		if (got_sol) {
			unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
			
			std::vector<double> solvalues(GetNumJoints());
                        printf("--------------------------\n");
			printf("Num of solutions  found: %d\n", (int)solutions.GetNumSolutions());
			for (std::size_t i =0; i < num_of_solutions; i++) {
			    const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
			    int this_sol_free_params = (int)sol.GetFree().size();
					    std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
			    sol.GetSolution(&solvalues[0], vsolfree.size()>0?&vsolfree[0]:NULL);
			    /* Check joint limits */
			    bool jl_ok = check_joint_limits(solvalues);
			    if (jl_ok) {
                                 	printf("sol%d (free=%d): ", (int)i, this_sol_free_params);
				 for(std::size_t j = 0; j < solvalues.size(); ++j)
				 {
                                	printf("%.15f, ", solvalues[j]);
					if (j == solvalues.size()-1)
						joint_sol << solvalues[j] << endl;
					else
						joint_sol << solvalues[j] << ",";
				}
				printf("\n");
				num_of_valid_solutions++;
			    }
			   }
		 }
		 w1_ang += dang; /* increase w2_ang */
        }
        if (num_of_valid_solutions != 0) {
		std::cout << "IKFast found " << num_of_valid_solutions << " valid IKs\n" << std::endl;
		return Py_True;
        }
        else {
	       std::cout << "No solutions found\n" << std::endl;
               return Py_False;
        }
    }

   // Wrapper function for forward kinematics
   static PyObject* compFK_wrap(PyObject* /*self*/, PyObject* args) {
	PyObject* jointAngles;
	PyArrayObject* pyArrFk;
	npy_intp dims[2] = {3, 4};

	IKREAL_TYPE eerot[9], eetrans[3];
   	IKREAL_TYPE joints[GetNumJoints()];

	
	unsigned int total_elements = 12;
	double* fkArr = new double[total_elements];
	int count_idx = 0;
	int trans_idx = 0;
	int rot_idx = 0;

	if (!PyArg_ParseTuple(args, "O!", &PyList_Type, &jointAngles)) {
            printf("Error parsing input argument(s)\n");
            return NULL;
        }
	int numListElement = PyList_Size(jointAngles);
	if (numListElement < 7) {
	    Py_INCREF(Py_None);
	    printf("There is no 7 elements given in the joint angles list\n");
	    return Py_None;
	}

	for (int i = 0; i < numListElement; i++) {
        	joints[i] = PyFloat_AsDouble(PyList_GetItem(jointAngles, i));;
        }

	// Call ComputeFk method of IKFast module
        ComputeFk(joints, eetrans, eerot);
	printf(" Translation: x: %f  y: %f  z: %f  \n", eetrans[0], eetrans[1], eetrans[2]);
	printf("\n");
	printf("    Rotation       %f   %f   %f \n", eerot[0], eerot[1], eerot[2] );
        printf("      Matrix       %f   %f   %f \n", eerot[3], eerot[4], eerot[5] );
	printf("                   %f   %f   %f \n", eerot[6], eerot[7], eerot[8] );
	
	// machinary to return FK to python side
	for (unsigned int j=0; j < total_elements; j++) {
	    if (j == 3 || j == 7 || j == 11) {
	   	 *(fkArr + count_idx) = eetrans[trans_idx];
	   	 trans_idx++;
            }
	    else {
	   	 *(fkArr + count_idx) = eerot[rot_idx];
	   	 rot_idx++;
	    }
	    count_idx++;
	}
	pyArrFk = (PyArrayObject*) PyArray_SimpleNewFromData(2, dims, NPY_DOUBLE, fkArr);

	// follwoing flag moves ownership of the allocated momory on stack to NPY_ARRAY
	// and frees the memory when memory becomes out of scope
	PyArray_ENABLEFLAGS(pyArrFk, NPY_ARRAY_OWNDATA);
	
	return Py_BuildValue("O", pyArrFk);
    }

   // An array specifying exactly which methods are wrappers
   static PyMethodDef CppMethods[] = {
	{"compIKs", compIKs_wrap, METH_VARARGS, "Computes all IKs of Baxter left/right arm"},
	{"compFK", compFK_wrap, METH_VARARGS, "Computes FK of Baxter left/right arm"},
        {NULL, NULL, 0, NULL}
   };
  // Define module information
  static struct PyModuleDef ikModule = {
	PyModuleDef_HEAD_INIT,
    "ikModule",
    "inverse_kinematics Module",
    -1,
    CppMethods
  };
  // Init function for module
  PyMODINIT_FUNC PyInit_ikModule(void) {
	import_array();
  	return PyModule_Create(&ikModule);
  }
}

