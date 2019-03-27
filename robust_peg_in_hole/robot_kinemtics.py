import numpy as np
import math
import copy
import ikModule


class Pose3d(object):
    def __init__(self):
        self._p = list()
        self._q = list()

    def set_position(self, position):
        self._p = position

    def set_rotquat(self, quat):
        self._q = quat


class RobotKin(object):
    def __init__(self, end_link="left_gripper_base"):
        self._base = np.array([[0.7071, -0.7071, 0, 0.0640],
                               [0.7071, 0.7071, 0, 0.2590],
                               [0, 0, 1.0, 0.1296],
                               [0, 0, 0, 1.0]])
        self._ak = np.array([0.069, 0.0, 0.069, 0.0, 0.010, 0.0, 0.0])
        self._alpk = np.array([-np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0])
        if end_link == "left_gripper_base":
            self._end_link_len = 0.025
        elif end_link == "left_gripper":
            self._end_link_len = 0.15
        elif end_link == "IKFast_frame":
            self._end_link_len = 0.05
        else:
            self._end_link_len = 0
        self._dk = np.array([0.27035, 0.0, (0.102 + 0.26242), 0.0, (0.10359 + 0.2707), 0.0,
                             (0.115975 + 0.11355)+self._end_link_len])
        self._premult_ikfast = np.linalg.inv(np.array([[0.7071, -0.7071, 0, 0.0246],
                                                       [0.7071, 0.7071, 0, 0.2196],
                                                       [0, 0, 1.0, 0.1186],
                                                       [0, 0, 0, 1]]))
        self._ikfast_exlen = self._end_link_len - 0.05

    # Function to solve forward kiematics upto end-frame
    def forward_kin(self, ang):
        all_trans = np.zeros((len(ang)+1, 4, 4))
        all_trans[0, :, :] = self._base
        for i in range(len(ang)):
            all_trans[i+1, :, :] = np.dot(all_trans[i, :, :], self.localtrans(ang[i], i))
        return all_trans

    # Function to compute analytical Jacobian
    def analytic_jacobian(self, ang):
        trans_upto = self.forward_kin(ang)
        n = trans_upto.shape[0]
        z_i = np.zeros((3, n))
        o_i = np.zeros((3, n))
        jacob = np.zeros((6, n-1))

        z_i[:, 0] = self._base[0:3, 2]
        o_i[:, 0] = self._base[0:3, 3]

        for ii in range(1, n):
            z_i[:, ii] = trans_upto[ii, 0:3, 2]
            o_i[:, ii] = trans_upto[ii, 0:3, 3]

        for jj in range(1, n):
            jacob[0:3, jj-1] = np.cross(z_i[:, jj-1], o_i[:, n-1] - o_i[:, jj-1]).reshape((3,))
            jacob[3:, jj-1] = z_i[:, jj-1]
        return jacob

    # Local transformations
    def localtrans(self, th, idx):
        if idx == 1:
            th += np.pi/2
        tmat = np.array([[math.cos(th), -math.cos(self._alpk[idx])*math.sin(th), math.sin(self._alpk[idx])*math.sin(th), self._ak[idx]*math.cos(th)],
                         [math.sin(th),  math.cos(self._alpk[idx])*math.cos(th), -math.sin(self._alpk[idx])*math.cos(th), self._ak[idx]*math.sin(th)],
                         [0, math.sin(self._alpk[idx]), math.cos(self._alpk[idx]), self._dk[idx]],
                         [0, 0, 0, 1]])
        return tmat


class PoseErrBound(RobotKin):
    def __init__(self, end_link, knum, sig, p, q):
        RobotKin.__init__(self, end_link)
        self._cval = math.pow(knum*sig, 2)
        self._des_config = Pose3d()
        self._des_config.set_position(p)
        self._des_config.set_rotquat(q)

    # Position error bound
    def position_err_bound(self, **kwargs):
        if "jp" in kwargs.keys():
            jp = kwargs['jp']
        else:
            th = self.get_one_jointsol()
            if len(th) > 0:
                manipulator_jac = self.analytic_jacobian(th)
                jp = manipulator_jac[:3, :]
            else:
                print("No IK found: unable to compute position_err_bound\n")
                return 0
        w, v = np.linalg.eig(np.dot(jp, jp.T))
        max_id = np.argmax(w)
        max_eig = w[max_id]
        return math.sqrt(self._cval*max_eig)

    # Orientation error bound
    def rotation_err_bound(self, **kwargs):
        if "jr" in kwargs.keys():
            jr = kwargs['jr']
        else:
            th = self.get_one_jointsol()
            if len(th) > 0:
                manipulator_jac = self.analytic_jacobian(th)
                jr = manipulator_jac[3:, :]
            else:
                print("No IK found: unable to compute rotation_err_bound\n")
                return 0
        qd = np.array(copy.deepcopy(self._des_config._q))
        Hd = self.skew_quat(qd)
        w, v = np.linalg.eig(np.dot(jr, jr.T))
        max_id = np.argmax(w)
        max_eig = w[max_id]
        max_v = v[:, max_id]
        v_vec = (1/2)*math.sqrt(self._cval*max_eig)*max_v
        q_star = qd + np.dot(v_vec.T, Hd)
        q_star = q_star/self.norm(q_star)
        qd /= self.norm(qd)
        worst_rot_err = np.arccos(np.dot(q_star, qd.T))
        return worst_rot_err, q_star

    # # Pose error bound
    # def pose_error_bound(self, peg_len=0.1):
    #     th = self.get_one_jointsol()
    #     if len(th) > 0:
    #         manipulator_jac = self.analytic_jacobian(th)
    #         return self.position_err_bound(jp=manipulator_jac[:3, :])
    #                   + peg_len * self.rotation_err_bound(jr=manipulator_jac[3:, :])
    #     else:
    #         print("No IK found: unable to compute pose_err_bound\n")
    #         return 0

    # Compute Robust IK
    # This is only for peg tip position error
    def multi_pose_err(self, config_num, peg_len):
        self.ikfast_ip_quat()
        pose_err_list = list()
        if self.solve_ikfast(config_num):
            file = open("ik_sol_config" + str(config_num) + ".txt", "r")
            for line in file:
                th = list()
                for ele in line.split(","):
                    th.append(float(ele))
                manipulator_jac = self.analytic_jacobian(th)
                position_err = self.position_err_bound(jp=manipulator_jac[:3, :])
                rotation_err, wrst_quat = self.rotation_err_bound(jr=manipulator_jac[3:, :])
                wrst_rotm = self.unitquat2rotm(wrst_quat)
                des_rotm = self.unitquat2rotm(self._des_config._q)
                ang = np.arccos(np.dot(des_rotm[:, 2], wrst_rotm[:, 2]))
                pose_err = position_err + peg_len*ang
                pose_err_list.append(pose_err)
            # min_pose_err = min(pose_err_list)
            # return min_pose_err.index(min(min_pose_err))
            return pose_err_list
        else:
            return []

    # Adjust ee_pose for IKFast input
    def ikfast_ip_quat(self):
        print("desired quat: {}".format(self._des_config._q))
        ee_rotm = self.unitquat2rotm(np.array(self._des_config._q))
        ee_position = np.array(self._des_config._p).reshape((3, 1))
        ee_pose = np.vstack((np.hstack((ee_rotm, ee_position)), np.array([0, 0, 0, 1]).reshape(1, 4)))
        ikfast_pose = np.dot(self._premult_ikfast, ee_pose)
        ikfast_pose[:3, 3] = ikfast_pose[:3, 3] - self._ikfast_exlen*ikfast_pose[:3, 2]
        ikfast_pose_flattened = ikfast_pose[:3, :4].reshape(12, ).tolist()
        file = open("ikfastinput.txt", "w")
        for i in ikfast_pose_flattened:
            file.write("%2.6f " % i)
        file.close()

    # Compute rotation matrix from a unit quaternion
    # Make sure q = [W,X,Y,Z] format
    def unitquat2rotm(self, q):
        temp_rotm = np.array([[math.pow(q[0], 2)+math.pow(q[1], 2)-math.pow(q[2], 2)-math.pow(q[3], 2), 2*q[1]*q[2]-2*q[0]*q[3], 2*q[1]*q[3]+2*q[0]*q[2]],
                              [2*q[1]*q[2]+2*q[0]*q[3], math.pow(q[0], 2)-math.pow(q[1], 2)+math.pow(q[2], 2)-math.pow(q[3], 2), 2*q[2]*q[3]-2*q[0]*q[1]],
                              [2*q[1]*q[3]-2*q[0]*q[2], 2*q[2]*q[3]+2*q[0]*q[1], math.pow(q[0], 2)-math.pow(q[1], 2)-math.pow(q[2], 2)+math.pow(q[3], 2)]])
        # Norlize columns
        for i in range(3):
            temp_rotm[:, i] /= self.norm(temp_rotm[:, i])
        return temp_rotm

    # Pick one joint solution given ee_pose
    def get_one_jointsol(self, config_num=1):
        self.ikfast_ip_quat()
        if self.solve_ikfast(config_num):
            # Read first joint solution
            file = open("ik_sol_config" + str(config_num) + ".txt", "r")
            th = list()
            for line in file:
                for ele in line.split(","):
                    th.append(float(ele))
                break
            return th
        else:
            return []

    # Solve for multiple IKs
    @staticmethod
    def solve_ikfast(config_num):
        file = open("ikfastinput.txt", "r")
        ikfast_input_list = list()
        for line in file:
            for ele in line.split():
                ikfast_input_list.append(float(ele))
        return ikModule.compIKs(config_num, ikfast_input_list)

    # Compute skew-symmetric quaternion
    @staticmethod
    def skew_quat(q):
        return np.array([[-q[1], q[0], q[3], -q[2]],
                         [-q[2], -q[3], q[0], q[1]],
                         [-q[3], q[2], -q[1], q[0]]])

    # Compute norm of a vector
    @staticmethod
    def norm(v):
        return math.sqrt(np.dot(v, v))
