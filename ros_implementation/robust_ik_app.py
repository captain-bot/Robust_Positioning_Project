from robot_kinemtics import PoseErrBound as Pb
import numpy as np
import csv

FILE_PATH = "/home/nobug-ros/dev_research/Robust_Positioning_Project/ros_implementation/"
FILE_NAME = "datafile.csv"
RES_FILE_NAME = "result2.csv"
config_num = 0

w = csv.writer(open(FILE_PATH + RES_FILE_NAME, 'a'))
w.writerow(['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw', 'max_pose_err', 'min_pose_err'])

with open(FILE_PATH + FILE_NAME, "r") as f:
    reader = csv.reader(f)
    for row in reader:
        print(row)
        pd = list()
        qd = list()
        count = 0
        if config_num > 0:
            for ele in row:
                if count < 3:
                    pd.append(float(ele))
                else:
                    qd.append(float(ele))
                count += 1
            b = Pb(end_link="left_gripper", knum=3, sig=0.0045, p=pd, q=qd)
            err_list = b.multi_pose_err(config_num, peg_len=0.009)
            if len(err_list) > 0:
                print("Minimum Error is {}m at IK num {}".format(np.min(err_list), np.argmin(err_list)))
                print("Maximum Error is {}m at IK num {}".format(np.max(err_list), np.argmax(err_list)))
                result = pd + qd
                result.append(np.max(err_list))
                result.append(np.min(err_list))
                w.writerow(result)
            else:
                print("Error can not be found for the given pose")
            print("=================================")
            del b
        config_num += 1
        # if config_num > 1:
        #     break

