from robot_kinemtics import PoseErrBound as Pb
import numpy as np
import time

# pd = [0.6165, 0.077, 0.4025]
# qd = [0.6839, 0.7174, 0.0799, -0.1064]
pd = [0.756, -0.013, 0.537]
qd = [0.6839, 0.7174, 0.0799, -0.1064]
config_num = 1

b = Pb(end_link="left_gripper", knum=3, sig=0.0100, p=pd, q=qd)
start_time = time.time()
err_list = b.multi_pose_err(config_num, peg_len=0.100)

if len(err_list) > 0:
    print("Minimum Error is {}m at IK num {}".format(np.min(err_list), np.argmin(err_list)))
    print("Maximum Error is {}m at IK num {}".format(np.max(err_list), np.argmax(err_list)))
else:
    print("Error can not be found for the given pose")
print("=================================")
del b
end_time = time.time()
print("Time elapsed: {}seconds".format(end_time-start_time))
