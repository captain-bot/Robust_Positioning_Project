from robot_kinemtics import RobotKin as Rk
from robot_kinemtics import PoseErrBound as Pb

# inputs
std = 0.0045
num_std = 3
th = [-0.0619, -0.9151, -0.5675, 1.7231, 2.0478, -1.5560, 0.4143]
end_link = "left_gripper"
# pd = [0.6165, 0.0774, 0.4025]
# qd = [0.6838, 0.7173, 0.0799, -0.1064]
pd = [1.082, 0.315, 0.189]
qd = [0.034, 0.833, 0.060, 0.548]

# Use of RobotKin class
a = Rk(end_link)
trans_mat = a.forward_kin(th)
jac = a.analytic_jacobian(th)
print("------------------------------")
print("All transformation upto end frame: ")
print(trans_mat)
print("Manipulator jacobian: ")
print(jac)
print("------------------------------")


# Use of PoseErrBound class
b = Pb(end_link, num_std, std, pd, qd)
wrst_position_err = b.position_err_bound()
wrst_rotation_err = b.rotation_err_bound()
wrst_pose_err = b.pose_error_bound(peg_len=0.09)
b.ikfast_ip_quat()

print("------------------------------")
print("Position error bound: {} m".format(wrst_position_err))
print("Rotation error bound: {} rad".format(wrst_rotation_err))
print("Pose error bound: {} m".format(wrst_pose_err))
print("------------------------------")
