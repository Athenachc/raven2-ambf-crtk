import tty, sys, termios
import ambf_raven_controller
import rospy
import numpy as np
import tf.transformations as tft
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# filedescriptors = termios.tcgetattr(sys.stdin)
# tty.setcbreak(sys.stdin)

rospy.init_node('raven_keyboard_controller', anonymous=True)

controller = ambf_raven_controller.ambf_raven_controller()

# Import data from dataset
file_location = '/home/athena/Downloads/doi_10_5061_dryad_tqjq2bw84__v20241114/record_1_different_directions'
file_name = 'data_record_x_03.csv'

df = pd.read_csv(file_location+'/'+file_name) # Load CSV file

# Joint velocity (Columns FG,FH,FI)
joint_vel_1 = df.iloc[:,162]
joint_vel_2 = df.iloc[:,163]
joint_vel_3 = df.iloc[:,164]

# Ground truth joint position (Columns B,C,D)
joint_pos_1_gt = df.iloc[:,1] # Unit: rad
joint_pos_2_gt = df.iloc[:,2] # Unit: rad
joint_pos_3_gt = df.iloc[:,3] # Unit: m
# CONVERT ground truth joint position
joint_pos_1_gt = np.rad2deg(joint_pos_1_gt) # Unit: deg
joint_pos_2_gt = np.rad2deg(joint_pos_2_gt) # Unit: deg
joint_pos_3_gt = np.rad2deg(joint_pos_3_gt) # Unit: m

# Joint position (Columns N,O,P) 
joint_pos_1 = df.iloc[:,13] # Unit: deg
joint_pos_2 = df.iloc[:,14] # Unit: deg
joint_pos_3 = df.iloc[:,15] # Unit: m
joint_pos_1_plot = df.iloc[:,13].values 
joint_pos_2_plot = df.iloc[:,14].values 
joint_pos_3_plot = df.iloc[:,15].values 

t = df.iloc[:,0] # Recorded time

# Calculate velocities of joints 1-3 (deg/s; deg/s; m/s)
def cal_vel(joint_pos, time):
    init_joint_pos = joint_pos[0]  # Set initial joint position to the first joint pos
    init_t = time[0]  # Set initial time to the first timestamp
    velocity = []
    
    for i in range(1, len(time)):
        cur_t = time[i]
        if cur_t == init_t:
            # Handle case where the timestamps are the same
            vel = 0
        else:
            vel = (joint_pos[i] - init_joint_pos) / (cur_t - init_t)
        velocity.append(vel)
        
        init_joint_pos = joint_pos[i]
        init_t = time[i]
        
    return velocity

joint_vel_1 = cal_vel(joint_pos_1, t)
joint_vel_2 = cal_vel(joint_pos_2, t)
joint_vel_3 = cal_vel(joint_pos_3, t)

joint_vel_1_gt = cal_vel(joint_pos_1_gt, t)
joint_vel_2_gt = cal_vel(joint_pos_2_gt, t)
joint_vel_3_gt = cal_vel(joint_pos_3_gt, t)

# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# ax.plot(joint_pos_1_plot, joint_pos_2_plot, joint_pos_3_plot)
# plt.title('Test')
# ax.set_xlabel('Joint Position 1')
# ax.set_ylabel('Joint Position 2')
# ax.set_zlabel('Joint Position 3')
# plt.show()

cmds = np.zeros((len(joint_vel_1),16))
cmds[:,1] = np.deg2rad(joint_vel_1) # rad
cmds[:,2] = np.deg2rad(joint_vel_2) # rad
cmds[:,3] = joint_vel_3 # m

cmds_gt = np.zeros((len(joint_vel_1_gt),16))
cmds_gt[:,1] = np.deg2rad(joint_vel_1_gt) # rad
cmds_gt[:,2] = np.deg2rad(joint_vel_2_gt) # rad
cmds_gt[:,3] = joint_vel_3_gt # m

# # for cmd in cmds_gt:
# for cmd in cmds:
#     print("Joint 1 velocity (deg/s): "+str(cmd[1])
#           +"\nJoint 2 velocity (deg/s): "+str(cmd[2])
#           +"\nJoint 3 velocity (m/s): "+str(cmd[3]))
#     controller.pub_servo_jr_command(cmd) 

print("done")
