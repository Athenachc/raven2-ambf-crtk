import tty, sys, termios
import ambf_raven_controller_new
import rospy
import numpy as np
import tf.transformations as tft
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# filedescriptors = termios.tcgetattr(sys.stdin)
# tty.setcbreak(sys.stdin)

rospy.init_node('raven_keyboard_controller', anonymous=True)

controller = ambf_raven_controller_new.ambf_raven_controller()

# Import data from dataset
file_location = '/home/athena/Downloads/doi_10_5061_dryad_tqjq2bw84__v20241114/record_1_different_directions'
file_name = 'data_record_x_03.csv'

df = pd.read_csv(file_location+'/'+file_name) # Load CSV file

# Joint velocity
joint_vel_1 = df.iloc[:,162]
joint_vel_2 = df.iloc[:,163]
joint_vel_3 = df.iloc[:,164]
joint_vel_4 = df.iloc[:,165]
joint_vel_5 = df.iloc[:,166]
joint_vel_6 = df.iloc[:,167]
joint_vel_7 = df.iloc[:,168]
#print(joint_vel_1)

# Joint position desired
joint_pos_desired_1 = df.iloc[:,194]
joint_pos_desired_2 = df.iloc[:,195]
joint_pos_desired_3 = df.iloc[:,196]
# # print(N1, N2, N3)

# Joint position
joint_pos_1 = df.iloc[:,13].values
joint_pos_2 = df.iloc[:,14].values
joint_pos_3 = df.iloc[:,15].values


fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(joint_pos_1, joint_pos_2, joint_pos_3)
plt.title('Test')
ax.set_xlabel('Joint Position 1')
ax.set_ylabel('Joint Position 2')
ax.set_zlabel('Joint Position 3')
plt.show()

cmds = np.zeros((len(joint_vel_1),16))
cmds[:,1] = np.deg2rad(joint_vel_1)
cmds[:,2] = np.deg2rad(joint_vel_2)
cmds[:,3] = joint_vel_3
cmds[:,4] = np.deg2rad(joint_vel_4)
cmds[:,5] = np.deg2rad(joint_vel_5)
cmds[:,6] = np.deg2rad(joint_vel_6)
cmds[:,7] = np.deg2rad(joint_vel_7)

datas = np.zeros(len(joint_vel_1),8)

# for cmd in cmds:
#     # print(cmd[1])
#     controller.pub_servo_jr_command(cmd) 

for data in datas:
    print(data[1])
    # controller.pub_joint_position_trajectory(data)
