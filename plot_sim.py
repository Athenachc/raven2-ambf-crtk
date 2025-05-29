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

# Joint velocity
velocity_joint_1 = 3 # degree/s
velocity_joint_2 = 3 # degree/s
velocity_joint_3 = 0.01 # m/s
velocity_joint_4 = 10 # degree/s
velocity_joint_5 = 10 # degree/s
velocity_joint_6 = 10 # degree/s
velocity_joint_7 = 10 # degree/s


rospy.init_node('raven_keyboard_controller', anonymous=True)

controller = ambf_raven_controller.ambf_raven_controller()

# Import data from dataset
file_location = '/home/athena/Downloads/doi_10_5061_dryad_tqjq2bw84__v20241114/record_1_different_directions'
file_name = 'data_record_yz_03.csv'

df = pd.read_csv(file_location+'/'+file_name) # Load CSV file

# Joint velocity
joint_vel_1 = df.iloc[:,161]
joint_vel_2 = df.iloc[:,162]
joint_vel_3 = df.iloc[:,163]
joint_vel_4 = df.iloc[:,164]
joint_vel_5 = df.iloc[:,165]
joint_vel_6 = df.iloc[:,166]
joint_vel_7 = df.iloc[:,167]
#print(joint_vel_1)

# Joint position desired
joint_pos_desired_1 = df.iloc[:,194].values
joint_pos_desired_2 = df.iloc[:,195].values
joint_pos_desired_3 = df.iloc[:,196].values
# N1 = len(joint_pos_desired_1)
# N2 = len(joint_pos_desired_2)
# N3 = len(joint_pos_desired_3)
# # print(N1, N2, N3)

# Joint position
joint_pos_1 = df.iloc[:,13]
joint_pos_2 = df.iloc[:,14]
joint_pos_3 = df.iloc[:,15]
# n1 = len(joint_pos_1)
# n2 = len(joint_pos_2)
# n3 = len(joint_pos_3)

cmds = np.zeros((len(joint_vel_1),16))
cmds[:,1] = np.deg2rad(joint_vel_1)
cmds[:,2] = np.deg2rad(joint_vel_2)
cmds[:,3] = np.deg2rad(joint_vel_3)
cmds[:,4] = np.deg2rad(joint_vel_4)
cmds[:,5] = np.deg2rad(joint_vel_5)
cmds[:,6] = np.deg2rad(joint_vel_6)
cmds[:,7] = np.deg2rad(joint_vel_7)

# for cmd in cmds:
#     # print(cmd[1])
#     controller.pub_servo_jr_command(cmd)

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(joint_pos_desired_1, joint_pos_desired_2, joint_pos_desired_3)
plt.title('Test')
ax.set_xlabel('Desired Joint Position 1')
ax.set_ylabel('Desired Joint Position 2')
ax.set_zlabel('Desired Joint Position 3')

plt.show()