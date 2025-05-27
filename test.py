import tty, sys, termios
import ambf_raven_controller
import rospy
import numpy as np
import tf.transformations as tft

# filedescriptors = termios.tcgetattr(sys.stdin)
# tty.setcbreak(sys.stdin)

velocity_joint_1 = 10 # degree/s
velocity_joint_2 = 10 # degree/s
velocity_joint_3 = 0.1 # m/s
velocity_joint_4 = 100 # degree/s
velocity_joint_5 = 50 # degree/s
velocity_joint_6 = 50 # degree/s
velocity_joint_7 = 50 # degree/s


rospy.init_node('raven_keyboard_controller', anonymous=True)

controller = ambf_raven_controller.ambf_raven_controller()


for i in range(30):
    cmd = np.zeros((16))
    cmd[1] = np.deg2rad(velocity_joint_1)
    # cmd[2] = np.deg2rad(velocity_joint_2)
    # cmd[3] = velocity_joint_3
    # cmd[4] = np.deg2rad(velocity_joint_4)
    # cmd[5] = np.deg2rad(velocity_joint_5)
    # cmd[6] = np.deg2rad(velocity_joint_6)
    # cmd[7] = np.deg2rad(velocity_joint_7)
    print(cmd)
    controller.pub_servo_jr_command(cmd)


# pos = np.zeros((3))
# rot = tft.quaternion_from_euler(0, 0, 0.01)
# controller.pub_servo_cr_command(pos, rot)

# # for i in range(10):
# #     print("Moving Joint 2 --")
# #     cmd = np.zeros((16))
# #     #print(cmd)
# #     cmd[2] = -np.deg2rad(velocity_joint_2)
# #     controller.pub_servo_cr_command(cmd)