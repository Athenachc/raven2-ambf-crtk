import rospy
import numpy as np

import sensor_msgs.msg
import geometry_msgs.msg

import tf.transformations as tft

class crtk_api:
    def __init__(self, robot_state):
        self.robot_state = robot_state
        self.__init_pub_sub()
        return None

    def __init_pub_sub(self):
        self.__publisher_measured_js = rospy.Publisher('/arm1/measured_js', sensor_msgs.msg.JointState, latch = True, queue_size = 1)
        self.__publisher_measured_cp = rospy.Publisher('/arm1/measured_cp', geometry_msgs.msg.TransformStamped, latch = True, queue_size = 1)
        self.__subscriber_servo_jr = rospy.Subscriber('/arm1/servo_jr', sensor_msgs.msg.JointState, self.__callback_servo_jr)
        self.__subscriber_servo_cr = rospy.Subscriber('/arm1/servo_cr', geometry_msgs.msg.TransformStamped, self.__callback_servo_cr)
        return None
         
    def pub_measured_js(self):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.position[:] = self.robot_state.joint_positions
        msg.velocity[:] = self.robot_state.joint_velocities
        msg.effort[:] = self.robot_state.joint_efforts
        
        self.__publisher_measured_js.publish(msg)
        return None
    
    def pub_measured_cp(self):
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "end_effector"
        
        msg.transform.translation.x = self.robot_state.cartesian_pose[0, 3]
        msg.transform.translation.y = self.robot_state.cartesian_pose[1, 3]
        msg.transform.translation.z = self.robot_state.cartesian_pose[2, 3]
        
        quaternion = tft.quaternion_from_matrix(self.robot_state.cartesian_pose)
        msg.transform.rotation.x = quaternion[0]
        msg.transform.rotation.y = quaternion[1]
        msg.transform.rotation.z = quaternion[2]
        msg.transform.rotation.w = quaternion[3]
        
        self.__publisher_measured_cp.publish(msg)
        return None
    
    def __callback_servo_jr(self, msg):
        if msg is not None:
            if not self.robot_state.is_busy:
                self.robot_state.control_mode = "JOINT"
                self.robot_state.motion_state = "MOVING"
                self.robot_state.joint_delta[:] = msg.position[0:7]
        return None
    
    
    def __callback_servo_cr(self, msg):
        if msg is not None:
            if not self.robot_state.is_busy:
                self.robot_state.control_mode = "CARTESIAN"
                self.robot_state.motion_state = "MOVING"
                
                self.robot_state.delta_pos[:] = [msg.transform.translation.x, 
                                                 msg.transform.translation.y, 
                                                 msg.transform.translation.z]
                    
                self.robot_state.delta_ori[:] = [msg.transform.rotation.x,
                                                 msg.transform.rotation.y,
                                                 msg.transform.rotation.z,
                                                 msg.transform.rotation.w]
        return None