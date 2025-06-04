import rospy
import time

import crtk_api 
import robot_state
import update_device_state
import ambf_raven_state
import raven_toolkits

from ambf_client import Client

class main:
    def __init__(self):
        rospy.init_node("AMBF_raven2_controller", anonymous=True)
        
        self.ambf_client = Client()
        self.ambf_client.connect()
        time.sleep(1)
        self.arm = self.ambf_client.get_obj_handle('base_link_L')

        self.robot_state = robot_state.robot_state()
        self.crtk_api = crtk_api.crtk_api(self.robot_state)
        self.update_device_state = update_device_state.update_device_state(self.robot_state, self.arm)
        self.ambf_raven_state = ambf_raven_state.ambf_raven_state(self.robot_state)
        self.raven_toolkits = raven_toolkits.raven_toolkits(self.arm)
        
        rospy.loginfo("CRTK to AMBF Adapter is connecting...")
        time.sleep(1)
        rospy.loginfo("CRTK to AMBF Adapter initialized and connected to AMBF.")
        
        return None
        
        
    def run(self):
        print("Reached Home position")
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            
            self.robot_state.update_from_ambf(self.arm)
            
            self.crtk_api.pub_measured_js()
            self.crtk_api.pub_measured_cp()
            
            self.ambf_raven_state.pub_raven_state()
            
            self.update_device_state.update_crtk_motion()

            rate.sleep()
            
        return None
    
    def home(self):
        self.raven_toolkits.set_home()
        print("Setting Home")
        return None
        

if __name__ == "__main__":
    controller = main()
    controller.home()
    controller.run()
