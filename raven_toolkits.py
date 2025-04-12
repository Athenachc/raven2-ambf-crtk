from utils import ambf_raven_def as ard

class raven_toolkits:
    def __init__(self, arm):
        self.arm = arm
        return None
    
    def set_home(self):
        for i in range(7):
            self.arm.set_joint_pos(i, ard.HOME_JOINTS[i])
        return None
        