import time
import rclpy
import DR_init
from ..utils.base_action import BaseAction

VEL, ACC = 100, 100
DR = None

class HeadTiltAction(BaseAction):
    def __init__(self, node, poses=None, target=None):
        DR_init.__dsr__node = node

        try:
            import DSR_ROBOT2

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return

        global DR
        DR = DSR_ROBOT2

        self.drop_pose = poses
        self.target = target


    def execute(self):
        DR.movej([0,0,0,0,0,-15], mod=DR.DR_MV_MOD_REL, vel=VEL, acc=ACC)
        DR.movej([0,0,0,0,0,30], mod=DR.DR_MV_MOD_REL, vel=VEL, acc=ACC)
        DR.movej([0,0,0,0,0,-15], mod=DR.DR_MV_MOD_REL, vel=VEL, acc=ACC)
