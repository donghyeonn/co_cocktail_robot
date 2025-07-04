import time
import rclpy
import DR_init
from ..utils.base_action import BaseAction

VEL, ACC = 100, 100
DR = None

class HeadNodAction(BaseAction):
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
        DR.movej([-26.71, -17.56, 96.51, 25.36, 58.04, 55.84], vel=VEL, acc=ACC)
        DR.movej([-26.71, -18.13, 96.35, 29.95, 50.87, 55.84], vel=VEL, acc=ACC)
        DR.movej([-26.71, -17.56, 96.51, 25.36, 58.04, 55.84], vel=VEL, acc=ACC)
        DR.movej([-26.71, -18.13, 96.35, 29.95, 50.87, 55.84], vel=VEL, acc=ACC)
        DR.movej([-26.71, -17.56, 96.51, 25.36, 58.04, 55.84], vel=VEL, acc=ACC)