from .onrobot import RG
from ..utils.base_action import BaseAction
import DR_init
import time
ON, OFF = 1, 0
DR = None
VEL, ACC = 100, 50

GRIPPER_NAME = "rg2"
TOOLCHANGER_IP = "192.168.1.1"
TOOLCHANGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)

class ShakerAction(BaseAction):
    def __init__(self, node, poses):
        DR_init.__dsr__node = node
        self.poses = poses
        global DR

        try:
            import DSR_ROBOT2 as DR
        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return

    def execute(self):
        shaking_poses = self.poses['home']['task']
        shaker_pick = self.poses['shaker_pick']['task']
        shaker_pick_before = self.poses['shaker_pick']['task'].copy()
        shaker_pick_before[2] += 150

        DR.movel(shaker_pick_before,vel=VEL,acc=ACC)
        DR.movel(shaker_pick,vel=VEL,acc=ACC)
        self.grasp()
        DR.movel(shaker_pick_before,vel=VEL,acc=ACC,radius=50)
        DR.movel(shaking_poses,vel=VEL,acc=ACC,ref=DR.DR_BASE)
        for _ in range(2):
            moving_pose = [0,20,20,0,0,60]
            DR.amovel(moving_pose,vel=VEL,acc=ACC,ref=DR.DR_TOOL,mod=DR.DR_MV_MOD_REL)
            DR.wait(0.45)
        for _ in range(2):
            moving_pose = [0,-20,-20,0,0,-60]
            DR.amovel(moving_pose,vel=VEL,acc=ACC,ref=DR.DR_TOOL,mod=DR.DR_MV_MOD_REL)
            DR.wait(0.45)
        
        DR.movel(shaking_poses,vel=VEL,acc=ACC)
        
        DR.movel(shaker_pick_before,vel=VEL,acc=ACC)
        DR.movel(shaker_pick,vel=VEL,acc=ACC)
        self.release()
        DR.movel(shaker_pick_before,vel=VEL,acc=ACC)
        


    def grasp(self):
        gripper.move_gripper(600,400)
        DR.wait(1) 

    def release(self):
        gripper.move_gripper(800,400)
        DR.wait(1)

        