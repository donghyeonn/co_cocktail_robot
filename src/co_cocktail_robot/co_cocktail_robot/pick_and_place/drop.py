import os
import time
import rclpy
import sys
import DR_init
from ..utils.base_action import BaseAction

from .onrobot import RG

GRIPPER_NAME = "GripperDA_v2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

VELOCITY, ACCURACY = 60, 60
DR = None

PICK_SET = {"pos4", "pos5"}
TAKE_SET = {"pos3_1", "pos3_2", "pos3_3", "pos3_4"}
SHAKER_SET = {"pos1"}
GARNISH_SET = {"pos2"}


class DropAction(BaseAction):
    def __init__(self, node, poses, target):
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

        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)



    def execute(self):
        apporach_pos = self.drop_pose[self.target]["task"].copy()
        apporach_pos[2] = self.drop_pose[self.target]["task"][2] + 50

        if self.target in PICK_SET: # 협업 구역으로
            # 경유점
            DR.movel(self.drop_pose["home"]["task"], vel=VELOCITY*0.3, acc=ACCURACY)
            DR.movel(self.drop_pose["place_detect"]["task"], vel=50, acc=25)

            # 접근
            DR.movel(apporach_pos, vel=VELOCITY, acc=ACCURACY)

            # 내려놓기
            self.place()
            DR.mwait()
            
            # 그리퍼 열기(close_gripper)
            self.gripper.move_gripper(1000, force_val=200)
            time.sleep(1)

            # 마무리 동작
            DR.movel(apporach_pos, vel=VELOCITY, acc=ACCURACY)


        elif self.target in TAKE_SET: # 선반으로
            # 놓는 위치로 이동
            DR.movej(self.drop_pose["home"]["joint"], vel=VELOCITY, acc=ACCURACY)
            DR.movel(self.drop_pose["beverage_detect"]["task"], vel=50, acc=25)

            # 접근
            DR.movel(apporach_pos, vel=VELOCITY, acc=ACCURACY)

            # 내려놓기
            self.place()
            DR.mwait()
            
            # 그리퍼 열기(close_gripper)
            self.gripper.move_gripper(1000, force_val=200)
            time.sleep(1)

            # 마무리 동작
            DR.movel(apporach_pos, vel=VELOCITY, acc=ACCURACY)


        elif self.target in GARNISH_SET:
            place_after = self.drop_pose[self.target]["task"].copy()
            place_after[2] += 50

            DR.movel(place_after, vel=VELOCITY*0.8, acc=ACCURACY)
            DR.movej(self.drop_pose[self.target]["joint"], vel=VELOCITY*0.5, acc=ACCURACY)
            DR.mwait()
            
            # 그리퍼 열기(close_gripper)
            self.gripper.move_gripper(1000, force_val=200)
            time.sleep(1)
            DR.movel(place_after, vel=VELOCITY*0.3, acc=ACCURACY)


        elif self.target in SHAKER_SET:
            place_after = self.drop_pose[self.target]["task"].copy()
            place_after[2] += 50

            # 접근
            DR.movel(place_after, vel=VELOCITY*0.8, acc=ACCURACY)

            # 내려놓기
            DR.movel(self.drop_pose[self.target]["task"], vel=VELOCITY*0.8, acc=ACCURACY)
            DR.mwait()

            # 그리퍼 열기(close_gripper)
            self.gripper.move_gripper(1000, force_val=200)
            time.sleep(1)

            # 돌아가기
            DR.movel(place_after, vel=VELOCITY*0.6, acc=ACCURACY)



    def place(self):
        DR.task_compliance_ctrl([5,5,500,100,100,100])
        time.sleep(0.1)
        DR.set_desired_force([0,0,-25,0,0,0],dir=[0,0,3,0,0,0])
        while True: # first
            if DR.check_force_condition(DR.DR_AXIS_Z,max=10):
                break
            time.sleep(0.1)
        DR.release_force()
        time.sleep(0.1)
        DR.release_compliance_ctrl()
        time.sleep(0.1)

    
