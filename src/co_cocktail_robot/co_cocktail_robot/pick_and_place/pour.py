import time
from rclpy.node import Node
import rclpy

from dsr_msgs2.srv import SetRobotMode
import DR_init
from ..utils.base_action import BaseAction
from .onrobot import RG

VELOCITY, ACCURACY = 100, 60
ON, OFF = 1, 0
DR = None
GRIPPER_NAME = "rg2"
TOOLCHANGER_IP = "192.168.1.1"
TOOLCHANGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)
# PourAction(arm, "tequila", 50, shaker, poses["pour_tequila"])

class PourAction(BaseAction):
    def __init__(self, node, poses, target, on_hand):
        DR_init.__dsr__node = node
        self._node = node
        try:
            import DSR_ROBOT2

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return

        global DR
        DR = DSR_ROBOT2
    
        self.target = target
        self.poses = poses
        self.on_hand = on_hand
        self.cli = node.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('서비스 기다리는 중: /dsr01/system/set_robot_mode ...')

        self.req = SetRobotMode.Request()

        self.req.robot_mode = 0

    def execute(self):
        self._node.get_logger().info("pour start")
        pour_pose = self.poses[self.target]
        # ==========================================================================================
        # 툴 변경
        DR.set_robot_mode(0)
        
        if self.on_hand in ['tequila', 'blue-juice', 'red-juice', 'green-juice']:
            DR.set_tcp('grasp bottle')
        else:
            self._node.get_logger().warning("wrong object on hand")
            raise KeyError("wrong object on hand")

        if self.target == 'pos1':
            fd = [0,1,-1,5,0,0]
            dir = [0,1,1,1,0,0]
        elif self.target == 'pos2':
            fd = [0,1,1,-5,0,0]
            dir = [0,1,1,1,0,0]
        else:
            self._node.get_logger().warning("wrong target on hand")
            raise KeyError("wrong target on hand")

        DR.movel(pour_pose["task"], vel=VELOCITY, acc = ACCURACY)
        
        DR.wait(1)
        DR.set_robot_mode(0)

        # 힘제어 붓기
        DR.task_compliance_ctrl([20000,20000,20000,10,10,10])
        time.sleep(0.1)
        DR.set_desired_force(fd=fd,dir=dir,mod=DR.DR_FC_MOD_ABS)
        DR.wait(20) 
        DR.release_force()
        DR.release_compliance_ctrl()

        DR.movel(pour_pose["task"], vel=VELOCITY, acc = ACCURACY)
        
        #============================================================================================ 
        # 툴 변경
        time.sleep(1.0)
        DR.set_tcp("GripperDA_v2")
        DR.set_robot_mode(1)
        # ==========================================================================================
        
    
    def grasp(self):
        gripper.move_gripper(600,400)
        DR.wait(1)

    def release(self):
        gripper.move_gripper(800,400)
        DR.wait(1)
