from ..utils.base_action import BaseAction
import DR_init
import time
from .onrobot import RG
#from ..utils.base_action import BaseAction
VELOCITY, ACCURACY = 100,60
ON, OFF = 1, 0
DR = None
GRIPPER_NAME = "rg2"
TOOLCHANGER_IP = "192.168.1.1"
TOOLCHANGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)

class TumblerAction(BaseAction):
    def __init__(self,node, poses, move):     # pose_dict로 location.yaml 파일을 불러옴
        self.move = move
        self._node = node
        DR_init.__dsr__node = node
        print('initialized')
        global DR
        try:
            import DSR_ROBOT2 as DR 

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return
        self.tumbler_pose = poses
        

    def execute(self):
        self._node.get_logger().info("open/close start")

        tumbler_cover_up = self.tumbler_pose["cover_top"]["task"].copy()
        tumbler_mouth_up = self.tumbler_pose["mouth_close"]["task"].copy()
        tumbler_mouth_up_open = self.tumbler_pose["mouth_open"]["task"].copy()

        tumbler_cover_up[2] = 250
        tumbler_mouth_up[2] = 250
        tumbler_mouth_up_open[2] = 250
        if self.move == 'close':
            DR.movel(pos=tumbler_cover_up,vel=VELOCITY, acc = ACCURACY) #pos
            DR.movel(pos=self.tumbler_pose["cover_top"]["task"],vel=VELOCITY, acc = ACCURACY) #pos
            self.grasp()
            DR.movel(pos=tumbler_cover_up,vel=VELOCITY, acc = ACCURACY,radius=20) #pos
            DR.movel(pos=tumbler_mouth_up,vel=VELOCITY,acc=ACCURACY,radius=20)
            DR.movel(pos=self.tumbler_pose["mouth_close"]["task"],vel = VELOCITY,acc = ACCURACY) #pos
            self.force_down()
            self.spin()
            self.release()
            print('stop spin')
            DR.movel(pos=tumbler_mouth_up,vel=VELOCITY,acc=ACCURACY)

        elif self.move == 'open':
            DR.movel(pos=tumbler_mouth_up,vel = VELOCITY,acc = ACCURACY) #poss
            DR.movel(pos=self.tumbler_pose["mouth_open"]["task"],vel = VELOCITY,acc = ACCURACY) #poss
            self.grasp()
            print('start respin')
            self.spin()
            DR.wait(1)
            print('stop respin')
            DR.movel(pos=tumbler_mouth_up_open,vel=VELOCITY,acc=ACCURACY,radius=20)
            DR.movel(pos=tumbler_cover_up,vel=VELOCITY, acc = ACCURACY,radius=20) #pos
            DR.movel(pos=self.tumbler_pose["cover_grasp"]["task"],vel=VELOCITY, acc = ACCURACY) #pos
            self.force_down()
            self.release()
            DR.movel(pos=tumbler_cover_up,vel=VELOCITY, acc = ACCURACY,radius=20) #pos

        else:
            self._node.get_logger().warning('move값 에러')
            raise KeyError("이 키는 존재하지 않습니다!")
        
    
    def grasp(self):
        gripper.move_gripper(550,400)
        DR.wait(1)

    def release(self):
        gripper.move_gripper(800,400)
        DR.wait(1)

    def spin(self):
        if self.move == 'open':
            fd = [0,-7.5,0,0,0,-15]
        if self.move == 'close':
            fd = [0,7.5,-3,0,0,15]
        dir = [0,1,1,0,0,1]
        DR.wait(0.1)
        init_posx = DR.get_current_posx()
        init_posx = list(init_posx)[0]
        complete = False

        if self.move == 'close':
            DR.task_compliance_ctrl([100,100,50,100,100,10])
            time.sleep(0.1)
            DR.set_desired_force(fd=[0,-7.5,-1,0,0,-15],dir=dir)
            angle_limit = False
            while not angle_limit: 
                angle_limit = DR.check_orientation_condition(DR.DR_AXIS_C,min=-40,max=40,ref=DR.DR_BASE,mod=DR.DR_MV_MOD_REL,pos=init_posx)
                if self.check_complete_condition(init_posx=init_posx):
                    complete = True
                    print('force detected')
                    break
                DR.wait(0.1)
            DR.release_force()
            time.sleep(0.1)
            DR.release_compliance_ctrl()

        while True:
            angle_limit = False
            DR.task_compliance_ctrl([100,100,50,100,100,10])
            time.sleep(0.1)
            DR.set_desired_force(fd=fd,dir=dir)
            while not angle_limit: 
                angle_limit = DR.check_orientation_condition(DR.DR_AXIS_C,min=-60,max=60,ref=DR.DR_BASE,mod=DR.DR_MV_MOD_REL,pos=init_posx)
                if self.check_complete_condition(init_posx=init_posx):
                    complete = True
                    print('force detected')
                    break
                DR.wait(0.1)
            DR.release_force()
            time.sleep(0.1)
            DR.release_compliance_ctrl()
            if complete:
                break
            self.release()

            changed_height = DR.get_current_posx()
            changed_height = list(changed_height)[0][2]

            changed_pos = init_posx.copy()
            changed_pos[2] = changed_height

            DR.wait(0.1)
            # DR.movej(init_posj,vel=VELOCITY,acc=ACCURACY)
            DR.movel(changed_pos,vel=VELOCITY,acc=ACCURACY)

            
            self.grasp()
    
    def check_complete_condition(self,init_posx):
        if self.move == "close":
            return DR.check_force_condition(axis=DR.DR_AXIS_C,min=0,max=1,ref=DR.DR_BASE)
         
        elif self.move == "open":
            print(init_posx[2]-list(DR.get_current_posx())[0][2])
            return DR.check_position_condition(axis=DR.DR_AXIS_Z,min=-2,max=11.5,ref=DR.DR_BASE,mod=DR.DR_MV_MOD_REL,pos=init_posx)
        
    def force_down(self):
        DR.task_compliance_ctrl([10,10,3000,500,500,500])
        time.sleep(0.1)
        fd = [0,0,-10,0,0,0]
        dir = [0,0,1,0,0,0]
        DR.set_desired_force(fd=fd,dir=dir)
        while True:
            if DR.check_force_condition(axis=DR.DR_AXIS_Z,max=10):
                DR.wait(0.1)
                break
            DR.wait(0.1)
        DR.release_force()
        time.sleep(0.1)
        DR.release_compliance_ctrl()