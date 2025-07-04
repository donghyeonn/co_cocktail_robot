import os
import time
import sys
from scipy.spatial.transform import Rotation
import rclpy
import numpy as np
import DR_init
from ..utils.base_action import BaseAction


from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from .onrobot import RG

package_path = get_package_share_directory("co_cocktail_robot")

BEVERAGE_SET = {"tequila", "red-juice", "blue-juice", "green-juice"}
GARNISH_SET = {"cherry", "lime"}


GRIPPER_NAME = "GripperDA_v2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

VELOCITY, ACCURACY = 60, 60
DR = None


class TakeAction(BaseAction):
    def __init__(self, node, poses, ingredient, target):
        DR_init.__dsr__node = node

        try:
            import DSR_ROBOT2

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return

        global DR
        DR = DSR_ROBOT2

        self.node = node
        self.take_pose = poses
        self.ingredient = ingredient
        self.target = target
        self.get_logger = node.get_logger

        self.max_retry = 3
        self.success = False

        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


        self.get_position_client = node.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()


    # 실행되는 동작
    def execute(self):
        
        DR.movej(self.take_pose["home"]["joint"], vel=VELOCITY*0.3, acc=ACCURACY)

        # 예외처리 반복
        for attempt in range(self.max_retry):
            # 초기 Detect 위치로 로봇팔 이동
            DR.movej(self.take_pose["place_detect"]["joint"], vel=VELOCITY, acc=ACCURACY)
            
            # 그리퍼 열기(open_gripper)
            self.gripper.move_gripper(1100, force_val=200)
            DR.mwait()

            # 거리 계산
            self.get_position_request.target = self.ingredient
            self.get_logger().info(f"Requesting 3D position for {self.ingredient}")
            depth_future = self.get_position_client.call_async(self.get_position_request)
            rclpy.spin_until_future_complete(self.node, depth_future)

            if depth_future.result():
                pos = depth_future.result().depth_position.tolist()
                self.get_logger().info(f"Object {self.ingredient} detected at {pos}")
                # 예외처리 위치
                # if 문으로 작성

                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                robot_posx = DR.get_current_posx()[0]
                td_coord = self.transform_to_base(pos, gripper2cam_path, robot_posx)

                if td_coord[0] and sum(td_coord) != 0:
                        td_coord[0] += 40  # DEPTH_OFFSET
                        # td_coord[1] = max(td_coord[1], 2)  # MIN_DEPTH: float = 2.0
                
                # 계산된 결과
                target_pos = list(td_coord[:3]) + robot_posx[3:]
                apporach_pos = target_pos.copy()
                grap_pos = target_pos.copy()
                apporach_pos[2] += 10


                # Detect 위치로 이동
                DR.movel(target_pos, vel=VELOCITY, acc=ACCURACY)
                DR.mwait()

                if self.ingredient == 'blue_juice':
                    grap_pos[2] -= 20
                    DR.movel(grap_pos, vel=VELOCITY, acc=ACCURACY)
                # 그리퍼 닫기(close_gripper)
                self.gripper.move_gripper(400, force_val=400)
                time.sleep(1)

                # 예외처리 체크
                status = self.gripper.get_status()
                if status[1]:  # grip detected
                    self.success = True
                    break
                else:
                    self.gripper.move_gripper(1100, force_val=200)
                    self.get_logger().warn(f"Grip not detected (attempt {attempt+1}), retrying...")

            # 위로 이동
            DR.movel(apporach_pos, vel=VELOCITY, acc=ACCURACY)
            DR.movel(self.take_pose["place_detect"]["task"], vel=VELOCITY, acc=ACCURACY*0.3)

            # # 물건 제자리로
            # DR.movel(self.take_pose[self.target]["task"], vel=VELOCITY, acc=ACCURACY)
            # # 위로 갔다가
            # DR.movel(self.take_pose[self.target]["task"], vel=VELOCITY, acc=ACCURACY)
            # DR.movej(self.take_pose["place_detect"]["joint"], vel=VELOCITY, acc=ACCURACY)
        



    def robot_control(self):
        target_list = []
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self.node, get_keyword_future)
        if get_keyword_future.result() and get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()

            target_list = get_keyword_result.message.split()

            for target in target_list:
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn("No target position")
                else:
                    self.get_logger().info(f"target position: {target_pos}")
                    self.pick_and_place_target(target_pos)
                    self.init_robot()

        else:
            self.get_logger().warning(f"{get_keyword_result.message}")
            return


    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T
    

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self.node, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                self.get_logger().warn("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = DR.get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

            target_pos = list(td_coord[:3]) + robot_posx[3:]
            return target_pos
        return None


    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        ###### 디버깅용 확인 ######
        self.get_logger().info(f"Received depth position: {td_coord[:3]}")

        return td_coord[:3]