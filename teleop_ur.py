#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from pika.sense import Sense
from pika.gripper import Gripper
from tools import MATHTOOLS
# UR机械臂控制
from ur_control import URCONTROL
import logging
import threading

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('teleop_ur')

class PIKATELEOP():
    def __init__(self):
        self.tools = MATHTOOLS()
        self.ur_control = URCONTROL("192.168.1.15")
        self.pika_to_arm = [0,0,0,   0, 1.570796, 0]
        
        self.target_device = "T20"
        
        self.initial_pose_rotvec = self.ur_control.get_tcp_pose()
        temp_rotvec = [self.initial_pose_rotvec[3],self.initial_pose_rotvec[4],self.initial_pose_rotvec[5]]

        # 将旋转向量转换成欧拉角
        roll, pitch, yaw = self.tools.rotvec_to_rpy(temp_rotvec)
        # 创建副本
        self.initial_pose_rpy = self.initial_pose_rotvec[:]
        
        self.initial_pose_rpy[3] = roll
        self.initial_pose_rpy[4] = pitch
        self.initial_pose_rpy[5] = yaw

        # 连接到 Sense 设备
        self.sense = Sense('/dev/ttyUSB0')
        if not self.sense.connect():
            print("连接 Pika Sense 设备失败，请检查设备连接和串口路径")
            return
        print("成功连接到 Pika Sense 设备")
    
        # 连接到 Gripper 设备
        self.gripper = Gripper('/dev/ttyUSB1')
        if not self.gripper.connect():
            print("连接 Pika Gripper 设备失败，请检查设备连接和串口路径")
            return
        print("成功连接到 Pika Gripper 设备")
          
        # 启用电机
        print("\n正在启用电机...")
        if self.gripper.enable():
            print("电机启用成功")
        else:
            print("电机启用失败")
            
        self.running = True
        self.flag = False

        # 单机械臂的初始位置
        self.base_pose = self.initial_pose_rpy #想要的目标姿态数据
        
        # 这里是为了防止程序启动时，pika_pose 数据为空的时候，程序执行失败
        self.x, self.y, self.z = self.initial_pose_rpy[0],self.initial_pose_rpy[1],self.initial_pose_rpy[2]
        self.roll, self.pitch, self.yaw = self.initial_pose_rpy[3],self.initial_pose_rpy[4],self.initial_pose_rpy[5]
        
        # 初始化tracker线程
        self.tracker_thread = threading.Thread(target=self.get_tracker_pose)
        self.tracker_thread.daemon = True # 设置线程为守护线程，当主程序退出时，线程也会退出
        
        self.last_value = None 
        self.bool_trigger = False
        self.takeover = False
        
    def control_gripper(self):
        # 获取编码器数据
        encoder_data = self.sense.get_encoder_data()
        # 将sense弧度直接发送给gripper
        self.gripper.set_motor_angle(encoder_data['rad'])
    
    def handle_trigger(self):
        current_value = self.sense.get_command_state()
        
        if self.last_value is None:
            self.last_value = current_value
        if current_value != self.last_value: # 检测到状态改变
            self.bool_trigger = not self.bool_trigger # 反转 bool_trigger
            self.last_value =  current_value # 更新 last_value
            # 根据新的 bool_trigger 值执行相应的操作
            if self.bool_trigger :
                self.base_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
                self.flag = True
                print("开始遥操作")
            
            elif not self.bool_trigger :
                self.flag = False
                self.initial_pose_rotvec = self.ur_control.get_tcp_pose()
                
                temp_rotvec = [self.initial_pose_rotvec[3], self.initial_pose_rotvec[4], self.initial_pose_rotvec[5]]
                
                # 将旋转向量转换成欧拉角
                roll, pitch, yaw = self.tools.rotvec_to_rpy(temp_rotvec)
                
                self.initial_pose_rpy = self.initial_pose_rotvec[:]
                self.initial_pose_rpy[3] = roll
                self.initial_pose_rpy[4] = pitch
                self.initial_pose_rpy[5] = yaw
                
                self.base_pose = self.initial_pose_rpy # 想要的目标姿态数据
                print("停止遥操")

    # 增量式控制
    def calc_pose_incre(self,base_pose, pose_data):
        begin_matrix = self.tools.xyzrpy2Mat(base_pose[0], base_pose[1], base_pose[2],
                                                    base_pose[3], base_pose[4], base_pose[5])
        zero_matrix = self.tools.xyzrpy2Mat(self.initial_pose_rpy[0],self.initial_pose_rpy[1],self.initial_pose_rpy[2],
                                            self.initial_pose_rpy[3],self.initial_pose_rpy[4],self.initial_pose_rpy[5])
        end_matrix = self.tools.xyzrpy2Mat(pose_data[0], pose_data[1], pose_data[2],
                                                pose_data[3], pose_data[4], pose_data[5])
        result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
        xyzrpy = self.tools.mat2xyzrpy(result_matrix)
        return xyzrpy   
        
    # 调整矩阵函数
    def adjustment(self,x,y,z,Rx,Ry,Rz):
        transform = self.tools.xyzrpy2Mat(x,y,z,   Rx, Ry, Rz)

        r_adj = self.tools.xyzrpy2Mat(self.pika_to_arm[0],self.pika_to_arm[1],self.pika_to_arm[2],
                                      self.pika_to_arm[3],self.pika_to_arm[4],self.pika_to_arm[5],)   # 调整坐标轴方向  pika--->机械臂末端

        transform = np.dot(transform, r_adj)
        
        x_,y_,z_,Rx_,Ry_,Rz_ = self.tools.mat2xyzrpy(transform)
        
        return x_,y_,z_,Rx_,Ry_,Rz_
    
    def get_tracker_pose(self):
        
        # 循环获取WM0设备的位姿数据
        logger.info(f"开始获取{self.target_device}设备的位姿数据...")
        while True:
            # 获取WM0设备的位姿数据
            pose = self.sense.get_pose(self.target_device)
            if pose:
                # 提取位置和旋转数据用于进一步处理
                position = pose.position  # [x, y, z]
                rotation = self.tools.quaternion_to_rpy(pose.rotation[0],pose.rotation[1],pose.rotation[2],pose.rotation[3])  # [x, y, z， w] 四元数
                
                self.x,self.y,self.z,   self.roll, self.pitch, self.yaw = self.adjustment(position[0],position[1],position[2],
                                                                                          rotation[0],rotation[1],rotation[2])                                                                           
            else:
                
                logger.warning(f"未能获取{self.target_device}的位姿数据，等待下一次尝试...")
                
            time.sleep(0.02)  # 每0.02秒获取一次（50Hz）

    def start(self):
        self.tracker_thread.start() # 启动线程        
        # 主线程继续执行其他任务
        while self.running:
            self.handle_trigger()
            self.control_gripper()
            current_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            increment_pose = self.calc_pose_incre(self.base_pose,current_pose)
            
            finally_pose  = self.tools.rpy_to_rotvec(increment_pose[3], increment_pose[4], increment_pose[5])
            
            increment_pose[3:6] = finally_pose
            
            #下发pose至机械臂
            if self.flag:
                self.ur_control.sevol_l(increment_pose)
                
            time.sleep(0.02) # 50Hz更新
                
    def stop(self):
        self.running = False 
        if self.tracker_thread.is_alive():
            self.tracker_thread.join() # 等待tracker线程结束

if __name__ == "__main__":
    
    system = PIKATELEOP()
    try:
        system.start()
    except KeyboardInterrupt:
        system.stop()
        print("程序已退出")

    
    