#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy                                                                      # ROS2 Python接口库
from rclpy.node   import Node                                                     # ROS2 节点类
from dobot_msgs_v3.srv import *   # 自定义的服务接口                                   
import time

import re

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化
        self.EnableRobot_l = self.create_client(EnableRobot,'/dobot_bringup_v3/srv/EnableRobot')
        self.DisableRobot_l = self.create_client(DisableRobot,'/dobot_bringup_v3/srv/DisableRobot') #Custum部
        self.GetPose_l = self.create_client(GetPose,'/dobot_bringup_v3/srv/GetPose')
        
        self.MovJ_l = self.create_client(MovJ,'/dobot_bringup_v3/srv/MovJ')
        self.SpeedFactor_l = self.create_client(SpeedFactor,'/dobot_bringup_v3/srv/SpeedFactor')
        self.MovL_l = self.create_client(MovL,'/dobot_bringup_v3/srv/MovL')

        self.RelMovL_l = self.create_client(RelMovL,'/dobot_bringup_v3/srv/RelMovL') #Custum部
        self.RelMovJ_l = self.create_client(RelMovJ,'/dobot_bringup_v3/srv/RelMovJ') #Custum部

        self.AccL_l = self.create_client(AccL,'/dobot_bringup_v3/srv/AccL')
        self.SpeedL_l = self.create_client(SpeedL,'/dobot_bringup_v3/srv/SpeedL')
        self.CP_l = self.create_client(CP,'/dobot_bringup_v3/srv/CP')
        self.GetInRegs_l = self.create_client(GetInRegs,'/dobot_bringup_v3/srv/GetInRegs')
        self.ModbusCreate_l = self.create_client(ModbusCreate,'/dobot_bringup_v3/srv/ModbusCreate')
        self.ModbusClose_l = self.create_client(ModbusClose,'/dobot_bringup_v3/srv/ModbusClose')

        self.DO_l = self.create_client(DO,'/dobot_bringup_v3/srv/srv/DO' )

        self.pose = 0

        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):                  # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...') 

        while not self.ModbusCreate_l.wait_for_service(timeout_sec=1.0):   #modbus用　#Custum部
            self.get_logger().info('Modbus service not available, waiting again...') 

        
                    
    def initialization(self):  # 初始化：速度、坐标系、负载、工具偏心等
        enable = EnableRobot.Request() #Custum部
        enable.load = float(0) #負荷[kg]　#Custum部
        response = self.EnableRobot_l.call_async(enable)
        print(response)
        spe = SpeedFactor.Request()
        spe.ratio = 10
        response = self.SpeedFactor_l.call_async(spe)
        print(response)

        # #modbus有効化
        # self.ModbusClose(0)#一旦確実に閉じている状態にする。
        # self.ModbusCreate("192.168.5.1",502,0,0)#接続

    def finish(self): #Custum部
        disable = DisableRobot.Request()
        response = self.DisableRobot_l.call_async(disable)
        print(response.result())
        self.ModbusClose(0)

    def ModbusCreate(self,Ip,Port,Id,Rtu):
        modbuscreate = ModbusCreate.Request()
        modbuscreate.ip =Ip
        modbuscreate.port = Port
        modbuscreate.slave_id = Id
        modbuscreate.is_rtu = Rtu
        response = self.ModbusCreate_l.call_async(modbuscreate)
        print(response)

    def ModbusClose(self,Index):
        modbusclose = ModbusClose.Request()
        modbusclose.index = Index
        response = self.ModbusClose_l.call_async(modbusclose)
        print(response)


    def reswait(self,response):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # self.get_logger().info(f"Future done: {response.done()}")# 通信待ち確認
            if response.done():  # 非同期リクエストの完了を確認
                try:
                    # リクエストの結果を取り出す
                    resresult = response.result()
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {e}")
                break
        return resresult

    def GetPose(self):
        getpose = GetPose.Request()
        getpose.user = int(0)
        getpose.tool = int(0)
        response = self.GetPose_l.call_async(getpose)

        resresult = self.reswait(response)

        self.get_logger().info(f"Response received:ok {resresult.res},Pose:{resresult.pose}")  # レスポンスのログ出力
        numbers = re.findall(r'-?\d+\.\d*',resresult.pose)
        numbers = list(map(float,numbers))
        self.pose = numbers


    def GetInRegs(self,Index,Addr,Count,Val_type):
        getinregs = GetInRegs.Request()
        getinregs.index = Index
        getinregs.addr = Addr
        getinregs.count = Count
        getinregs.val_type = Val_type
        response = self.GetInRegs_l.call_async(getinregs)

        resresult = self.reswait(response)

        return resresult


    def point(self, Move, X_j1, Y_j2, Z_j3, RX_j4, RY_j5, RZ_j6):  # 运动指令
        if Move == "MovJ": #目標位置
            P1 = MovJ.Request()
            P1.x = float(X_j1)
            P1.y = float(Y_j2)
            P1.z = float(Z_j3)
            P1.rx = float(RX_j4)
            P1.ry = float(RY_j5)
            P1.rz = float(RZ_j6)
            response = self.MovJ_l.call_async(P1)
            print(response)
        elif Move == "MovL": #手先が直線的な動き
            P1 = MovL.Request()
            P1.x = float(X_j1)
            P1.y = float(Y_j2)
            P1.z = float(Z_j3)
            P1.rx = float(RX_j4)
            P1.ry = float(RY_j5)
            P1.rz = float(RZ_j6)
            response = self.MovL_l.call_async(P1)
            print(response)
        elif Move == "RelMovL":
            P1 = RelMovL.Request()
            P1.offset1 = float(X_j1)
            P1.offset2 = float(Y_j2)
            P1.offset3 = float(Z_j3)
            P1.offset4 = float(RX_j4)
            P1.offset5 = float(RY_j5)
            P1.offset6 = float(RZ_j6)
            response = self.RelMovL_l.call_async(P1)
            print(response.result())
        elif Move == "RelMovJ":
            P1 = RelMovJ.Request()
            P1.offset1 = float(X_j1)
            P1.offset2 = float(Y_j2)
            P1.offset3 = float(Z_j3)
            P1.offset4 = float(RX_j4)
            P1.offset5 = float(RY_j5)
            P1.offset6 = float(RZ_j6)
            response = self.RelMovJ_l.call_async(P1)
            print(response.result())
        else:
            print("无该指令")

    #速度加速度のmaxからの%、加速度のmaxからの%、速度のmaxからの%、連続度合い
    def set_parameter_L(self,SpeedFactor_para,AccL_para,SpeedL_para,CP_para):#関節ごとの条件は別途か？
        #(デフォルト10,50,50,50)
        spe = SpeedFactor.Request()
        spe.ratio = SpeedFactor_para
        response = self.SpeedFactor_l.call_async(spe)
        print(response.result())
        # time.sleep(1)
        accl = AccL.Request()
        accl.r = AccL_para
        response = self.AccL_l.call_async(accl)
        print(response.result())
        time.sleep(1)
        speedl = SpeedL.Request()
        speedl.r = SpeedL_para
        response = self.SpeedL_l.call_async(speedl)
        print(response.result())
        # time.sleep(1)
        cpl = CP.Request()
        cpl.r = CP_para
        response = self.CP_l.call_async(cpl)
        print(response.result())
        time.sleep(1)

        return response

    def DO(self, index, status):  # IO 控制夹爪/气泵
        DO_V = DO.Request()
        DO_V.index = index
        DO_V.status = status
        response = self.DO_l.call_async(DO_V)
        print(response)


def main(args=None):
    rclpy.init(args=args)                                                         # ROS2 Python接口初始化
    node = adderClient("service_adder_client")                                    # 创建ROS2节点对象并进行初始化
    #node.send_request()                                                           # 发送服务请求
    node.initialization()
    node.GetPose()
    for i in range(6):
        print(node.pose[i])

    node.set_parameter_L(10,10,10,100)
    node.point("MovJ", -400, 100, 260, -180, 0, -90)  #手先移動(適当)絶対値
    node.point("MovL", -400, 50, 230, -180, 0, -90)  #手先移動(直線)絶対値
    node.set_parameter_L(50,50,50,100)
    time.sleep(3)  
    node.point("RelMovJ", 0, 0, 0, 0, 20, 0)  #ジョイントの相対位置
    node.point("RelMovL", 40, 0, 0, 0, 0, 0)  #手先の相対位置
    time.sleep(3)  
    
    node.finish()#disableをする #Custom部
    node.destroy_node()                                                           # 销毁节点对象
    rclpy.shutdown()                                                              # 关闭ROS2 Python接口
