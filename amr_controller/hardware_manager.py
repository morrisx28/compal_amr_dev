from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_

from Motor_Manager_origin_v1 import *
from wheel_motor_manager import *

import numpy as np
import threading

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"

NUM_MOTOR = 8 
STEERING_NUM = 4


class DDSHandler:
    def __init__(self, config_path = "config/motor_config.yaml"):
        ChannelFactoryInitialize(1, "lo")
        ## Steering motor control ##
        self.motor_manager = SteeringMotorManager(config_path)
        self.motor_manager.control_cmd.enable_motor()
        self.motor_manager.run()

        ## Wheel motor control ##
        self.wheel_motor_manager = WheelMotorManager()
        self.wheel_motor_manager.control_cmd.enable()
        self.wheel_motor_manager.start_thread()
        self.wheel_vel_scale = 20
                

        self.pub = None
        self.low_state = None

        self.is_running = True
        self.low_state_puber = ChannelPublisher("rt/lowstate", LowState_)
        self.low_state_puber.Init()  
        self.low_state_threading = threading.Thread(target=self.PublishLowState)
        self.low_state_threading.start()
        print("low_state_start")
        self.low_state = unitree_go_msg_dds__LowState_()


        self.sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.sub.Init(self.LowCmdMessageHandler, 10)

        ## Joint name sequence ##
        ## FL, FR, RR, RL 

        self.steer_motor_pos = np.zeros(4)
        self.steer_motor_kp = np.zeros(4)
        self.steer_motor_kd = np.zeros(4)

        self.wheel_motor_vel = np.zeros(4)
        self.wheel_motor_kp = np.zeros(4)
        self.wheel_motor_kd = np.zeros(4)
        

    def LowCmdMessageHandler(self, msg: LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            
            for i in range(STEERING_NUM):
                self.steer_motor_pos[i] = self.low_cmd.motor_cmd[i].q
                self.steer_motor_kp[i] = self.low_cmd.motor_cmd[i].kp
                self.steer_motor_kd[i] = self.low_cmd.motor_cmd[i].kd

                self.wheel_motor_vel[i] = self.low_cmd.motor_cmd[i+4].dq
            
            self.motor_manager.send_pos_cmd(self.steer_motor_pos, self.steer_motor_kp, self.steer_motor_kd)

            self.wheel_motor_vel = self.wheel_motor_vel * self.wheel_vel_scale
            self.wheel_motor_manager.send_vel_cmd(self.wheel_motor_vel[0], self.wheel_motor_vel[1],
                                                  self.wheel_motor_vel[2], self.wheel_motor_vel[3])

            time.sleep(0.005)

                    
    def PublishLowState(self):
        while True:
            joint_dof_pos = -np.array(self.motor_manager.joint_position)
            if joint_dof_pos is None:
                joint_dof_pos = [0.0] * 4
            
            joint_dof_vel = -np.array(self.motor_manager.joint_velocity)
            if joint_dof_vel is None:
                joint_dof_vel = [0.0] * 4

            wheel_motor_vel = self.wheel_motor_manager.motor_velocity
            if wheel_motor_vel is None:
                wheel_motor_vel = [0.0] * 4
            
            # print("wheel_motor_vel",wheel_motor_vel)

            low_state = unitree_go_msg_dds__LowState_()
            # IMU

            low_state.head[0] = 0x00
            low_state.head[1] = 0x00
            low_state.level_flag = 0x00
            low_state.imu_state.quaternion    = [0.0, 0.0, 0.0, 0.0]
            low_state.imu_state.gyroscope     = [0.0, 0.0, 0.0]
            low_state.imu_state.accelerometer = [0.0, 0.0, 0.0]

            for i in range(STEERING_NUM):
                low_state.motor_state[i].q  = float(joint_dof_pos[i])
                low_state.motor_state[i].dq = float(joint_dof_vel[i])
                low_state.motor_state[i].tau_est = 0.0

                low_state.motor_state[i+4].dq = float(wheel_motor_vel[i])

            # print(f"joint_dof_pos{joint_dof_pos}")
            self.low_state_puber.Write(low_state)
            time.sleep(1/500)
    

def main():
    hardware_manager = DDSHandler()

if __name__ == '__main__':
    main()
