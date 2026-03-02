import time
import sys
import numpy as np
import threading
import traceback
import yaml
import matplotlib.pyplot as plt
import csv
from math import atan2, sqrt, pi, atan

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import struct
import gamepad

NUM_MOTORS = 8

class Controller:
    def __init__(self):


        config_file = 'compal_amr.yaml'
        with open(f"{config_file}", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.dt = config["dt"]
            self.cmd_scale = config["cmd_scale"]

            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)

            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            self.w_base = config["wheelbase"]
            self.t_width = config["trackwidth"]
            
            self.cmd_init = np.array(config["cmd_init"], dtype=np.float32)

        self.L1 = 0.19
        self.pad = gamepad.control_gamepad( 3, [-1.0, 1.0], [-1.0, 1.0], [-3.14, 3.14], [1.0, 1.0, 3.14, 0.05])
        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  


        self.controller_rt = 0.0
        self.is_running = False

        # thread handling
        self.lowCmdWriteThreadPtr = None

        # state
        self.target_dof_pos = self.default_angles.copy()
        self.target_dof_vel = np.zeros(NUM_MOTORS)
        self.qpos = np.zeros(NUM_MOTORS, dtype=np.float32)
        self.qvel = np.zeros(NUM_MOTORS, dtype=np.float32)
        self.qtau = np.zeros(NUM_MOTORS, dtype=np.float32)
        self.quat = np.zeros(4) # q_w q_x q_y q_z
        self.ang_vel = np.zeros(3)

        # Wheel order: FL, FR, RL, RR
        self.wheel_positions = {
            'FL': [self.w_base/2, self.t_width/2],
            'FR': [self.w_base/2, -self.t_width/2],
            'RL': [-self.w_base/2, self.t_width/2],
            'RR': [-self.w_base/2, -self.t_width/2],
        }

        self.r_bw = 0.39
        self.wheel_radius = 0.06

        self.x_ekf = np.zeros(6, dtype=float)  # x, y, theta, vx, vy, wz
        
        self.P_ekf = np.diag([0.5**2, 0.5**2, (5.0*np.pi/180)**2, 0.2**2, 0.2**2, (5.0*np.pi/180)**2])

       
        self.Q_base = np.diag([
            1e-4,             # x
            1e-4,             # y
            (0.5*np.pi/180)**2,  # theta
            5e-3,             # vx
            5e-3,             # vy
            (1.0*np.pi/180)**2   # wz
        ])

        
        self.R_enc = np.diag([
            0.05**2,  # vx_enc (m/s)
            0.05**2,  # vy_enc (m/s)
            (2.0*np.pi/180)**2  # wz_enc (rad/s)
        ])

        self.mode = ''

        self.crc = CRC()

    # Public methods
    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        # Init default pos #
        self.Start()

        print("Initial Sucess !!!")

    def get_gravity_orientation(self, quaternion):
        qw = quaternion[0]
        qx = quaternion[1]
        qy = quaternion[2]
        qz = quaternion[3]

        gravity_orientation = np.zeros(3)

        gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
        gravity_orientation[1] = -2 * (qz * qy + qw * qx)
        gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

        return gravity_orientation


    def Start(self):
        self.is_running = True
        self.lowCmdWriteThreadPtr = threading.Thread(target=self.LowCmdWrite)
        self.lowCmdWriteThreadPtr.start()

    def ShutDown(self):
        self.is_running = False
        self.lowCmdWriteThreadPtr.join()

    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(NUM_MOTORS):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= self.default_angles[i]
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        self.update_state()
    
    def normalize(self, angle, speed):
        angle = ((angle + pi) % (2*pi)) - pi

        if angle > pi/2:
            angle -= pi
            speed *= -1
        elif angle < -pi/2:
            angle += pi
            speed *= -1

        return angle, speed
    
    def wheelchair_to_base_vel(self, vx_w, vy_w, wz):
        vx_b = vx_w
        vy_b = vy_w + wz * self.r_bw
        return vx_b, vy_b, wz
    
    def compute(self, cmd_vel):
        vx_w, vy_w, wz = cmd_vel
        # Only use planar control
        vx_b, vy_b, w_z = self.wheelchair_to_base_vel(vx_w, vy_w, wz)

        wheel_angles = []
        wheel_speeds = []

        for name in ['FL', 'FR', 'RL', 'RR']:
            x_offset, y_offset = self.wheel_positions[name]

            # Relative rotational velocity at wheel position
            delta_vx = -w_z * y_offset
            delta_vy = w_z * x_offset

            total_vx = vx_b + delta_vx
            total_vy = vy_b + delta_vy

            speed = sqrt(total_vx**2 + total_vy**2) / self.wheel_radius
            angle = atan2(total_vy, total_vx)
            angle, speed = self.normalize(angle, speed)

            wheel_speeds.append(speed)
            wheel_angles.append(angle)

        return wheel_angles, wheel_speeds
    
    def move(self):
        command, reset_flag, plot_flag, startlog_flag = self.pad.get_commands()

        wheel_ang, wheel_speed = self.compute(command * self.cmd_scale)
        pos_cmd = np.concatenate([wheel_ang, np.zeros(4)])
        vel_cmd = np.concatenate([np.zeros(4), wheel_speed])

        for i in range(NUM_MOTORS):
            self.low_cmd.motor_cmd[i].q = pos_cmd[i]
            self.low_cmd.motor_cmd[i].kp = self.kps[i]
            self.low_cmd.motor_cmd[i].dq = vel_cmd[i]
            self.low_cmd.motor_cmd[i].kd = self.kds[i]
            self.low_cmd.motor_cmd[i].tau = 0.0
    
    def move_robot(self):
        self.mode = 'move'
    
    
    def update_state(self):
        for i in range(NUM_MOTORS):
            self.qpos[i] = self.low_state.motor_state[i].q
            self.qvel[i] = self.low_state.motor_state[i].dq

        # print(f"qpos position {self.qpos}")
        # print(f"qvel velocity {self.qvel}")

        
        for i in range(3):
            self.ang_vel[i] = self.low_state.imu_state.gyroscope[i]

        for i in range(4):
            self.quat[i] = self.low_state.imu_state.quaternion[i]

        # self.update_odometry()

    def get_current_state(self):
        return self.qpos, self.qvel, self.ang_vel, self.quat
    
    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    def ekf_predict(self, dt):
        x, y, th, vx, vy, wz = self.x_ekf
        c, s = np.cos(th), np.sin(th)

        # state predict
        x_pred  = x  + (vx*c - vy*s) * dt
        y_pred  = y  + (vx*s + vy*c) * dt
        th_pred = self._wrap_angle(th + wz*dt)
        vx_pred = vx
        vy_pred = vy
        wz_pred = wz

        self.x_ekf = np.array([x_pred, y_pred, th_pred, vx_pred, vy_pred, wz_pred])

        # Jacobian F
        F = np.eye(6)
        F[0,2] = (-vx*s - vy*c) * dt
        F[0,3] =  c * dt
        F[0,4] = -s * dt

        F[1,2] = ( vx*c - vy*s) * dt
        F[1,3] =  s * dt
        F[1,4] =  c * dt

        F[2,5] = dt  

        # noice
        Q = self.Q_base * max(dt, 1e-3)

        self.P_ekf = F @ self.P_ekf @ F.T + Q
    
    def ekf_update_enc(self, vx_enc, vy_enc, wz_enc):
        # h_enc(x) = [vx, vy, wz]
        H = np.zeros((3,6))
        H[0,3] = 1.0  # vx
        H[1,4] = 1.0  # vy
        H[2,5] = 1.0  # wz

        z = np.array([vx_enc, vy_enc, wz_enc])
        z_pred = H @ self.x_ekf
        y = z - z_pred

        S = H @ self.P_ekf @ H.T + self.R_enc
        K = self.P_ekf @ H.T @ np.linalg.inv(S)

        self.x_ekf = self.x_ekf + K @ y
        self.P_ekf = (np.eye(6) - K @ H) @ self.P_ekf
        self.x_ekf[2] = self._wrap_angle(self.x_ekf[2])
    
    def update_odometry(self):
        wheel_angles = self.qpos[0:4] - self.default_angles[0:4] 
        wheel_vels = self.qvel[4:8] * np.pi / 180.0    

        vxs, vys = [], []
        for name, idx in zip(['FL', 'FR', 'RL', 'RR'], range(4)):
            angle = wheel_angles[idx]
            speed = wheel_vels[idx] * self.wheel_radius # m/s
            vxs.append(speed * np.cos(angle))
            vys.append(speed * np.sin(angle))

        A, b = [], []
        for (name, idx) in zip(['FL', 'FR', 'RL', 'RR'], range(4)):
            x_i, y_i = self.wheel_positions[name]
            A.append([1, 0, -y_i]); b.append(vxs[idx])
            A.append([0, 1,  x_i]); b.append(vys[idx])
        A = np.array(A); b = np.array(b)
        vx_enc, vy_enc, wz_enc = np.linalg.lstsq(A, b, rcond=None)[0]
        
        self.ekf_predict(self.dt)

        self.ekf_update_enc(vx_enc, vy_enc, wz_enc)

        x, y, th, vx, vy, wz = self.x_ekf
        print(f"vx: {vx} vy: {vy} vz: {wz}")

    


    def LowCmdWrite(self):
        
        while self.is_running:
            step_start = time.perf_counter()
            if self.mode == 'move':
                self.move()
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)

            self.lowcmd_publisher.Write(self.low_cmd)

            time_until_next_step = self.dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        self.ResetParam()
    
    
        
    def ResetParam(self):
        self.is_running = False


if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo") # default DDS port for amr

    controller = Controller()
    controller.Init()

    command_dict = {
        "move": controller.move_robot,
    }

    while True:        
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                controller.ShutDown()
                break

        except Exception as e:
            traceback.print_exc()
            break
    sys.exit(0)