import yaml
import serial
import time
import math
import traceback
import threading
import numpy as np

from DM_CAN import *

class SteeringMotorManager:
    def __init__(self, config_path="config/motor_config.yaml"):

        self.control_cmd = DualControlCmd()
        self.Is_Run = False
        self.run_thread = None
        self.read_thread = None  # 初始化
        self.joint_angles = [0, 0, 0, 0]
        self.kp_list = [3, 3, 3, 3]
        self.kd_list = [0.1, 0.1, 0.1, 0.1]
        
        self.joint_position = []
        self.joint_velocity = []

        with open(config_path, "r") as file:
            self.motor_limits = yaml.safe_load(file)["motor_limits"]

    
    def _run_motor(self):
        interval = 1.0 / 100  # 150 Hz -> 每次執行間隔 6.67 ms
        prev_time = time.time()
        count = 0

        while self.Is_Run:
            start_time = time.time()
        
            self.control_cmd.motor_position_control( self.joint_angles, self.kp_list, self.kd_list)

            count += 1
            elapsed_time = time.time() - prev_time
            if elapsed_time >= 1.0:
                # print(f"Motor control frequency: {count} Hz")
                count = 0
                prev_time = time.time()

            sleep_time = interval - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _read_motor(self):
        interval = 1.0 / 100  # 150 Hz -> 每次執行間隔 6.67 ms
        prev_time_read = time.time()
        count_read = 0

        while self.Is_Run:
            start_time = time.time()
            
            self.control_cmd.update_joint_state() 

            joint_velocity, joint_position = self.control_cmd.getAllJointState()
            # print(f"joint_position: {joint_position}")

            self.joint_position = joint_position.flatten().tolist()
            self.joint_velocity = joint_velocity.flatten().tolist()

            count_read += 1
            elapsed_time_read = time.time() - prev_time_read
            if elapsed_time_read >= 1.0:
                # print(f"Motor reading frequency: {count_read} Hz")
                count_read = 0
                prev_time_read = time.time()

            sleep_time = interval - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        if not self.Is_Run:
            self.Is_Run = True
            self.control_cmd.enable_motor()
            # self.executor.add_node(self.jointStatePub)

            self.run_thread = threading.Thread(target=self._run_motor)
            self.run_thread.start()     
            self.read_thread = threading.Thread(target=self._read_motor) 
            self.read_thread.start()   

    def stop(self):
        if self.Is_Run:
            self.Is_Run = False
            if self.run_thread and self.run_thread.is_alive():
                self.run_thread.join()
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join()
        print("Motors Set stopped running")
    
    def send_pos_cmd(self, pos_cmd, kp, kd):
        self.joint_angles[0] = -pos_cmd[0]
        self.joint_angles[1] = -pos_cmd[1]
        self.joint_angles[2] = -pos_cmd[2]
        self.joint_angles[3] = -pos_cmd[3]
        
        self.kp_list = kp[:]
        self.kd_list = kd[:]

class DualControlCmd:
    """Contro-l two sets of DM_CAN motors using different serial ports."""
    def __init__(self):
        self.setup_serials()
        self.setup_motors()
        self.joint_positions = np.zeros(4)
        self.joint_velocity = np.zeros(4)
        
    def getAllJointState(self):
        return self.joint_velocity, self.joint_positions

    # [Previous setup_serials, setup_motors, load_config methods remain the same]
    def setup_serials(self):
        self.serial_device_1 = serial.Serial('/dev/ttywheel', 921600, timeout=0.5)
        self.motor_control_1 = MotorControl(self.serial_device_1)

    def setup_motors(self):
        motor_names_1 = [ 'FL_motor', 'FR_motor', 'RL_motor', 'RR_motor']
        motor_params_1 = [(0x01, 0x11), (0x02, 0x12), (0x04, 0x14), (0x03, 0x13)]

        self.motors_1 = {
            name: Motor(DM_Motor_Type.DM4310, id_1, id_2)
            for name, (id_1, id_2) in zip(motor_names_1, motor_params_1)
        }
        for motor in self.motors_1.values():
            self.motor_control_1.addMotor(motor)
            if self.motor_control_1.switchControlMode(motor, Control_Type.MIT):
                print(f"DM_CAN Motor {motor.SlaveID} (Set 1): switched to MIT control mode")
            self.motor_control_1.save_motor_param(motor)
            self.motor_control_1.enable(motor)

        self.leg_motor_list = [ self.motors_1['FL_motor'], self.motors_1['FR_motor'], self.motors_1['RL_motor'], self.motors_1['RR_motor']]

    def reset(self):
        pos_cmd = [0, 0, 0, 0.5]
        kp = [3, 3, 3, 3]
        kd = [0.1, 0.1, 0.1, 0.1]
        self.motor_position_control(pos_cmd, kp, kd)
        print("Motors Set reset")

    def motor_position_control(self, position, kp_list, kd_list):
        """ FL FR RL RR """
        for i, motor in enumerate(self.leg_motor_list):
            self.motor_control_1.controlMIT(motor, kp_list[i], kd_list[i], position[i], 0, 0)
        

    def read(self):
        self.refresh_motor()
        self.update_joint_state()
        np.set_printoptions(suppress=True) 
        print(self.joint_positions / math.pi * 180 )
        print(self.joint_positions)

    def refresh_motor(self):
        for i, motor in enumerate(self.leg_motor_list):
            self.motor_control_1.refresh_motor_status(motor)

    def update_joint_state(self):
        for i, motor in enumerate(self.leg_motor_list):
            self.joint_positions[i] = motor.getPosition()
            self.joint_velocity[i] = motor.getVelocity()

    def enable_motor(self):
        for i, motor in enumerate(self.leg_motor_list):
            self.motor_control_1.enable(motor)
        print("enable the motor")

    def disable_motor(self):
        for i, motor in enumerate(self.leg_motor_list):
            self.motor_control_1.disable(motor)
        print("Disable the motor")

    def set_zero(self):
        for i, motor in enumerate(self.leg_motor_list):
            self.motor_control_1.set_zero_position(motor)
        print("Motor zero position set")

    def closeSystem(self):
        """Shut down the system."""
        self.serial_device_1.close()
        print("System closed")


def main():
    motor_manager = SteeringMotorManager()
    
    command_dict = {
        "r": motor_manager.run,
        "stop": motor_manager.stop,
        "reset": motor_manager.control_cmd.reset,
        "read": motor_manager.control_cmd.read,
        "enable": motor_manager.control_cmd.enable_motor,
        "disable": motor_manager.control_cmd.disable_motor,
        "set" : motor_manager.control_cmd.set_zero,
    }
    
    print("Available commands:")
    print("r - Run all motors")
    print("stop - Stop all motors")
    print("reset - Reset all motors")
    print("read - Read status of all motors")
    print("enable - Enable all motors")
    print("disable - Disable all motors")
    print("exit - Close the system")
    
    while True:
        try:
            cmd = input("CMD: ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                motor_manager.stop()
                motor_manager.control_cmd.closeSystem()
                break
            else:
                print("Invalid command")
        except Exception as e:
            traceback.print_exc()
            break

if __name__ == '__main__':
    main()
