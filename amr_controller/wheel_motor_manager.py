import serial
import threading
import time
import traceback
import numpy as np
from wheel_motor import MotorControl 
import matplotlib.pyplot as plt



class WheelMotorManager:
    def __init__(self):
        self.thread_running = False
        self.thread_running = False
        self.control_cmd = DualControlCmd() 

        self.vel_cmd = np.array([0, 0, 0, 0])
        self.vel_scale = 100 * 6
        self.iq = 2000
        self.dt = 0.001

        self.motor_velocity = []

    def loop_speed_and_read(self):
        
        prev_time = time.time()
        while self.thread_running:
            # print("self.vel_cmd",self.vel_cmd)
            self.control_cmd.motor_velocity_control(self.vel_cmd * self.vel_scale, self.iq)
            
            self.control_cmd.read_motors()

            motor_velocity = self.control_cmd.getAllMotorState()

            self.motor_velocity = motor_velocity.flatten().tolist()

            now = time.time()
            dt = now - prev_time
            prev_time = now
            freq = 1.0 / dt if dt > 0 else 0

            # print(f"freq={freq:.2f} Hz | ")

            time.sleep(self.dt)

    def start_thread(self):
        if self.thread_running:
            print("Thread already running")
            return

        self.thread_running = True
        self.MG_run_thread = threading.Thread(target=self.loop_speed_and_read)
        self.MG_run_thread.start()

        print("thread started")

    def stop_thread(self):
        self.thread_running = False
        if self.MG_run_thread and self.MG_run_thread.is_alive():
            self.MG_run_thread.join()
        self.control_cmd.stop()
        print("thread stopped")
        # 確保串口釋放
        self.control_cmd.serial_device.close()
    
    def send_vel_cmd(self, FL_vel_cmd, FR_vel_cmd, RL_vel_cmd, RR_vel_cmd):
        """ vel_cmd [FL, FR, RL, RR] """
        self.vel_cmd[0] = -FL_vel_cmd
        self.vel_cmd[1] = FR_vel_cmd
        self.vel_cmd[2] = -RL_vel_cmd
        self.vel_cmd[3] = RR_vel_cmd



class DualControlCmd:
    def __init__(self):
        self.setup_serial()
        self.setup_motors()
        self.motorMG_positions = np.zeros(4)
        self.motorMG_velocity = np.zeros(4)
        motorMG_velocity = np.zeros(4)

        self.history = {name: [] for name in self.motor_MG.keys()}
        self.time_stamps = []  # 記錄時間軸

    def getAllMotorState(self):
        motorMG_velocity = self.motorMG_velocity
        motorMG_velocity[0] = -np.array(motorMG_velocity[0])
        motorMG_velocity[2] = -np.array(motorMG_velocity[2])
        return self.motorMG_velocity

    def setup_serial(self):
        self.serial_device = serial.Serial("/dev/ttysteering", 115200, timeout=0.5)
        self.motor_contorl_MG = MotorControl(self.serial_device)
    
    def setup_motors(self):
        motor_name_MG = [ 'FL_motor', 'FR_motor', 'RL_motor', 'RR_motor']  # motor name
        motor_params_MG = [ 0x141, 0x142, 0x143, 0x144]  # motor id

        self.motor_MG = dict(zip(motor_name_MG, motor_params_MG))
        self.motor_MG_list = [
             self.motor_MG['FL_motor'], self.motor_MG['FR_motor'],
             self.motor_MG['RL_motor'], self.motor_MG['RR_motor']]
    
    def motor_velocity_control(self, vel_cmd: np.array, iq=2000):            
        for i, motor_id in enumerate(self.motor_MG_list):
            self.motor_contorl_MG.speed_control(motor_id, vel_cmd[i], iqControl=iq)

    def read_motors(self):
        now = time.time()
        self.time_stamps.append(now)

        for i,  motor_id in enumerate(self.motor_MG_list):
            state = self.motor_contorl_MG.read_motor_state2(motor_id)
            # print(f"{i} {motor_id} {state}")
            if state is not None:
                self.motorMG_velocity[i] = state["speed"] /6
        
        # print(f"Motor Velocities: {self.motorMG_velocity}")
    
    def plot_history(self):
        """每個馬達畫在自己的 subplot"""
        num_motors = len(self.history)
        fig, axes = plt.subplots(num_motors, 1, figsize=(8, 2.5 * num_motors), sharex=True)

        if num_motors == 1:
            axes = [axes]  # 確保即使只有一個 motor 也能迭代

        for ax, (name, speeds) in zip(axes, self.history.items()):
            ax.plot(self.time_stamps, speeds, label=name)
            ax.set_ylabel("Speed (dps)")
            ax.set_title(name)
            ax.grid(True)
            ax.legend()

        axes[-1].set_xlabel("Time (s)")  # 只有最後一個 subplot 設定 X label
        plt.tight_layout()
        plt.show()

    def save_history_to_csv(self, filename="motor_history.csv"):
        """把歷史資料存成 CSV"""
        import pandas as pd
        df = pd.DataFrame(self.history, index=self.time_stamps)
        df.index.name = "timestamp"
        df.to_csv(filename)
        print(f"History saved to {filename}")

    def enable(self):
        for i, motor in enumerate(self.motor_MG_list):
            self.motor_contorl_MG.enable(motor)

    def disable(self):
        for i, motor in enumerate(self.motor_MG_list):
            self.motor_contorl_MG.disable(motor)
    
    def reset(self):
        for i, motor in enumerate(self.motor_MG_list):
            self.motor_contorl_MG.reset(motor)
            time.sleep(0.1)  # 每個馬達 reset 間隔 100ms
    
    def stop(self):
        for i, motor in enumerate(self.motor_MG_list):
            self.motor_contorl_MG.stop(motor)


def main():
    
    motor_manager = WheelMotorManager()

    command_dict = {
        "enable": motor_manager.control_cmd.enable,
        "disable": motor_manager.control_cmd.disable,
        "reset": motor_manager.control_cmd.reset,
        "stop": motor_manager.control_cmd.stop,
        "start": motor_manager.start_thread,
        "st": motor_manager.stop_thread,
        "plot": motor_manager.control_cmd.plot_history,
    }

    print("Available commands: enable, disable, reset, stop, start, st, exit")

    while True:
        try:
            cmd = input("cmd: ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                motor_manager.stop_thread()
                break
            else:
                print("Unknown command. Please try again.")
        except KeyboardInterrupt:
            traceback.print_exc()
            motor_manager.stop_thread()
            break

if __name__ == '__main__':
    main()
