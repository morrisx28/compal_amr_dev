import serial
import numpy as np
import time

class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 
         0x0a, 0x00, 0x00, 0x00, 0x00, 
         0, 0, 0, 0, 
         0x00, 0x08, 0x00, 0x00, 
         0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)

    def __init__(self, serial_device):
        """
        define MotorControl object 定义电机控制对象
        :param serial_device: serial object 串口对象
        """
        self.serial_ = serial_device
        if not self.serial_.is_open:
            self.serial_.open()
        print("Serial port is open")

        self.data_save = bytes()

    def _send_data(self, motor_id, data):
        self.send_data_frame[13] = motor_id & 0xff # id low 8 bits
        self.send_data_frame[14] = (motor_id >> 8)& 0xff  #id high 8 bits
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def read_motor_state(self, motor_id):
        """讀取馬達狀態，回傳 dict 結果"""
        self.serial_.reset_input_buffer()

        data = np.array([0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self._send_data(motor_id, data)

        received = self.serial_.read(64)  # 假設回覆不會超過 64 bytes
        # print("received raw:", received)

        idx = received.find(b'\x9a')
        if idx != -1 and len(received) >= idx + 8:
            frame = received[idx:idx+8]

            command = frame[0]
            temperature = int.from_bytes(frame[1:2], 'little', signed=True)
            voltage = int.from_bytes(frame[2:4], 'little') * 0.01
            current = int.from_bytes(frame[4:6], 'little') * 0.01
            motor_state = frame[6]
            error_state = frame[7]

            return {
                "command": command,
                "temperature": temperature,
                "voltage": voltage,
                "current": current,
                "motor_state": motor_state,
                "error_state": error_state
            }
        else:
            print("⚠️ No valid DATA found")
            return None
        
    def read_motor_state2(self, motor_id, timeout=0.05):
        """讀取馬達狀態，回傳 dict 結果（非阻塞，支援多馬達封包）"""

        # 發送查詢命令
        data = np.array([0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self._send_data(motor_id, data)

        deadline = time.time() + timeout

        while time.time() < deadline:
            try: 
                raw_serial_data = self.serial_.read_all()
                if not raw_serial_data and not self.data_save:
                    time.sleep(0.001)
                    continue

                data_recv = b''.join([self.data_save, raw_serial_data])
                frames = self.__extract_packets(data_recv)

                for frame in frames:
                    if len(frame) != 16:
                        continue

                    motor_id = frame[3] + 0x100   # 第 4 byte, 0x41=0x141, 0x42=0x142
                    command  = frame[7]           # 第 8 byte
                    payload  = frame[8:14]        # 6 bytes

                    if command == 0x9C:
                        # print(f"✅ Motor 0x{motor_id:X} state payload: {payload.hex()}")

                        temperature = int.from_bytes(payload[0:1], 'little', signed=True)
                        iq_or_power = int.from_bytes(payload[1:3], 'little', signed=True)
                        speed       = int.from_bytes(payload[3:5], 'little', signed=True)
                        encoder     = int.from_bytes(payload[5:7], 'little', signed=False)  # 改成 2 bytes

                        iq_ampere   = iq_or_power * (66/4096)

                        return {
                            "motor_id": motor_id,
                            "command": command,
                            "temperature": temperature,
                            "iq_raw": iq_or_power,
                            "iq": iq_ampere,
                            "speed": speed,
                            "encoder": encoder
                        }


            except Exception as e:
                print(f"Error in read_motor_state2: {e}")
                break
            
        # print("Timeout waiting for motor response")
        return None

    def __extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames

    def speed_control(self, motor_id, speedControl, iqControl):
        data = np.zeros(8, np.uint8)
        data[0] = 0xA2  
        data[1] = 0x00  

        data[2:4] = np.array([iqControl & 0xFF, (iqControl >> 8) & 0xFF], np.uint8)

        data[4] = speedControl & 0xFF
        data[5] = (speedControl >> 8) & 0xFF
        data[6] = (speedControl >> 16) & 0xFF
        data[7] = (speedControl >> 24) & 0xFF

        self._send_data(motor_id, data)
        # print(f"Sent speedControl={speedControl}, iqControl={iqControl} to motor 0x{motor_id:X}")

    def torque_control(self, motor_id, iqControl):
        data = np.zeros(8, np.uint8)
        data[0] = 0xA1  
        data[1] = 0x00
        data[2] = 0x00
        data[3] = 0x00

        # iqControl 16-bit, little endian
        data[4] = iqControl & 0xFF
        data[5] = (iqControl >> 8) & 0xFF

        data[6] = 0x00
        data[7] = 0x00

        self._send_data(motor_id, data)
        # print(f"Sent torque control iq={iqControl} to motor 0x{motor_id:X}")    

    def enable(self, motor_id):
        data = np.zeros(8, np.uint8)
        data[0] = 0x88 
        data[1:7] = np.zeros(6, np.uint8)
        self._send_data( motor_id, data)
        print(f" 0x{motor_id:X} enable")

    def disable(self, motor_id):
        data = np.zeros(8, np.uint8)
        data[0] = 0x80 
        data[1:7] = np.zeros(6, np.uint8)  
        self._send_data( motor_id, data)
        print(f" 0x{ motor_id:X} disable")
    
    def reset(self, motor_id):
        data = np.zeros(8, np.uint8)
        data[0] = 0x9B 
        data[1:7] = np.zeros(6, np.uint8)  
        self._send_data( motor_id, data)
        print(f" 0x{ motor_id:X} reset")
        
    def stop(self, motor_id):
        data = np.zeros(8, np.uint8)
        data[0] = 0x81 
        data[1:7] = np.zeros(6, np.uint8)  
        self._send_data( motor_id, data)
        print(f" 0x{ motor_id:X} stop")

#0x140+id
# id=1 motor_id=0x141

# MotorControl1.read_motor_state()
# MotorControl1.enable()
# MotorControl1.disable()
# MotorControl1.reset()
# MotorControl1.stop()
# MotorControl1.speed_control(speedControl=0, iqControl=500)
# MotorControl1.torque_control(15)

# 主机发送该命令以控制电机的速度， 同时带有力矩限制。控制值 speedControl 为 int32_t 类型，对应
# 实际转速为 0.01dps/LSB；控制值 iqControl 为 int16_t 类型，数值范围-2048~ 2048，对应 MF 电机实际转矩
# 电流范围-16.5A~16.5A，对应 MG 电机实际转矩电流范围-33A~33A，母线电流和电机的实际扭矩因不同电
# 机而异。
