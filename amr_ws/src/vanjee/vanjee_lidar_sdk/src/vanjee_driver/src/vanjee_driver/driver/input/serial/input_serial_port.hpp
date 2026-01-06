/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#if false
#include <iostream>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <vanjee_driver/driver/input/input.hpp>

using namespace boost::asio;

namespace vanjee {
namespace lidar {

class InputSerialPort : public Input {
 public:
  InputSerialPort(const WJInputParam &input_param, double sec_to_delay);
  virtual bool init();
  virtual bool start();
  virtual ~InputSerialPort();

 private:
  void recvPacket();

  bool openPort(std::string port_name, uint32_t baud_rate);
  void closePort();
  ssize_t sendData(const uint8* data, uint32 size);

  virtual int32 send_(uint8 *buf, uint32 size);

 private:
  io_service io_;
  serial_port serial_;
};

InputSerialPort::InputSerialPort(const WJInputParam &input_param, double sec_to_delay)
    : Input(input_param), io_(), serial_(io_, input_param.port_name) {
  
}

inline bool InputSerialPort::init() {
  if (init_flag_) {
    return true;
  }
  int msop_fd = -1;
  msop_fd = openPort(input_param_.port_name, input_param_.baud_rate);
  if (msop_fd < 0) {
    WJ_ERROR << "failed to create serial port!" << WJ_REND;
    goto failMsop;
  }

  init_flag_ = true;
  return true;

  failMsop:
  return false;
}

inline bool InputSerialPort::start() {
  if (start_flag_) {
    return true;
  }
  if (!init_flag_) {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacket, this));
  start_flag_ = true;
  return true;
}

inline InputSerialPort::~InputSerialPort() {
  stop();
  if (serial_.is_open()) {
    closePort();
  }
}

bool InputSerialPort::openPort(std::string port_name, uint32_t baud_rate) {
  // 设置串口参数
  serial_.set_option(serial_port_base::baud_rate(baud_rate));
  serial_.set_option(serial_port_base::character_size(8));
  serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
  serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  return true;
}


void InputSerialPort::closePort() {
  if (serial_.is_open()) {
    serial_.close();
  }
}

ssize_t InputSerialPort::sendData(const uint8* data, uint32 size) {
  if (!serial_.is_open()) {
    std::cerr << "Port not open" << std::endl;
    return -1;
  }
  std::vector<uint8_t> send_data;
  send_data.assign(data, data + size);
  return write(serial_, buffer(send_data));
}

int32 InputSerialPort::send_(uint8 *buf, uint32 size) {
  return sendData(buf, size);
}

inline void InputSerialPort::recvPacket() {
  while (!to_exit_recv_) {
    if (serial_.is_open()) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret = read(serial_, buffer(pkt->buf(), pkt->bufSize()));
      
      if (ret < 0) {
        std::cerr << "Port not open" << std::endl;
      } else if (ret > 0) {
        pkt->setData(0, ret, "0.0.0.0");
        pushPacket(pkt);
      }
    }
  }
}

}  // namespace lidar
}  // namespace vanjee
#endif

#if true
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

// #include <asm-generic/termbits.h>
#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/driver/input/serial/my_termbits.hpp>

namespace vanjee {
namespace lidar {

class InputSerialPort : public Input {
 public:
  InputSerialPort(const WJInputParam &input_param, double sec_to_delay);
  virtual bool init();
  virtual bool start();
  virtual ~InputSerialPort();

 private:
  void recvPacket();

  bool openPort(std::string port_name, uint32_t baud_rate);
  void closePort();
  ssize_t sendData(const uint8 *data, uint32 size);

  virtual int32 send_(uint8 *buf, uint32 size);

 private:
  std::string portName;
  int baudRate;
  int fd_;
};

InputSerialPort::InputSerialPort(const WJInputParam &input_param, double sec_to_delay) : Input(input_param) {
}

inline bool InputSerialPort::init() {
  if (init_flag_) {
    return true;
  }
  int msop_fd = -1;
  msop_fd = openPort(input_param_.port_name, input_param_.baud_rate);
  if (msop_fd < 0) {
    WJ_ERROR << "failed to create serial port!" << WJ_REND;
    goto failMsop;
  }

  init_flag_ = true;
  return true;

failMsop:
  return false;
}

inline bool InputSerialPort::start() {
  if (start_flag_) {
    return true;
  }
  if (!init_flag_) {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacket, this));
  start_flag_ = true;
  return true;
}

inline InputSerialPort::~InputSerialPort() {
  stop();
  if (fd_ != -1) {
    closePort();
  }
}

bool InputSerialPort::openPort(std::string port_name, uint32_t baud_rate) {
#if true
  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if (fd_ == -1) {
    std::cerr << "Failed to open port: " << port_name << std::endl;
    return false;
  }

  struct termios2 newtio, oldtio;
  if (ioctl(fd_, TCGETS2, &oldtio) != 0) {
    return false;
  }
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  newtio.c_cflag |= CS8;

  newtio.c_cflag &= ~PARENB;

  newtio.c_cflag |= BOTHER;
  newtio.c_ispeed = baud_rate;
  newtio.c_ospeed = baud_rate;

  newtio.c_cflag &= ~CSTOPB;

  tcflush(fd_, TCIFLUSH);
  if (ioctl(fd_, TCSETS2, &newtio) != 0) {
    return false;
  }
  return true;

#endif

#if false
  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    std::cerr << "Failed to open port: " << port_name << std::endl;
    return false;
  }

  struct termios options;
  tcgetattr(fd_, &options);

  // Set baud rate
  speed_t baud_rate1 = B115200;
  int result1 = cfsetispeed(&options, baud_rate1);
  int result2 = cfsetospeed(&options, baud_rate1);

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    return false;
  }

  // Set 8N1 (8 data bits, no parity, 1 stop bit)
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Enable the receiver and set local mode
  options.c_cflag |= (CLOCAL | CREAD);

  // Set the new options for the port
  int result = tcsetattr(fd_, TCSANOW, &options);

  return true;
#endif
}

void InputSerialPort::closePort() {
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
}

ssize_t InputSerialPort::sendData(const uint8 *data, uint32 size) {
  if (fd_ == -1) {
    std::cerr << "Port not open" << std::endl;
    return -1;
  }
  return write(fd_, data, size);
}

int32 InputSerialPort::send_(uint8 *buf, uint32 size) {
  int32 ret = -1;
  if (fd_ != -1) {
    ret = sendData(buf, size);
  }
  return ret;
}

inline void InputSerialPort::recvPacket() {
  uint32_t size = 0;
  double start_ts = getTimeHost() * 1e-6;
  double cur_ts = 0;
  uint32_t index = 0;
  while (!to_exit_recv_) {
    if (fd_ != -1) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(20000);
      ssize_t ret = read(fd_, pkt->buf(), pkt->bufSize());
      cur_ts = getTimeHost() * 1e-6;
      if (ret < 0) {
        // std::cerr << "Port not open" << std::endl;
      } else if (ret > 0) {
        pkt->setData(0, ret, "0.0.0.0");
        pushPacket(pkt);
        size += ret;
      }
    }
    double ts_gap = cur_ts - start_ts;
    if (ts_gap >= 1.0) {
      if (size == 0) {
        cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      } else {
        size = 0;
      }
      start_ts = cur_ts;
    }
  }
}

}  // namespace lidar
}  // namespace vanjee
#endif
