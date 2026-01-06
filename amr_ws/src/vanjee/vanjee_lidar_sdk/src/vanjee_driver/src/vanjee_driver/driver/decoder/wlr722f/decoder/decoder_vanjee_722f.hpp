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

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_base.hpp>
#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/imuParamGet.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/cmd_repository_722f.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_error_code_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_imu_packet_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_imu_temp_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_ldvalue_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_work_mode_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_work_mode_set.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee722fChannel {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t tag;
} Vanjee722fChannel;

typedef struct _Vanjee722fDifop {
  uint8_t mac_id[2];
  uint16_t circle_id;
  uint32_t sec;
  uint32_t nsec;
  uint8_t remain[2];
  int16_t imu_linear_acce_x;
  int16_t imu_linear_acce_y;
  int16_t imu_linear_acce_z;
  int16_t imu_angle_voc_x;
  int16_t imu_angle_voc_y;
  int16_t imu_angle_voc_z;
  uint8_t remain1[11];
  uint8_t return_wave_mode;
  uint16_t rpm;
  uint8_t device_id[3];
  uint8_t versoin;
  uint8_t remain2[12];
  uint16_t frame_id;
  uint8_t tail[2];
} Vanjee722fDifop;

typedef struct _Vanjee722fBlockChannel16 {
  uint16_t azimuth;
  Vanjee722fChannel channel[16];
} Vanjee722fBlockChannel16;

typedef struct _Vanjee722fMsopPktChannel16 {
  uint8_t head[2];
  uint8_t channel_num;
  uint8_t return_wave_num;
  uint8_t block_num;
  Vanjee722fBlockChannel16 blocks[10];
  Vanjee722fDifop difop;
} Vanjee722fMsopPktChannel16;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee722F : public DecoderMech<T_PointCloud> {
 private:
  int32_t optcent_2_lidar_arg_ = 21870;
  float optcent_2_lidar_l_ = 2.112 * 1e-2;
  float optcent_2_lidar_z_ = 6.39 * 1e-3;

  uint32_t frame_num_per_circle_ = 120;
  std::vector<std::vector<double>> all_points_luminous_moment_722_16_;
  const double luminous_period_of_ld_16_ = 8.33e-5;
  const double luminous_period_of_adjacent_ld_ = 5e-6;
  std::vector<uint32_t> all_points_luminous_moment_size_;
  int32_t pkt_id_in_circle_trans_pre_ = -1;

  int32_t pre_frame_id_ = -1;
  int32_t pre_circle_id_ = -1;
  uint8_t publish_mode_ = 0;
  double pre_pkt_time_ = -1;

  bool angle_param_get_flag_ = false;

  uint8_t protocol_versoin_ = 0;
  int32_t pre_imu_frame_id_ = -1;
  double pre_imu_timestamp_ = 0.0;
  uint32_t start_angle_ = 0;
  uint32_t end_angle_ = 360000;

  std::vector<std::string> error_code_str_ =
      std::vector<std::string>{"输入电压异常", "温度异常", "电机芯片转速异常", "码盘转速异常", "陀螺仪异常", "FPGA程序异常", "接收模块温度异常"};

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 100;
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual bool decodeMsopPktChannel16(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee722F() = default;
  explicit DecoderVanjee722F(const WJDecoderParam &param);

  void SendImuData(Vanjee722fDifop difop, double temperature, double timestamp, double lidar_timestamp);

 public:
  std::shared_ptr<ImuParamGet> m_imu_params_get_;
  double imu_temperature_;
  bool imu_ready_;
  bool point_cloud_ready_;
};

template <typename T_PointCloud>
void DecoderVanjee722F<T_PointCloud>::initLdLuminousMoment() {
  all_points_luminous_moment_722_16_.resize(1);
  all_points_luminous_moment_722_16_[0].resize(19200);
  all_points_luminous_moment_size_.resize(1);
  all_points_luminous_moment_size_[0] = 19200;
  for (uint16_t col = 0; col < 1200; col++) {
    for (uint8_t row = 0; row < 16; row++) {
      all_points_luminous_moment_722_16_[0][col * 16 + row] = col * luminous_period_of_ld_16_ + row * luminous_period_of_adjacent_ld_;
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee722F<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJ_INFOL << "publish_mode ============mode=================" << mode <<
  // WJ_REND;
  uint16_t msop_len = 725;
  uint16_t laser_num = 16;
  uint16_t block_num = 10;
  uint16_t chan_num = 16;
  float distance_min = 0.2f;
  float distance_max = 70.0f;
  float distance_resolution = 0.004f;
  float init_temperature = 80.0f;

  static WJDecoderMechConstParam param = {
      msop_len  /// msop len
      ,
      laser_num  /// laser number
      ,
      block_num  /// blocks per packet
      ,
      chan_num  /// channels per block
      ,
      distance_min  /// distance min
      ,
      distance_max  /// distance max
      ,
      distance_resolution  /// distance resolution
      ,
      init_temperature  /// initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 360;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee722F<T_PointCloud>::DecoderVanjee722F(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
#if defined(ENABLE_TRANSFORM)
  m_imu_params_get_ = std::make_shared<ImuParamGet>(-param.transform_param.yaw);
#else
  m_imu_params_get_ = std::make_shared<ImuParamGet>(0);
#endif
  imu_temperature_ = -100.0;
  imu_ready_ = false;
  point_cloud_ready_ = false;
  if (param.imu_enable == -1) {
    point_cloud_ready_ = true;
  } else if (param.imu_enable == 0) {
    imu_temperature_ = 40.0;
    point_cloud_ready_ = true;
  } else {
    WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
  }
  start_angle_ = this->param_.start_angle * 1000;
  end_angle_ = this->param_.end_angle * 1000;

  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);

    this->chan_angles_.loadFromFile(this->param_.angle_path_hor, 1200);

    this->imu_calibration_param_.loadFromFile(param.imu_param_path);

    m_imu_params_get_->setImuTempCalibrationParams(this->imu_calibration_param_.x_axis_temp_k, this->imu_calibration_param_.x_axis_temp_b,
                                                   this->imu_calibration_param_.y_axis_temp_k, this->imu_calibration_param_.y_axis_temp_b,
                                                   this->imu_calibration_param_.z_axis_temp_k, this->imu_calibration_param_.z_axis_temp_b);

    m_imu_params_get_->setImuAcceCalibrationParams(this->imu_calibration_param_.x_axis_acc_k, this->imu_calibration_param_.x_axis_acc_b,
                                                   this->imu_calibration_param_.y_axis_acc_k, this->imu_calibration_param_.y_axis_acc_b,
                                                   this->imu_calibration_param_.z_axis_acc_k, this->imu_calibration_param_.z_axis_acc_b);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee722F<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  uint16_t l_pktheader = pkt[0] << 8 | pkt[1];
  switch (l_pktheader) {
    case 0xFFDD: {
      if (size == sizeof(Vanjee722fMsopPktChannel16))
        ret = decodeMsopPktChannel16(pkt, size);
    } break;
    default:
      break;
  }

  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee722F<T_PointCloud>::decodeMsopPktChannel16(const uint8_t *pkt, size_t size) {
  auto &packet = *(Vanjee722fMsopPktChannel16 *)pkt;
  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  uint16_t frame_id = ntohs(packet.difop.frame_id);
  uint32_t loss_packets_num = (frame_id + frame_num_per_circle_ - pre_frame_id_) % frame_num_per_circle_;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  pkt_host_ts = getTimeHost() * 1e-6;
  pkt_lidar_ts = packet.difop.sec + packet.difop.nsec * 1e-9;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts;
    if (pkt_ts < 3600) {
      pkt_ts = 3600;
    }
  }

  protocol_versoin_ = packet.difop.versoin;

  int32_t resolution = 30;
  uint8_t resolurion_index = 0;  // resolution / 30 - 1;

  if (packet.difop.versoin < 2 && pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!imu_ready_ && !point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!point_cloud_ready_) {
    point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }
  uint16_t block_num = packet.block_num * packet.return_wave_num;

  uint32_t pkt_id_in_circle = frame_id;
  uint32_t pkt_id_in_circle_trans = (frame_id + 1) % frame_num_per_circle_;
  if ((this->split_strategy_->newBlock(pkt_id_in_circle_trans) && pkt_id_in_circle_trans != 0) ||
      (loss_circles_num == 1 && pkt_id_in_circle != 0 && this->point_cloud_->points.size() != 0)) {
    uint32_t point_gap_num = (pkt_id_in_circle * 10) * 16 + 1;
    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_722_16_[resolurion_index][point_gap_num];
    this->first_point_ts_ =
        this->last_point_ts_ - all_points_luminous_moment_722_16_[resolurion_index][all_points_luminous_moment_size_[resolurion_index] - 1];

    this->cb_split_frame_(packet.channel_num, this->cloudTs());
    ret = true;
  }

  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }

    const Vanjee722fBlockChannel16 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;

    {
      double timestamp_point;
      uint32_t point_id_first = (pkt_id_in_circle * 10 + blk) * packet.channel_num;
      int32_t azimuth_10 = azimuth * 10;
      for (uint16_t chan = 0; chan < packet.channel_num; chan++) {
        float x, y, z, xy;

        uint32_t point_id = point_id_first + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_722_16_[resolurion_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_722_16_[resolurion_index][point_id] -
                            all_points_luminous_moment_722_16_[resolurion_index][all_points_luminous_moment_size_[resolurion_index] - 1];
        }

        const Vanjee722fChannel &channel = block.channel[chan];

        float distance = channel.distance * this->const_param_.DISTANCE_RES;
        int32_t verticalVal_722 = (this->chan_angles_.vertAdjust(chan) + 360000) % 360000;
        int32_t azimuth_index = (this->chan_angles_.horizAdjust(chan, azimuth_10) + 360000) % 360000;

        // int32_t angle_horiz_mask = 360000 - azimuth_index;
        int32_t angle_horiz_mask = azimuth_index;
        if (start_angle_ < end_angle_) {
          if (angle_horiz_mask < start_angle_ || angle_horiz_mask > end_angle_) {
            distance = 0;
          }
        } else {
          if (angle_horiz_mask > end_angle_ && angle_horiz_mask < start_angle_) {
            distance = 0;
          }
        }

        if (this->hide_range_flag_ && distance != 0 &&
            this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
          distance = 0;
        }
        int32_t optcent_2_lidar_angle_hor = (azimuth * 10 + optcent_2_lidar_arg_ + 360000) % 360000;

        if (this->distance_section_.in(distance)) {
          xy = distance * COS(verticalVal_722);
          // x = xy * COS(azimuth_index);
          // y = -xy * SIN(azimuth_index);
          x = xy * SIN(azimuth_index) + optcent_2_lidar_l_ * SIN(optcent_2_lidar_angle_hor);
          y = xy * COS(azimuth_index) + optcent_2_lidar_l_ * COS(optcent_2_lidar_angle_hor);
          z = distance * SIN(verticalVal_722) + optcent_2_lidar_z_;
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
          setTag(point, channel.tag);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_722 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            float x_zero = 0;
            float y_zero = 0;
            float z_zero = 0;
            this->transformPoint(x_zero, y_zero, z_zero);

            setX(point, x_zero);
            setY(point, y_zero);
            setZ(point, z_zero);
          }
          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
          setTag(point, channel.tag);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_722 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
  }
  if (pkt_id_in_circle_trans == 0) {
    uint32_t point_gap_num = 10 * 16 - 1;
    this->last_point_ts_ = pkt_ts + all_points_luminous_moment_722_16_[resolurion_index][point_gap_num];
    this->first_point_ts_ =
        this->last_point_ts_ - all_points_luminous_moment_722_16_[resolurion_index][all_points_luminous_moment_size_[resolurion_index] - 1];

    this->cb_split_frame_(packet.channel_num, this->cloudTs());
    ret = true;
  }
  pkt_id_in_circle_trans_pre_ = pkt_id_in_circle_trans;

  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee722F<T_PointCloud>::SendImuData(Vanjee722fDifop difop, double temperature, double timestamp, double lidar_timestamp) {
  if (this->param_.imu_enable == -1)
    return;
  imu_ready_ = m_imu_params_get_->imuGet(difop.imu_angle_voc_x, difop.imu_angle_voc_y, difop.imu_angle_voc_z, difop.imu_linear_acce_x,
                                         difop.imu_linear_acce_y, difop.imu_linear_acce_z, lidar_timestamp, temperature);
  if (imu_ready_) {
    this->imu_packet_->timestamp = timestamp;
    this->imu_packet_->angular_voc[0] = m_imu_params_get_->imu_result_stu_.x_angle;
    this->imu_packet_->angular_voc[1] = m_imu_params_get_->imu_result_stu_.y_angle;
    this->imu_packet_->angular_voc[2] = m_imu_params_get_->imu_result_stu_.z_angle;

    this->imu_packet_->linear_acce[0] = m_imu_params_get_->imu_result_stu_.x_acc;
    this->imu_packet_->linear_acce[1] = m_imu_params_get_->imu_result_stu_.y_acc;
    this->imu_packet_->linear_acce[2] = m_imu_params_get_->imu_result_stu_.z_acc;

    this->imu_packet_->orientation[0] = m_imu_params_get_->imu_result_stu_.q0;
    this->imu_packet_->orientation[1] = m_imu_params_get_->imu_result_stu_.q1;
    this->imu_packet_->orientation[2] = m_imu_params_get_->imu_result_stu_.q2;
    this->imu_packet_->orientation[3] = m_imu_params_get_->imu_result_stu_.q3;

    this->cb_imu_pkt_();
  }
}

template <typename T_PointCloud>
void DecoderVanjee722F<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract722F> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_ld_value_get_)) {
    p = std::make_shared<Protocol_LDValueGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_get_work_mode_)) {
    p = std::make_shared<Protocol_WorkModeGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_set_work_mode_)) {
    p = std::make_shared<Protocol_WorkModeSet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_get_imu_packet_)) {
    p = std::make_shared<Protocol_ImuPacketGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_get_error_code_)) {
    p = std::make_shared<Protocol_ErrorCodeGet722F>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDValue722F)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    if (!angle_param_get_flag_) {
      std::shared_ptr<Params_LDValue722F> param = std::dynamic_pointer_cast<Params_LDValue722F>(params);
      std::vector<double> vert_angles;
      std::vector<double> offset_angles;
      for (int num_of_lines = 0; num_of_lines < param->num_of_lines_; num_of_lines++) {
        vert_angles.push_back((double)(param->ver_angle_[num_of_lines] / 1000.0));
        offset_angles.push_back((double)(param->offset_angle_[num_of_lines] / 1000.0));
      }

      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->num_of_lines_, vert_angles, offset_angles);
        WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }

      angle_param_get_flag_ = true;
    }

    if (angle_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else if (typeid(*params) == typeid(Params_WorkModeGet722F)) {
    std::shared_ptr<Params_WorkModeGet722F> param = std::dynamic_pointer_cast<Params_WorkModeGet722F>(params);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);

  } else if (typeid(*params) == typeid(Params_WorkModeSet722F)) {
    std::shared_ptr<Params_WorkModeSet722F> param = std::dynamic_pointer_cast<Params_WorkModeSet722F>(params);
    if (this->param_.device_ctrl_state_enable && param->flag_ == 1) {
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      this->device_ctrl_->cmd_id = 1;
      this->device_ctrl_->cmd_state = 1;
      this->cb_device_ctrl_state_(pre_pkt_time_ < 0 ? 0 : pre_pkt_time_);
    }

  } else if (typeid(*params) == typeid(Params_ImuPacketGet722F)) {
    std::shared_ptr<Params_ImuPacketGet722F> param = std::dynamic_pointer_cast<Params_ImuPacketGet722F>(params);
    if (protocol_versoin_ < 2 || this->param_.imu_enable == -1)
      return;
    double pkt_ts = 0.0;
    double imu_timestamp = param->imu_sec_ + (param->imu_nsec_ * 1e-9);
    if (this->param_.use_lidar_clock)
      pkt_ts = imu_timestamp;
    else
      pkt_ts = getTimeHost() * 1e-6;

    uint32_t loss_packets_num = (param->frame_id_ + 65536 - pre_imu_frame_id_) % 65536;
    if (loss_packets_num > 1 && pre_imu_frame_id_ >= 0)
      WJ_WARNING << "loss " << (loss_packets_num - 1) << " imu packets" << WJ_REND;
    pre_imu_frame_id_ = param->frame_id_;

    if (pre_imu_timestamp_ > 0 && imu_timestamp > pre_imu_timestamp_) {
      imu_ready_ =
          m_imu_params_get_->imuGet(param->imu_angle_voc_x_, param->imu_angle_voc_y_, param->imu_angle_voc_z_, param->imu_linear_acce_x_,
                                    param->imu_linear_acce_y_, param->imu_linear_acce_z_, imu_timestamp, -100.0, false, false, false, true, false);
      if (imu_ready_) {
        this->imu_packet_->timestamp = pkt_ts;
        this->imu_packet_->angular_voc[0] = m_imu_params_get_->imu_result_stu_.x_angle;
        this->imu_packet_->angular_voc[1] = m_imu_params_get_->imu_result_stu_.y_angle;
        this->imu_packet_->angular_voc[2] = m_imu_params_get_->imu_result_stu_.z_angle;

        this->imu_packet_->linear_acce[0] = m_imu_params_get_->imu_result_stu_.x_acc;
        this->imu_packet_->linear_acce[1] = m_imu_params_get_->imu_result_stu_.y_acc;
        this->imu_packet_->linear_acce[2] = m_imu_params_get_->imu_result_stu_.z_acc;

        this->imu_packet_->orientation[0] = m_imu_params_get_->imu_result_stu_.q0;
        this->imu_packet_->orientation[1] = m_imu_params_get_->imu_result_stu_.q1;
        this->imu_packet_->orientation[2] = m_imu_params_get_->imu_result_stu_.q2;
        this->imu_packet_->orientation[3] = m_imu_params_get_->imu_result_stu_.q3;

        this->cb_imu_pkt_();
      }
    }
    pre_imu_timestamp_ = imu_timestamp;

  } else if (typeid(*params) == typeid(Params_ErrorCodeGet722F)) {
    std::shared_ptr<Params_ErrorCodeGet722F> param = std::dynamic_pointer_cast<Params_ErrorCodeGet722F>(params);
    if (param->cmd_id_ == 0) {
      std::string err_str = "";
      for (int i = 0; i < error_code_str_.size(); i++) {
        if ((param->cmd_param_ & (0x01 << i)) != 0)
          err_str += " " + error_code_str_[i];
      }
      if (err_str != "") {
        WJ_WARNING << "error log:" << err_str << WJ_REND;
        this->device_ctrl_->cmd_id = 0x0100;
        this->device_ctrl_->cmd_param = param->cmd_param_;
        this->device_ctrl_->cmd_state = 1;
        this->cb_device_ctrl_state_(pre_pkt_time_ < 0 ? 0 : pre_pkt_time_);
      }
    }
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar

}  // namespace vanjee
