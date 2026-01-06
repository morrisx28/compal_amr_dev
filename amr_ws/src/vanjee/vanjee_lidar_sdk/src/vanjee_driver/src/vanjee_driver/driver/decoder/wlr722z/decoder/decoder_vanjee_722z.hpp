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
#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/imuParamGet.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/cmd_repository_722z.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_ldvalue_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee722zSerialBlockChannel {
  uint16_t distance;
  uint8_t reflectivity;
} Vanjee722zSerialBlockChannel;

typedef struct _Vanjee722zSerialPointCloud {
  uint16_t azimuth;
  Vanjee722zSerialBlockChannel channel[16];
  uint32_t dirty_degree;
  uint8_t lidar_state;
  uint8_t reserved_id;
  uint16_t reserved_info;
  uint16_t sequence_num;
} Vanjee722zSerialPointCloud;

typedef struct _Vanjee722zSerialPointCloudMsopPkt {
  uint8_t head[2];
  uint8_t protocol_major_version;
  uint8_t protocol_minor_version;
  uint8_t diagnostic_information_version;
  uint8_t data_type;
  uint8_t data_time[6];
  uint8_t timestamp[4];
  Vanjee722zSerialPointCloud blocks;
  uint32_t crc;
} Vanjee722zSerialPointCloudMsopPkt;

typedef struct _Vanjee722zSerialImu {
  int16_t imu_linear_acce_x;
  int16_t imu_linear_acce_y;
  int16_t imu_linear_acce_z;
  int16_t imu_angle_voc_x;
  int16_t imu_angle_voc_y;
  int16_t imu_angle_voc_z;
  uint16_t sequence_num;
} Vanjee722zSerialImu;

typedef struct _Vanjee722zSerialImuMsopPkt {
  uint8_t head[2];
  uint8_t protocol_major_version;
  uint8_t protocol_minor_version;
  uint8_t diagnostic_information_version;
  uint8_t data_type;
  uint8_t data_time[6];
  uint8_t timestamp[4];
  Vanjee722zSerialImu blocks;
  uint32_t crc;
} Vanjee722zSerialImuMsopPkt;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee722Z : public DecoderMech<T_PointCloud> {
 private:
  int32_t optcent_2_lidar_arg_ = 21570;
  float optcent_2_lidar_l_ = 2.067 * 1e-2;
  float optcent_2_lidar_z_ = 7.95 * 1e-3;

  std::vector<double> all_points_luminous_moment_serial_;  // Cache 16 channels for one circle of point cloud time difference

  const double luminous_period_of_ld_ = 3.33333 * 1e-4;           // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 2.08333 * 1e-5;  // Time interval between adjacent vertical angles within the group

  int32_t pre_imu_frame_id_ = -1;
  int32_t pre_point_cloud_frame_id_ = -1;
  uint8_t publish_mode_ = 0;

  bool angle_param_get_flag_ = false;
  std::vector<int32_t> eccentricity_angles_;
  std::vector<int32_t> eccentricity_angles_real_;

  std::vector<uint8_t> buf_cache_;
  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 100;

  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  bool decodeMsopPktImu(const uint8_t *pkt, size_t size);
  bool decodeMsopPktSerialPointCloud(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee722Z() = default;
  explicit DecoderVanjee722Z(const WJDecoderParam &param);

  void SendSerialImuData(Vanjee722zSerialImu difop, double timestamp, double lidar_timestamp);

 public:
  std::shared_ptr<ImuParamGet> m_imu_params_get_;
};

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::initLdLuminousMoment() {
  double offset = 0;
  all_points_luminous_moment_serial_.resize(9616);
  for (uint16_t col = 0; col < 601; col++) {
    for (uint8_t row = 0; row < 16; row++) {
      all_points_luminous_moment_serial_[col * 16 + row] = col * luminous_period_of_ld_ + row * luminous_period_of_adjacent_ld_;
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee722Z<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJ_INFOL << "publish_mode ============mode=================" << mode <<
  // WJ_REND;
  uint16_t msop_len = 1365;
  uint16_t laser_num = 16;
  uint16_t block_num = 10;
  uint16_t chan_num = 16;
  float distance_min = 0.3f;
  float distance_max = 120.0f;
  float distance_resolution = 0.002f;
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
inline DecoderVanjee722Z<T_PointCloud>::DecoderVanjee722Z(const WJDecoderParam &param)
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

  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee722Z<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  std::vector<uint8_t> data;
  if (buf_cache_.size() > 0) {
    std::copy(buf_cache_.begin(), buf_cache_.end(), std::back_inserter(data));
    std::copy(pkt, pkt + size, std::back_inserter(data));
  } else {
    std::copy(pkt, pkt + size, std::back_inserter(data));
  }

  buf_cache_.clear();
  buf_cache_.shrink_to_fit();

  uint32 indexLast = 0;
  for (size_t i = 0; i < data.size(); i++) {
    if (data.size() - i < 34)
      break;
    if (!(data[i] == 0xee && data[i + 1] == 0xff)) {
      indexLast = i + 1;
      continue;
    }

    uint32_t crc_check_34 = this->crc32_mpeg2_padded(&data[i], 30);
    uint32_t crc_pkg_34 = data[i + 30] | (data[i + 31] << 8) | (data[i + 32] << 16) | (data[i + 33] << 24);
    if (crc_check_34 == crc_pkg_34) {
      decodeMsopPktImu(&data[i], 34);
      i += 34 - 1;
      indexLast = i;
      continue;
    } else {
      if (data.size() - i < 80) {
        indexLast = i;
        break;
      } else {
        uint32_t crc_check_80 = this->crc32_mpeg2_padded(&data[i], 76);
        uint32_t crc_pkg_80 = data[i + 76] | (data[i + 77] << 8) | (data[i + 78] << 16) | (data[i + 79] << 24);
        if (crc_check_80 == crc_pkg_80) {
          if (decodeMsopPktSerialPointCloud(&data[i], 80)) {
            ret = true;
            indexLast = i + 80;
            break;
          } else {
            i += 80 - 1;
            indexLast = i;
            continue;
          }
        } else {
          indexLast = i + 1;
          break;
        }
      }
    }
    indexLast = i + 1;
  }

  if (indexLast < data.size()) {
    buf_cache_.assign(data.begin() + indexLast, data.end());
  }

  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee722Z<T_PointCloud>::decodeMsopPktImu(const uint8_t *pkt, size_t size) {
  if (size != sizeof(Vanjee722zSerialImuMsopPkt))
    return false;

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;

  auto &packet = *(Vanjee722zSerialImuMsopPkt *)pkt;
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = packet.data_time[0];  // + 100;
  stm.tm_mon = packet.data_time[1] - 1;
  stm.tm_mday = packet.data_time[2];
  stm.tm_hour = packet.data_time[3];
  stm.tm_min = packet.data_time[4];
  stm.tm_sec = packet.data_time[5];
  double usec = (packet.timestamp[0] + (packet.timestamp[1] << 8) + (packet.timestamp[2] << 16) + ((packet.timestamp[3] & 0x0F) << 24)) * 1e-6;
  pkt_lidar_ts = std::mktime(&stm) + usec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else
    pkt_ts = pkt_lidar_ts;

  uint32_t loss_packets_num = (packet.blocks.sequence_num + 65536 - pre_imu_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_imu_frame_id_ >= 0)
    WJ_WARNING << "imu: loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_imu_frame_id_ = packet.blocks.sequence_num;

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendSerialImuData(packet.blocks, pkt_ts, pkt_lidar_ts);
  }
  return false;
}

template <typename T_PointCloud>
bool DecoderVanjee722Z<T_PointCloud>::decodeMsopPktSerialPointCloud(const uint8_t *pkt, size_t size) {
  if (size != sizeof(Vanjee722zSerialPointCloudMsopPkt))
    return false;

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;

  auto &packet = *(Vanjee722zSerialPointCloudMsopPkt *)pkt;
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = packet.data_time[0];  // + 100;
  stm.tm_mon = packet.data_time[1] - 1;
  stm.tm_mday = packet.data_time[2];
  stm.tm_hour = packet.data_time[3];
  stm.tm_min = packet.data_time[4];
  stm.tm_sec = packet.data_time[5];
  double usec = (packet.timestamp[0] + (packet.timestamp[1] << 8) + (packet.timestamp[2] << 16) + ((packet.timestamp[3] & 0x0F) << 24)) * 1e-6;
  pkt_lidar_ts = std::mktime(&stm) + usec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else
    pkt_ts = pkt_lidar_ts;

  uint32_t loss_packets_num = (packet.blocks.sequence_num + 65536 - pre_point_cloud_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_point_cloud_frame_id_ >= 0)
    WJ_WARNING << "point cloud: loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_point_cloud_frame_id_ = packet.blocks.sequence_num;

  int32_t resolution = 60;

  const Vanjee722zSerialPointCloud &block = packet.blocks;
  int32_t azimuth = block.azimuth % 36000;
  int32_t azimuth_count = (block.azimuth + 30) % 36000;
  int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

  if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans >= 60) {
    uint32_t point_gap_num = (azimuth_count / resolution) * 16;
    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_serial_[point_gap_num];
    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_serial_[all_points_luminous_moment_serial_.size() - 1];
    this->cb_split_frame_(16, this->cloudTs());
    ret = true;
  }

  double timestamp_point;
  for (uint16_t chan = 0; chan < 16; chan++) {
    float x, y, z, xy;

    uint32_t point_id = azimuth_count / resolution * 16 + chan;
    if (this->param_.ts_first_point == true) {
      timestamp_point = all_points_luminous_moment_serial_[point_id];
    } else {
      timestamp_point =
          all_points_luminous_moment_serial_[point_id] - all_points_luminous_moment_serial_[all_points_luminous_moment_serial_.size() - 1];
    }

    const Vanjee722zSerialBlockChannel &channel = block.channel[chan];

    float distance = channel.distance * this->const_param_.DISTANCE_RES;
    int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
    int32_t angle_horiz_final = (this->chan_angles_.horizAdjust(chan, azimuth * 10) + 360000) % 360000;

    // int32_t angle_horiz_mask = 360000 - angle_horiz_final;
    int32_t angle_horiz_mask = angle_horiz_final;
    if (this->param_.start_angle < this->param_.end_angle) {
      if (angle_horiz_mask < this->param_.start_angle * 1000 || angle_horiz_mask > this->param_.end_angle * 1000) {
        distance = 0;
      }
    } else {
      if (angle_horiz_mask > this->param_.end_angle * 1000 && angle_horiz_mask < this->param_.start_angle * 1000) {
        distance = 0;
      }
    }

    if (this->hide_range_params_.size() > 0 && distance != 0 &&
        this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
      distance = 0;
    }

    int32_t azimuth_index = angle_horiz_final;
    int32_t verticalVal_720 = angle_vert;
    int32_t optcent_2_lidar_angle_hor = (azimuth * 10 + optcent_2_lidar_arg_ + 360000) % 360000;

    if (this->distance_section_.in(distance)) {
      xy = distance * COS(verticalVal_720);
      // x = xy * COS(azimuth_index);
      // y = -xy * SIN(azimuth_index);
      x = xy * SIN(azimuth_index) + optcent_2_lidar_l_ * SIN(optcent_2_lidar_angle_hor);
      y = xy * COS(azimuth_index) + optcent_2_lidar_l_ * COS(optcent_2_lidar_angle_hor);
      z = distance * SIN(verticalVal_720) + optcent_2_lidar_z_;
      this->transformPoint(x, y, z);

      typename T_PointCloud::PointT point;
      setX(point, x);
      setY(point, y);
      setZ(point, z);
      setIntensity(point, channel.reflectivity);
      setTimestamp(point, timestamp_point);
      setRing(point, chan + this->first_line_id_);
      setTag(point, 0);
#ifdef ENABLE_GTEST
      setPointId(point, point_id);
      setHorAngle(point, azimuth_index / 1000.0);
      setVerAngle(point, verticalVal_720 / 1000.0);
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
        setX(point, 0);
        setY(point, 0);
        setZ(point, 0);
      }
      setIntensity(point, 0);
      setTimestamp(point, timestamp_point);
      setRing(point, chan + this->first_line_id_);
      setTag(point, 0);
#ifdef ENABLE_GTEST
      setPointId(point, point_id);
      setHorAngle(point, azimuth_index / 1000.0);
      setVerAngle(point, verticalVal_720 / 1000.0);
      setDistance(point, distance);
#endif
      this->point_cloud_->points.emplace_back(point);
    }
  }

  if (azimuth_trans < 60) {
    this->last_point_ts_ = pkt_ts;
    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_serial_[all_points_luminous_moment_serial_.size() - 1];
    this->cb_split_frame_(16, this->cloudTs());
    ret = true;
  }
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::SendSerialImuData(Vanjee722zSerialImu difop, double timestamp, double lidar_timestamp) {
  double imu_angle_voc_x = difop.imu_angle_voc_x / 32.8 * 0.0174533;
  double imu_angle_voc_y = difop.imu_angle_voc_y / 32.8 * 0.0174533;
  double imu_angle_voc_z = difop.imu_angle_voc_z / 32.8 * 0.0174533;

  double imu_linear_acce_x = difop.imu_linear_acce_x / 8192.0 * 9.81;
  double imu_linear_acce_y = difop.imu_linear_acce_y / 8192.0 * 9.81;
  double imu_linear_acce_z = difop.imu_linear_acce_z / 8192.0 * 9.81;

  m_imu_params_get_->rotateImu(imu_linear_acce_x, imu_linear_acce_y, imu_linear_acce_z, imu_angle_voc_x, imu_angle_voc_y, imu_angle_voc_z);

  this->imu_packet_->timestamp = timestamp;
  this->imu_packet_->angular_voc[0] = imu_angle_voc_x;
  this->imu_packet_->angular_voc[1] = imu_angle_voc_y;
  this->imu_packet_->angular_voc[2] = imu_angle_voc_z;

  this->imu_packet_->linear_acce[0] = imu_linear_acce_x;
  this->imu_packet_->linear_acce[1] = imu_linear_acce_y;
  this->imu_packet_->linear_acce[2] = imu_linear_acce_z;

  this->imu_packet_->orientation[0] = 0;
  this->imu_packet_->orientation[1] = 0;
  this->imu_packet_->orientation[2] = 0;
  this->imu_packet_->orientation[3] = 0;

  this->cb_imu_pkt_();
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract722Z> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_ld_value_get_)) {
    p = std::make_shared<Protocol_LDValueGet722Z>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDValue722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    if (!angle_param_get_flag_) {
      std::shared_ptr<Params_LDValue722Z> param = std::dynamic_pointer_cast<Params_LDValue722Z>(params);
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
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar

}  // namespace vanjee
