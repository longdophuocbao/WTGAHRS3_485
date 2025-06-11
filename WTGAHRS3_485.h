#ifndef WTGAHRS3_485_H
#define WTGAHRS3_485_H

#include <Arduino.h>
#include <ModbusMaster.h>

// --- CÁC CẤU TRÚC DỮ LIỆU ---
// (Dữ liệu ban đầu bạn cung cấp)
struct GpsMotionData
{
  float altitude;
  float heading;
  float groundSpeed;
  bool isDataValid;
};

struct AccelerationData
{
  float accelX;
  float accelY;
  float accelZ;
  bool isDataValid;
};

struct AngularVelocityData
{
  float angularVelX;
  float angularVelY;
  float angularVelZ;
  bool isDataValid;
};

struct MagneticFieldData
{
  int16_t fieldX;
  int16_t fieldY;
  int16_t fieldZ;
  bool isDataValid;
};

struct SensorOffsets
{
  float accelOffsetX_g;
  float accelOffsetY_g;
  float accelOffsetZ_g;
  float angularVelOffsetX_dps;
  float angularVelOffsetY_dps;
  float angularVelOffsetZ_dps;
  int16_t magOffsetX_LSB;
  int16_t magOffsetY_LSB;
  int16_t magOffsetZ_LSB;
};

// --- CÁC ENUM CHO LỆNH VÀ CÀI ĐẶT ---
enum CalibrationCommand : uint16_t
{
  NORMAL_WORKING_MODE = 0x0000,
  AUTO_ADDER_CALIBRATION = 0x0001,
  HEIGHT_RESET = 0x0003,
  SET_HEADING_ANGLE_TO_ZERO = 0x0004,
  MAG_CAL_SPHERICAL_FITTING = 0x0007,
  SET_ANGLE_REFERENCE = 0x0008,
  MAG_CAL_DUAL_PLANE_MODE = 0x0009
};

enum SensorBandwidth : uint16_t
{
  BW_256_HZ = 0x0000,
  BW_188_HZ = 0x0001,
  BW_98_HZ = 0x0002,
  BW_42_HZ = 0x0003,
  BW_20_HZ = 0x0004,
  BW_10_HZ = 0x0005,
  BW_5_HZ = 0x0006
};

enum SystemCommand : uint16_t
{
  CMD_SAVE_CONFIG = 0x0000,
  CMD_REBOOT_DEVICE = 0x00FF,
  CMD_RESET_CONFIG = 0x0001
};

// --- LỚP THƯ VIỆN CHÍNH ---
class WTGAHRS3_485
{
public:
  // Constructor: nhận một tham chiếu đến đối tượng ModbusMaster đã được khởi tạo
  WTGAHRS3_485(ModbusMaster &node);

  // --- CÁC HÀM ĐỌC DỮ LIỆU CẢM BIẾN ---
  GpsMotionData getGpsMotionData();
  AccelerationData getAccelerationData();
  AngularVelocityData getAngularVelocityData();
  MagneticFieldData getMagneticFieldData();

  // --- CÁC HÀM CẤU HÌNH VÀ ĐIỀU KHIỂN ---
  bool setCalibrationCommand(CalibrationCommand command);
  uint16_t getCalibrationSwitchStatus(bool &isValid);

  bool setBandwidth(SensorBandwidth bwValue);
  SensorBandwidth getBandwidth(bool &isValid);

  bool setDataResponseDelay(uint16_t delay_us);
  uint16_t getDataResponseDelay(bool &isValid);

  // --- CÁC HÀM QUẢN LÝ OFFSET ---
  bool setAccelOffset(uint8_t axis, float offset_g);   // 0=X, 1=Y, 2=Z
  bool setGyroOffset(uint8_t axis, float offset_dps);  // 0=X, 1=Y, 2=Z
  bool setMagOffset(uint8_t axis, int16_t offset_LSB); // 0=X, 1=Y, 2=Z

  SensorOffsets getAllSensorOffsets(bool &allReadsValid);
  bool writeAllSensorOffsets(const SensorOffsets &offsets);

  // --- CÁC HÀM HỆ THỐNG QUAN TRỌNG ---
  bool saveConfiguration();
  bool rebootDevice();
  bool resetConfiguration();

  // --- HÀM TIỆN ÍCH ---
  static String bandwidthToString(SensorBandwidth bw);

private:
  ModbusMaster &_node; // Tham chiếu đến đối tượng ModbusMaster

  // Các hàm nội bộ
  bool unlockRegisters();
  bool sendSystemCommand(SystemCommand cmd);
};

#endif