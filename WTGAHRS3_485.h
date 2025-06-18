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

struct GpsCoordinates
{
  double latitude;  // Vĩ độ
  double longitude; // Kinh độ
  bool isDataValid;
};

struct GpsAccuracyData
{
  uint16_t numSatellites; // Số lượng vệ tinh
  float pdop;             // Position Dilution of Precision (càng thấp càng tốt)
  float hdop;             // Horizontal Dilution of Precision (càng thấp càng tốt)
  float vdop;             // Vertical Dilution of Precision (càng thấp càng tốt)
  bool isDataValid;
};

struct SynchronizedGpsData
{
  GpsCoordinates coordinates;
  GpsMotionData motion;
  GpsAccuracyData accuracy;
  bool isDataValid; // Cờ tổng thể cho biết tất cả dữ liệu GPS có hợp lệ không
};

struct AttitudeData
{
  float roll;  // Góc xoay quanh trục X
  float pitch; // Góc xoay quanh trục Y
  float yaw;   // Góc xoay quanh trục Z
  bool isDataValid;
};

struct OnChipTime
{
  uint8_t year; // e.g., 25 for 2025
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
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

// Thêm cấu trúc này vào phần CÁC CẤU TRÚC DỮ LIỆU
struct QuaternionData
{
  float q0; // Thành phần vô hướng (w)
  float q1; // Thành phần vector (x)
  float q2; // Thành phần vector (y)
  float q3; // Thành phần vector (z)
  bool isDataValid;
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

enum SensorBaudRate : uint16_t
{
  BAUD_4800 = 0x01,
  BAUD_9600 = 0x02,
  BAUD_19200 = 0x03,
  BAUD_38400 = 0x04,
  BAUD_57600 = 0x05,
  BAUD_115200 = 0x06,
  BAUD_230400 = 0x07,
  BAUD_460800 = 0x08,
  BAUD_921600 = 0x09
};

enum SystemCommand : uint16_t
{
  CMD_SAVE_CONFIG = 0x0000,
  CMD_REBOOT_DEVICE = 0x00FF,
  CMD_RESET_CONFIG = 0x0001
};

// --- CẤU TRÚC DỮ LIỆU ĐỒNG BỘ ---
struct SynchronizedSensorData
{
  AccelerationData accel;
  AngularVelocityData gyro;
  AttitudeData attitude;
  MagneticFieldData mag; // Được đọc cùng khối IMU
  GpsCoordinates gpsCoordinates;
  GpsMotionData gpsMotion;     // Dữ liệu chuyển động GPS
  GpsAccuracyData gpsAccuracy; // Dữ liệu độ chính xác GPS
  QuaternionData quaternion;   // Dữ liệu Quaternion

  bool isImuDataValid;        // Cờ cho accel, gyro, attitude, mag
  bool isGpsCoordValid;       // Cờ cho gpsCoordinates
  bool isGpsMotionValid;      // Cờ cho gpsMotion
  bool isGpsAccuracyValid;    // Cờ cho gpsAccuracy
  bool isQuaternionDataValid; // Cờ cho quaternion
};

// --- LỚP THƯ VIỆN CHÍNH ---
class WTGAHRS3_485
{
public:
  // Constructor: nhận một tham chiếu đến đối tượng ModbusMaster đã được khởi tạo
  WTGAHRS3_485(ModbusMaster &node);

  // --- CÁC HÀM ĐỌC DỮ LIỆU CẢM BIẾN ---
  GpsMotionData getGpsMotionData();
  GpsCoordinates getGpsCoordinates();
  GpsAccuracyData getGpsAccuracy();
  AttitudeData getAttitudeValues();
  AccelerationData getAccelerationData();
  AngularVelocityData getAngularVelocityData();
  MagneticFieldData getMagneticFieldData();
  QuaternionData getQuaternionData();

  // --- CÁC HÀM CẤU HÌNH VÀ ĐIỀU KHIỂN ---
  bool setCalibrationCommand(CalibrationCommand command);
  uint16_t getCalibrationSwitchStatus(bool &isValid);

  bool setBandwidth(SensorBandwidth bwValue);
  SensorBandwidth getBandwidth();

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

  OnChipTime getOnChipTime();
  bool setOnChipTime(const OnChipTime &newTime);

  bool setBaudRate(SensorBaudRate baudCode);
  SensorBaudRate getBaudRate(bool &isValid);

  // Hàm tiện ích để chuyển đổi mã baud rate sang số long
  static long baudCodeToLong(SensorBaudRate baudCode);

  // --- HÀM ĐỌC DỮ LIỆU ĐỒNG BỘ ---
  SynchronizedSensorData getSynchronizedData();

  SynchronizedGpsData getSynchronizedGpsData();

  

  // --- HÀM TIỆN ÍCH ---
  static String bandwidthToString(SensorBandwidth bw);

  static AttitudeData quaternionToEuler(const QuaternionData &quat);

private:
  ModbusMaster &_node; // Tham chiếu đến đối tượng ModbusMaster

  // Các hàm nội bộ
  bool unlockRegisters();
  bool sendSystemCommand(SystemCommand cmd);
};

#endif