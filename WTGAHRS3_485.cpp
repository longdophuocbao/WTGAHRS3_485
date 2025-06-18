#include "WTGAHRS3_485.h"

// --- CÁC HẰNG SỐ ĐỊA CHỈ THANH GHI ---
// Được giữ trong file .cpp để giữ cho file .h gọn gàng
const uint16_t SAVE_REGISTER_ADDRESS = 0x0000;
const uint16_t CALSW_REGISTER_ADDRESS = 0x0001;
const uint16_t BAUD_REGISTER_ADDRESS = 0x0004;
const uint16_t AXOFFSET_ADDR = 0x0005;
const uint16_t BANDWIDTH_REGISTER_ADDRESS = 0x001F;
const uint16_t ON_TIME_CHIP_REGISTER_ADDRESS = 0x0030;
const uint16_t ACCELERATION_BASE_REGISTER = 0x0034;
const uint16_t ANGULAR_VELOCITY_BASE_REGISTER = 0x0037;
const uint16_t MAGNETIC_FIELD_BASE_REGISTER = 0x003A;
const uint16_t ATTITUDE_BASE_REGISTER = 0x003D;
const uint16_t GPS_MOTION_BASE_REGISTER = 0x004D;
const uint16_t QUATERNION_BASE_REGISTER = 0x0051;
const uint16_t GPS_ACCURACY_BASE_REGISTER = 0x0055;
const uint16_t KEY_REGISTER_ADDRESS = 0x0069;
const uint16_t MODDELAY_REGISTER_ADDRESS = 0x0074;
const uint16_t GPS_COORD_BASE_REGISTER = 0x0075;

// Hằng số tỉ lệ và giá trị đặc biệt
const float GRAVITATIONAL_ACCELERATION = 9.8f;
const float ACCEL_OFFSET_SCALE_DIVISOR = 10000.0f;
const float GYRO_OFFSET_SCALE_DIVISOR = 10000.0f;
const uint16_t UNLOCK_KEY_VALUE = 0xB588;

// --- CONSTRUCTOR ---
WTGAHRS3_485::WTGAHRS3_485(ModbusMaster &node) : _node(node)
{
  // Constructor chỉ cần khởi tạo tham chiếu đến đối tượng ModbusMaster
}

// --- HÀM NỘI BỘ (PRIVATE) ---
bool WTGAHRS3_485::unlockRegisters()
{
  uint8_t result = _node.writeSingleRegister(KEY_REGISTER_ADDRESS, UNLOCK_KEY_VALUE);
  return (result == _node.ku8MBSuccess);
}

bool WTGAHRS3_485::sendSystemCommand(SystemCommand cmd)
{
  uint8_t result = _node.writeSingleRegister(SAVE_REGISTER_ADDRESS, static_cast<uint16_t>(cmd));
  return (result == _node.ku8MBSuccess);
}

OnChipTime WTGAHRS3_485::getOnChipTime()
{
  OnChipTime currentTime = {0};
  currentTime.isDataValid = false;

  const uint8_t NUM_TIME_REGISTERS = 4;
  // Địa chỉ bắt đầu của khối thời gian là 0x30
  uint8_t result = _node.readHoldingRegisters(ON_TIME_CHIP_REGISTER_ADDRESS, NUM_TIME_REGISTERS);

  if (result == _node.ku8MBSuccess)
  {
    // Tách dữ liệu từ các thanh ghi đã đọc
    uint16_t yymm = _node.getResponseBuffer(0);
    uint16_t ddhh = _node.getResponseBuffer(1);
    uint16_t mmss = _node.getResponseBuffer(2);
    uint16_t ms = _node.getResponseBuffer(3);

    // Byte thấp là Năm, byte cao là Tháng
    currentTime.year = yymm & 0xFF;
    currentTime.month = yymm >> 8;

    // Byte thấp là Ngày, byte cao là Giờ
    currentTime.day = ddhh & 0xFF;
    currentTime.hour = ddhh >> 8;

    // Byte thấp là Phút, byte cao là Giây
    currentTime.minute = mmss & 0xFF;
    currentTime.second = mmss >> 8;

    currentTime.millisecond = ms;

    currentTime.isDataValid = true;
  }
  return currentTime;
}

bool WTGAHRS3_485::setOnChipTime(const OnChipTime &newTime)
{
  // Bước 1: Mở khóa thiết bị
  if (!unlockRegisters())
  {
    // Serial.println("Mở khóa thất bại. Hủy thao tác cài đặt thời gian.");
    return false;
  }

  // Bước 2: Đóng gói dữ liệu vào các biến 16-bit
  uint16_t yymm = (newTime.month << 8) | newTime.year;
  uint16_t ddhh = (newTime.hour << 8) | newTime.day;
  uint16_t mmss = (newTime.second << 8) | newTime.minute;
  uint16_t ms = newTime.millisecond;

  // Bước 3: Ghi lần lượt từng thanh ghi
  bool success = true;
  success &= (_node.writeSingleRegister(0x30, yymm) == _node.ku8MBSuccess);
  success &= (_node.writeSingleRegister(0x31, ddhh) == _node.ku8MBSuccess);
  success &= (_node.writeSingleRegister(0x32, mmss) == _node.ku8MBSuccess);
  success &= (_node.writeSingleRegister(0x33, ms) == _node.ku8MBSuccess);

  return success;
}

bool WTGAHRS3_485::setBaudRate(SensorBaudRate baudCode)
{
  if (unlockRegisters())
  {
    uint8_t result = _node.writeSingleRegister(BAUD_REGISTER_ADDRESS, static_cast<uint16_t>(baudCode));
    return (result == _node.ku8MBSuccess);
  }
  return false;
}

SensorBaudRate WTGAHRS3_485::getBaudRate(bool &isValid)
{
  isValid = false;
  uint8_t result = _node.readHoldingRegisters(BAUD_REGISTER_ADDRESS, 1);
  if (result == _node.ku8MBSuccess)
  {
    isValid = true;
    return static_cast<SensorBaudRate>(_node.getResponseBuffer(0));
  }
  return BAUD_9600; // Trả về giá trị mặc định khi lỗi
}

long WTGAHRS3_485::baudCodeToLong(SensorBaudRate baudCode)
{
  switch (baudCode)
  {
  case BAUD_4800:
    return 4800;
  case BAUD_9600:
    return 9600;
  case BAUD_19200:
    return 19200;
  case BAUD_38400:
    return 38400;
  case BAUD_57600:
    return 57600;
  case BAUD_115200:
    return 115200;
  case BAUD_230400:
    return 230400;
  case BAUD_460800:
    return 460800;
  case BAUD_921600:
    return 921600;
  default:
    return 0;
  }
}

// --- CÁC HÀM ĐỌC DỮ LIỆU CẢM BIẾN ---
GpsMotionData WTGAHRS3_485::getGpsMotionData()
{
  GpsMotionData motionData;
  motionData.isDataValid = false;
  const uint8_t NUM_REGISTERS = 4;
  uint8_t result = _node.readHoldingRegisters(GPS_MOTION_BASE_REGISTER, NUM_REGISTERS);
  if (result == _node.ku8MBSuccess)
  {
    uint16_t rawGPSHeight = _node.getResponseBuffer(0);
    uint16_t rawGPSYaw = _node.getResponseBuffer(1);
    uint16_t rawGPSVL = _node.getResponseBuffer(2);
    uint16_t rawGPSVH = _node.getResponseBuffer(3);
    motionData.altitude = static_cast<float>(static_cast<int16_t>(rawGPSHeight)) / 10.0f;
    motionData.heading = static_cast<float>(rawGPSYaw) / 100.0f;
    uint32_t rawSpeedFull = (static_cast<uint32_t>(rawGPSVH) << 16) | rawGPSVL;
    motionData.groundSpeed = static_cast<float>(rawSpeedFull) / 1000.0f;
    motionData.isDataValid = true;
  }
  return motionData;
}

QuaternionData WTGAHRS3_485::getQuaternionData()
{
  QuaternionData quatData;
  quatData.isDataValid = false;

  const uint8_t NUM_REGISTERS = 4;
  uint8_t result = _node.readHoldingRegisters(QUATERNION_BASE_REGISTER, NUM_REGISTERS);

  if (result == _node.ku8MBSuccess)
  {
    // Đọc giá trị thô từ bộ đệm
    int16_t raw_q0 = static_cast<int16_t>(_node.getResponseBuffer(0));
    int16_t raw_q1 = static_cast<int16_t>(_node.getResponseBuffer(1));
    int16_t raw_q2 = static_cast<int16_t>(_node.getResponseBuffer(2));
    int16_t raw_q3 = static_cast<int16_t>(_node.getResponseBuffer(3));

    // Áp dụng công thức chuyển đổi
    quatData.q0 = static_cast<float>(raw_q0) / 32768.0f;
    quatData.q1 = static_cast<float>(raw_q1) / 32768.0f;
    quatData.q2 = static_cast<float>(raw_q2) / 32768.0f;
    quatData.q3 = static_cast<float>(raw_q3) / 32768.0f;

    quatData.isDataValid = true;
  }

  return quatData;
}

GpsCoordinates WTGAHRS3_485::getGpsCoordinates()
{
  GpsCoordinates coords;
  coords.isDataValid = false;

  // --- CÁC THÔNG TIN NÀY CẦN BẠN CUNG CẤP ---
   
  const uint8_t NUM_REGISTERS_TO_READ = 4;         // (2 cho Lat, 2 cho Lon)
  const double SCALING_FACTOR = 10000000.0;        // <-- VÍ DỤ, CẦN HỆ SỐ THỰC

  uint8_t result = _node.readHoldingRegisters(GPS_COORD_BASE_REGISTER, NUM_REGISTERS_TO_READ);

  if (result == _node.ku8MBSuccess)
  {
    // Giả sử thứ tự là: Lon_High, Lon_Low, Lat_High, Lat_Low
    uint16_t lon_high = _node.getResponseBuffer(0);
    uint16_t lon_low = _node.getResponseBuffer(1);
    uint16_t lat_high = _node.getResponseBuffer(2);
    uint16_t lat_low = _node.getResponseBuffer(3);

    // Ghép các thanh ghi 16-bit thành giá trị 32-bit có dấu
    int32_t raw_lon = (static_cast<int32_t>(lon_high) << 16) | lon_low;
    int32_t raw_lat = (static_cast<int32_t>(lat_high) << 16) | lat_low;

    // Áp dụng công thức chuyển đổi
    coords.longitude = static_cast<double>(raw_lon) / SCALING_FACTOR;
    coords.latitude = static_cast<double>(raw_lat) / SCALING_FACTOR;

    coords.isDataValid = true;
  }

  return coords;
}

GpsAccuracyData WTGAHRS3_485::getGpsAccuracy()
{
  GpsAccuracyData accData;
  accData.isDataValid = false;

  const uint8_t NUM_REGISTERS = 4;
  uint8_t result = _node.readHoldingRegisters(GPS_ACCURACY_BASE_REGISTER, NUM_REGISTERS);

  if (result == _node.ku8MBSuccess)
  {
    // Thanh ghi 0x55: Số lượng vệ tinh
    accData.numSatellites = _node.getResponseBuffer(0);
    // Thanh ghi 0x56: PDOP = giá trị / 100
    accData.pdop = static_cast<float>(_node.getResponseBuffer(1)) / 100.0f;
    // Thanh ghi 0x57: HDOP = giá trị / 100
    accData.hdop = static_cast<float>(_node.getResponseBuffer(2)) / 100.0f;
    // Thanh ghi 0x58: VDOP = giá trị / 100
    accData.vdop = static_cast<float>(_node.getResponseBuffer(3)) / 100.0f;

    accData.isDataValid = true;
  }

  return accData;
}

AttitudeData WTGAHRS3_485::getAttitudeValues()
{
  AttitudeData attitude;
  attitude.isDataValid = false;

  
  const uint8_t NUM_REGISTERS_TO_READ = 3;        // (1 cho Roll, 1 cho Pitch, 1 cho Yaw)
  const double SCALING_FACTOR = 100.0;           

  uint8_t result = _node.readHoldingRegisters(ATTITUDE_BASE_REGISTER, NUM_REGISTERS_TO_READ);

  if (result == _node.ku8MBSuccess)
  {
    // Giả sử thứ tự là: Roll, Pitch, Yaw
    int16_t raw_roll = static_cast<int16_t>(_node.getResponseBuffer(0));
    int16_t raw_pitch = static_cast<int16_t>(_node.getResponseBuffer(1));
    int16_t raw_yaw = static_cast<int16_t>(_node.getResponseBuffer(2));

    // Áp dụng công thức chuyển đổi
    attitude.roll = static_cast<float>(raw_roll) / SCALING_FACTOR;
    attitude.pitch = static_cast<float>(raw_pitch) / SCALING_FACTOR;
    attitude.yaw = static_cast<float>(raw_yaw) / SCALING_FACTOR;

    attitude.isDataValid = true;
  }

  return attitude;
}

AccelerationData WTGAHRS3_485::getAccelerationData()
{
  AccelerationData accData;
  accData.isDataValid = false;
  const uint8_t NUM_REGISTERS = 3;
  uint8_t result = _node.readHoldingRegisters(ACCELERATION_BASE_REGISTER, NUM_REGISTERS);
  if (result == _node.ku8MBSuccess)
  {
    uint16_t rawAX = _node.getResponseBuffer(0);
    uint16_t rawAY = _node.getResponseBuffer(1);
    uint16_t rawAZ = _node.getResponseBuffer(2);
    accData.accelX = static_cast<float>(static_cast<int16_t>(rawAX)) / 32768.0f * 16.0f * GRAVITATIONAL_ACCELERATION;
    accData.accelY = static_cast<float>(static_cast<int16_t>(rawAY)) / 32768.0f * 16.0f * GRAVITATIONAL_ACCELERATION;
    accData.accelZ = static_cast<float>(static_cast<int16_t>(rawAZ)) / 32768.0f * 16.0f * GRAVITATIONAL_ACCELERATION;
    accData.isDataValid = true;
  }
  return accData;
}

AngularVelocityData WTGAHRS3_485::getAngularVelocityData()
{
  AngularVelocityData angVelData;
  angVelData.isDataValid = false;
  const uint8_t NUM_REGISTERS = 3;
  uint8_t result = _node.readHoldingRegisters(ANGULAR_VELOCITY_BASE_REGISTER, NUM_REGISTERS);
  if (result == _node.ku8MBSuccess)
  {
    uint16_t rawGX = _node.getResponseBuffer(0);
    uint16_t rawGY = _node.getResponseBuffer(1);
    uint16_t rawGZ = _node.getResponseBuffer(2);
    angVelData.angularVelX = static_cast<float>(static_cast<int16_t>(rawGX)) / 32768.0f * 2000.0f;
    angVelData.angularVelY = static_cast<float>(static_cast<int16_t>(rawGY)) / 32768.0f * 2000.0f;
    angVelData.angularVelZ = static_cast<float>(static_cast<int16_t>(rawGZ)) / 32768.0f * 2000.0f;
    angVelData.isDataValid = true;
  }
  return angVelData;
}

MagneticFieldData WTGAHRS3_485::getMagneticFieldData()
{
  MagneticFieldData magData;
  magData.isDataValid = false;
  const uint8_t NUM_REGISTERS = 3;
  uint8_t result = _node.readHoldingRegisters(MAGNETIC_FIELD_BASE_REGISTER, NUM_REGISTERS);
  if (result == _node.ku8MBSuccess)
  {
    magData.fieldX = static_cast<int16_t>(_node.getResponseBuffer(0));
    magData.fieldY = static_cast<int16_t>(_node.getResponseBuffer(1));
    magData.fieldZ = static_cast<int16_t>(_node.getResponseBuffer(2));
    magData.isDataValid = true;
  }
  return magData;
}

/* --- CÁC HÀM CẤU HÌNH VÀ ĐIỀU KHIỂN ---
    NORMAL_WORKING_MODE = 0x0000,
    AUTO_ADDER_CALIBRATION = 0x0001,
    HEIGHT_RESET = 0x0003,
    SET_HEADING_ANGLE_TO_ZERO = 0x0004,
    MAG_CAL_SPHERICAL_FITTING = 0x0007,
    SET_ANGLE_REFERENCE = 0x0008,
    MAG_CAL_DUAL_PLANE_MODE = 0x0009 */
bool WTGAHRS3_485::setCalibrationCommand(CalibrationCommand command)
{
  if (unlockRegisters())
  {
    uint8_t result = _node.writeSingleRegister(CALSW_REGISTER_ADDRESS, static_cast<uint16_t>(command));
    return (result == _node.ku8MBSuccess);
  }
  return false;
}

uint16_t WTGAHRS3_485::getCalibrationSwitchStatus(bool &isValid)
{
  isValid = false;
  uint8_t result = _node.readHoldingRegisters(CALSW_REGISTER_ADDRESS, 1);
  if (result == _node.ku8MBSuccess)
  {
    isValid = true;
    return _node.getResponseBuffer(0);
  }
  return 0xFFFF; // Giá trị báo lỗi
}

bool WTGAHRS3_485::setBandwidth(SensorBandwidth bwValue)
{
  if (unlockRegisters())
  {
    uint8_t result = _node.writeSingleRegister(BANDWIDTH_REGISTER_ADDRESS, static_cast<uint16_t>(bwValue));
    return (result == _node.ku8MBSuccess);
  }
  return false;
}

SensorBandwidth WTGAHRS3_485::getBandwidth()
{
  uint8_t result = _node.readHoldingRegisters(BANDWIDTH_REGISTER_ADDRESS, 1);
  if (result == _node.ku8MBSuccess)
  {
    return static_cast<SensorBandwidth>(_node.getResponseBuffer(0));
  }
  return BW_256_HZ; // Giá trị mặc định khi lỗi
}

bool WTGAHRS3_485::setDataResponseDelay(uint16_t delay_us)
{
  if (unlockRegisters())
  {
    uint8_t result = _node.writeSingleRegister(MODDELAY_REGISTER_ADDRESS, delay_us);
    return (result == _node.ku8MBSuccess);
  }
  return false;
}

uint16_t WTGAHRS3_485::getDataResponseDelay(bool &isValid)
{
  isValid = false;
  uint8_t result = _node.readHoldingRegisters(MODDELAY_REGISTER_ADDRESS, 1);
  if (result == _node.ku8MBSuccess)
  {
    isValid = true;
    return _node.getResponseBuffer(0);
  }
  return 0; // Giá trị báo lỗi
}

// --- CÁC HÀM QUẢN LÝ OFFSET ---
bool WTGAHRS3_485::setAccelOffset(uint8_t axis, float offset_g)
{
  if (axis > 2)
    return false;
  if (!unlockRegisters())
    return false;
  int16_t valueToWrite = static_cast<int16_t>(round(offset_g * ACCEL_OFFSET_SCALE_DIVISOR));
  uint8_t result = _node.writeSingleRegister(AXOFFSET_ADDR + axis, valueToWrite);
  return (result == _node.ku8MBSuccess);
}

bool WTGAHRS3_485::setGyroOffset(uint8_t axis, float offset_dps)
{
  if (axis > 2)
    return false;
  if (!unlockRegisters())
    return false;
  int16_t valueToWrite = static_cast<int16_t>(round(offset_dps * GYRO_OFFSET_SCALE_DIVISOR));
  uint8_t result = _node.writeSingleRegister(AXOFFSET_ADDR + 3 + axis, valueToWrite);
  return (result == _node.ku8MBSuccess);
}

bool WTGAHRS3_485::setMagOffset(uint8_t axis, int16_t offset_LSB)
{
  if (axis > 2)
    return false;
  if (!unlockRegisters())
    return false;
  uint8_t result = _node.writeSingleRegister(AXOFFSET_ADDR + 6 + axis, offset_LSB);
  return (result == _node.ku8MBSuccess);
}

SensorOffsets WTGAHRS3_485::getAllSensorOffsets(bool &allReadsValid)
{
  SensorOffsets offsets = {0};
  allReadsValid = false;
  const uint8_t NUM_REGISTERS = 9;
  uint8_t result = _node.readHoldingRegisters(AXOFFSET_ADDR, NUM_REGISTERS);
  if (result == _node.ku8MBSuccess)
  {
    offsets.accelOffsetX_g = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(0))) / ACCEL_OFFSET_SCALE_DIVISOR;
    offsets.accelOffsetY_g = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(1))) / ACCEL_OFFSET_SCALE_DIVISOR;
    offsets.accelOffsetZ_g = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(2))) / ACCEL_OFFSET_SCALE_DIVISOR;
    offsets.angularVelOffsetX_dps = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(3))) / GYRO_OFFSET_SCALE_DIVISOR;
    offsets.angularVelOffsetY_dps = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(4))) / GYRO_OFFSET_SCALE_DIVISOR;
    offsets.angularVelOffsetZ_dps = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(5))) / GYRO_OFFSET_SCALE_DIVISOR;
    offsets.magOffsetX_LSB = static_cast<int16_t>(_node.getResponseBuffer(6));
    offsets.magOffsetY_LSB = static_cast<int16_t>(_node.getResponseBuffer(7));
    offsets.magOffsetZ_LSB = static_cast<int16_t>(_node.getResponseBuffer(8));
    allReadsValid = true;
  }
  return offsets;
}

bool WTGAHRS3_485::writeAllSensorOffsets(const SensorOffsets &offsets)
{
  bool success = true;
  success &= setAccelOffset(0, offsets.accelOffsetX_g);
  success &= setAccelOffset(1, offsets.accelOffsetY_g);
  success &= setAccelOffset(2, offsets.accelOffsetZ_g);
  success &= setGyroOffset(0, offsets.angularVelOffsetX_dps);
  success &= setGyroOffset(1, offsets.angularVelOffsetY_dps);
  success &= setGyroOffset(2, offsets.angularVelOffsetZ_dps);
  success &= setMagOffset(0, offsets.magOffsetX_LSB);
  success &= setMagOffset(1, offsets.magOffsetY_LSB);
  success &= setMagOffset(2, offsets.magOffsetZ_LSB);
  return success;
}

// --- CÁC HÀM HỆ THỐNG QUAN TRỌNG ---
bool WTGAHRS3_485::saveConfiguration()
{
  if (unlockRegisters())
  {
    return sendSystemCommand(CMD_SAVE_CONFIG);
  }
  return false;
}

bool WTGAHRS3_485::rebootDevice()
{
  if (unlockRegisters())
  {
    return sendSystemCommand(CMD_REBOOT_DEVICE);
  }
  return false;
}

bool WTGAHRS3_485::resetConfiguration()
{
  if (unlockRegisters())
  {
    return sendSystemCommand(CMD_RESET_CONFIG);
  }
  return false;
}

// --- HÀM TIỆN ÍCH ---
String WTGAHRS3_485::bandwidthToString(SensorBandwidth bw)
{
  switch (bw)
  {
  case BW_256_HZ:
    return "256 Hz";
  case BW_188_HZ:
    return "188 Hz";
  case BW_98_HZ:
    return "98 Hz";
  case BW_42_HZ:
    return "42 Hz";
  case BW_20_HZ:
    return "20 Hz";
  case BW_10_HZ:
    return "10 Hz";
  case BW_5_HZ:
    return "5 Hz";
  default:
    return "Unknown";
  }
}

// --- HÀM ĐỌC DỮ LIỆU ĐỒNG BỘ ---
SynchronizedSensorData WTGAHRS3_485::getSynchronizedData()
{
  SynchronizedSensorData syncData;
  syncData.isImuDataValid = false;
  syncData.isGpsCoordValid = false;
  syncData.isGpsMotionValid = false;
  syncData.isGpsAccuracyValid = false;
  syncData.isQuaternionDataValid = false;

  // --- Lệnh đọc 1: Đọc dữ liệu Thời gian, IMU & Góc ---
  // Địa chỉ bắt đầu cho IMU & Góc: ACCELERATION_BASE_REGISTER (0x0034)
  // Số thanh ghi: 3 (Accel) + 3 (Gyro) + 3 (Mag) + 3 (Attitude) = 12 thanh ghi (0x34 đến 0x3F)
  const uint16_t IMU_ATTITUDE_START_ADDRESS = ACCELERATION_BASE_REGISTER; // 0x0034
  const uint8_t NUM_IMU_ATTITUDE_REGISTERS = 12; // (0x3F - 0x34 + 1) = 12
  uint8_t resultImuAttitude = _node.readHoldingRegisters(IMU_ATTITUDE_START_ADDRESS, NUM_IMU_ATTITUDE_REGISTERS);

  if (resultImuAttitude == _node.ku8MBSuccess)
  {
    // Phân tích dữ liệu Gia tốc (3 words, offset 0 trong buffer này)
    uint16_t rawAX = _node.getResponseBuffer(0); // 0x34
    uint16_t rawAY = _node.getResponseBuffer(1); // 0x35
    uint16_t rawAZ = _node.getResponseBuffer(2); // 0x36
    syncData.accel.accelX = static_cast<float>(static_cast<int16_t>(rawAX)) / 32768.0f * 16.0f * GRAVITATIONAL_ACCELERATION;
    syncData.accel.accelY = static_cast<float>(static_cast<int16_t>(rawAY)) / 32768.0f * 16.0f * GRAVITATIONAL_ACCELERATION;
    syncData.accel.accelZ = static_cast<float>(static_cast<int16_t>(rawAZ)) / 32768.0f * 16.0f * GRAVITATIONAL_ACCELERATION;
    syncData.accel.isDataValid = true;

    // Phân tích dữ liệu Vận tốc góc (3 words, offset 3)
    uint16_t rawGX = _node.getResponseBuffer(3); // 0x37
    uint16_t rawGY = _node.getResponseBuffer(4); // 0x38
    uint16_t rawGZ = _node.getResponseBuffer(5); // 0x39
    syncData.gyro.angularVelX = static_cast<float>(static_cast<int16_t>(rawGX)) / 32768.0f * 2000.0f;
    syncData.gyro.angularVelY = static_cast<float>(static_cast<int16_t>(rawGY)) / 32768.0f * 2000.0f;
    syncData.gyro.angularVelZ = static_cast<float>(static_cast<int16_t>(rawGZ)) / 32768.0f * 2000.0f;
    syncData.gyro.isDataValid = true;

    // Phân tích dữ liệu Từ trường (3 words, offset 6)
    syncData.mag.fieldX = static_cast<int16_t>(_node.getResponseBuffer(6)); // 0x3A
    syncData.mag.fieldY = static_cast<int16_t>(_node.getResponseBuffer(7)); // 0x3B
    syncData.mag.fieldZ = static_cast<int16_t>(_node.getResponseBuffer(8)); // 0x3C
    syncData.mag.isDataValid = true;

    // Phân tích dữ liệu Góc quay (3 words, offset 9)
    const double ATTITUDE_SCALING_FACTOR = 100.0;
    syncData.attitude.roll = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(9))) / ATTITUDE_SCALING_FACTOR;  // 0x3D
    syncData.attitude.pitch = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(10))) / ATTITUDE_SCALING_FACTOR; // 0x3E
    syncData.attitude.yaw = static_cast<float>(static_cast<int16_t>(_node.getResponseBuffer(11))) / ATTITUDE_SCALING_FACTOR; // 0x3F
    syncData.attitude.isDataValid = true;

    syncData.isImuDataValid = syncData.accel.isDataValid && syncData.gyro.isDataValid && syncData.mag.isDataValid && syncData.attitude.isDataValid;
  }


  // --- Lệnh đọc 2: Đọc dữ liệu Định vị GPS ---
  // Địa chỉ bắt đầu: 0x0049 - 12 thanh ghi (0x49 đến 0x50)

  const uint16_t GPS_NAV_QUAT_START_ADDRESS = 0x0049; // Địa chỉ bắt đầu không đổi
  const uint8_t NUM_GPS_NAV_QUAT_REGISTERS = 12;      // 4 (Coords) + 4 (Motion) + 4 (Quaternion)
  const double GPS_COORD_SCALING_FACTOR = 10000000.0;
  uint8_t resultGpsNavQuat = _node.readHoldingRegisters(GPS_NAV_QUAT_START_ADDRESS, NUM_GPS_NAV_QUAT_REGISTERS);

  if (resultGpsNavQuat == _node.ku8MBSuccess)
  {
    // Tọa độ (offset 0 trong buffer này)
    uint16_t lon_low = _node.getResponseBuffer(0);  // 0x49 LonL
    uint16_t lon_high = _node.getResponseBuffer(1); // 0x4A LonH
    uint16_t lat_low = _node.getResponseBuffer(2);  // 0x4B LatL
    uint16_t lat_high = _node.getResponseBuffer(3); // 0x4C LatH

    // Ghép lại, giả sử LonH và LatH là phần cao
    int32_t raw_lon = (static_cast<int32_t>(lon_high) << 16) | lon_low; // (LonH << 16) | LonL
    int32_t raw_lat = (static_cast<int32_t>(lat_high) << 16) | lat_low; // (LatH << 16) | LatL

    syncData.gpsCoordinates.longitude = static_cast<double>(raw_lon) / GPS_COORD_SCALING_FACTOR;
    syncData.gpsCoordinates.latitude = static_cast<double>(raw_lat) / GPS_COORD_SCALING_FACTOR;
    syncData.gpsCoordinates.isDataValid = true;
    syncData.isGpsCoordValid = true;

    // Dữ liệu chuyển động GPS (offset 4 trong buffer này)
    uint16_t rawGPSHeight = _node.getResponseBuffer(4); // 0x4D GPSHeight
    uint16_t rawGPSYaw = _node.getResponseBuffer(5);    // 0x4E GPSYAW (GPS Heading)
    uint16_t rawGPSVL = _node.getResponseBuffer(6);     // 0x4F GPSVL (Ground speed low word)
    uint16_t rawGPSVH = _node.getResponseBuffer(7);     // 0x50 GPSVH (Ground speed high word)

    syncData.gpsMotion.altitude = static_cast<float>(static_cast<int16_t>(rawGPSHeight)) / 10.0f;
    syncData.gpsMotion.heading = static_cast<float>(rawGPSYaw) / 100.0f;
    uint32_t rawSpeedFull = (static_cast<uint32_t>(rawGPSVH) << 16) | rawGPSVL;
    syncData.gpsMotion.groundSpeed = static_cast<float>(rawSpeedFull) / 1000.0f; // km/h
    syncData.gpsMotion.isDataValid = true;
    syncData.isGpsMotionValid = true;

    // Dữ liệu Quaternion (offset 8 trong buffer này)
    int16_t raw_q0 = static_cast<int16_t>(_node.getResponseBuffer(8));  // 0x51
    int16_t raw_q1 = static_cast<int16_t>(_node.getResponseBuffer(9));  // 0x52
    int16_t raw_q2 = static_cast<int16_t>(_node.getResponseBuffer(10)); // 0x53
    int16_t raw_q3 = static_cast<int16_t>(_node.getResponseBuffer(11)); // 0x54

    syncData.quaternion.q0 = static_cast<float>(raw_q0) / 32768.0f;
    syncData.quaternion.q1 = static_cast<float>(raw_q1) / 32768.0f;
    syncData.quaternion.q2 = static_cast<float>(raw_q2) / 32768.0f;
    syncData.quaternion.q3 = static_cast<float>(raw_q3) / 32768.0f;
    syncData.quaternion.isDataValid = true;
    syncData.isQuaternionDataValid = true;
  }


  // --- Lệnh đọc 3: Đọc dữ liệu Chất lượng GPS ---
  // Địa chỉ bắt đầu: GPS_ACCURACY_BASE_REGISTER (0x0055)
  // Số lượng thanh ghi: 4
  const uint8_t NUM_GPS_ACCURACY_REGISTERS = 4;
  uint8_t resultGpsAccuracy = _node.readHoldingRegisters(GPS_ACCURACY_BASE_REGISTER, NUM_GPS_ACCURACY_REGISTERS);

  if (resultGpsAccuracy == _node.ku8MBSuccess)
  {
    syncData.gpsAccuracy.numSatellites = _node.getResponseBuffer(0); // 0x55 SVNUM
    syncData.gpsAccuracy.pdop = static_cast<float>(_node.getResponseBuffer(1)) / 100.0f; // 0x56 PDOP
    syncData.gpsAccuracy.hdop = static_cast<float>(_node.getResponseBuffer(2)) / 100.0f; // 0x57 HDOP
    syncData.gpsAccuracy.vdop = static_cast<float>(_node.getResponseBuffer(3)) / 100.0f; // 0x58 VDOP
    syncData.gpsAccuracy.isDataValid = true;
    syncData.isGpsAccuracyValid = true;
  }

  return syncData;
}