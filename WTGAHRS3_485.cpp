#include "WTGAHRS3_485.h"

// --- CÁC HẰNG SỐ ĐỊA CHỈ THANH GHI ---
// Được giữ trong file .cpp để giữ cho file .h gọn gàng
const uint16_t SAVE_REGISTER_ADDRESS = 0x0000;
const uint16_t CALSW_REGISTER_ADDRESS = 0x0001;
const uint16_t AXOFFSET_ADDR = 0x0005;
const uint16_t BANDWIDTH_REGISTER_ADDRESS = 0x001F;
const uint16_t ACCELERATION_BASE_REGISTER = 0x0034;
const uint16_t ANGULAR_VELOCITY_BASE_REGISTER = 0x0037;
const uint16_t MAGNETIC_FIELD_BASE_REGISTER = 0x003A;
const uint16_t GPS_MOTION_BASE_REGISTER = 0x004D;
const uint16_t KEY_REGISTER_ADDRESS = 0x0069;
const uint16_t MODDELAY_REGISTER_ADDRESS = 0x0074;

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

// --- CÁC HÀM CẤU HÌNH VÀ ĐIỀU KHIỂN ---
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

SensorBandwidth WTGAHRS3_485::getBandwidth(bool &isValid)
{
  isValid = false;
  uint8_t result = _node.readHoldingRegisters(BANDWIDTH_REGISTER_ADDRESS, 1);
  if (result == _node.ku8MBSuccess)
  {
    isValid = true;
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