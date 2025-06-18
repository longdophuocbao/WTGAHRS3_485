/**
 * @file ReadAllData.ino
 * @brief Example for WTGAHRS3_485 library.
 * * This sketch demonstrates how to read all available data from the sensor:
 * Acceleration, Angular Velocity, Magnetic Field, and GPS Motion Data.
 * * You need to have the ModbusMaster and WTGAHRS3_485 libraries installed.
 */
#include <Arduino.h>
#include <ModbusMaster.h>
#include <WTGAHRS3_485.h>

const uint8_t RXD2 = 16;
const uint8_t TXD2 = 17;

// instantiate ModbusMaster object
ModbusMaster node;
HardwareSerial RS485(1);
// instantiate WTGAHRS3_485 object, passing the ModbusMaster object
WTGAHRS3_485 sensor(node);

// Define the slave ID for your sensor
#define SENSOR_SLAVE_ID 0x50

void setup()
{
  Serial.begin(115200);
  delay(500); // Wait for Serial to initialize
  RS485.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(500); // Wait for RS485 to initialize

  // Node uses Serial (UART0)
  node.begin(SENSOR_SLAVE_ID, RS485);
  delay(500); // Wait for ModbusMaster to initialize)


  Serial.println("WTGAHRS3 485 - Read All Data Example");
}

void loop()
{
  // -- -Read Attitude Data(Roll, Pitch, Yaw)-- -

  AttitudeData attitude = sensor.getAttitudeValues();
  if (attitude.isDataValid)
  {
    Serial.print("Attitude (°):  ");
    Serial.print("Roll=");
    Serial.print(attitude.roll, 2);
    Serial.print(", Pitch=");
    Serial.print(attitude.pitch, 2);
    Serial.print(", Yaw=");
    Serial.println(attitude.yaw, 2);
  }
  else
  {
    Serial.println("Failed to read attitude data.");
  }

  // --- Đọc dữ liệu gia tốc ---
  AccelerationData accel = sensor.getAccelerationData();
  if (accel.isDataValid)
  {
    Serial.print("Accel (m/s^2): ");
    Serial.print("X=");
    Serial.print(accel.accelX);
    Serial.print(", Y=");
    Serial.print(accel.accelY);
    Serial.print(", Z=");
    Serial.println(accel.accelZ);
  }
  else
  {
    Serial.println("Failed to read acceleration data.");
  }

  // --- Đọc vận tốc góc ---
  AngularVelocityData gyro = sensor.getAngularVelocityData();
  if (gyro.isDataValid)
  {
    Serial.print("Gyro (°/s):    ");
    Serial.print("X=");
    Serial.print(gyro.angularVelX, 4);
    Serial.print(", Y=");
    Serial.print(gyro.angularVelY, 4);
    Serial.print(", Z=");
    Serial.println(gyro.angularVelZ, 4);
  }
  else
  {
    Serial.println("Failed to read angular velocity data.");
  }

  // --- Đọc từ trường ---
  MagneticFieldData mag = sensor.getMagneticFieldData();
  if (mag.isDataValid)
  {
    Serial.print("Mag (LSB):     ");
    Serial.print("X=");
    Serial.print(mag.fieldX);
    Serial.print(", Y=");
    Serial.print(mag.fieldY);
    Serial.print(", Z=");
    Serial.println(mag.fieldZ);
  }
  else
  {
    Serial.println("Failed to read magnetic field data.");
  }

  // --- Đọc dữ liệu chuyển động GPS ---
  GpsMotionData gps = sensor.getGpsMotionData();
  if (gps.isDataValid)
  {
    Serial.print("GPS Motion:    ");
    Serial.print("Alt(m)=");
    Serial.print(gps.altitude, 6);
    Serial.print(", Head(°)= ");
    Serial.print(gps.heading, 6);
    Serial.print(", Spd(km/h)=");
    Serial.println(gps.groundSpeed, 6);
  }
  else
  {
    Serial.println("Failed to read GPS motion data.");
  }

  GpsCoordinates coords = sensor.getGpsCoordinates(); // Đọc tọa độ GPS
  if (coords.isDataValid)
  {
    Serial.print("GPS Coords:    ");
    Serial.print("Lat=");
    Serial.print(coords.latitude, 7); // In với 7 chữ số thập phân cho độ chính xác cao
    Serial.print(", Lon=");
    Serial.println(coords.longitude, 7);
  }
  else
  {
    Serial.println("Failed to read GPS coordinates.");
  }

  // --- Đọc dữ liệu Quaternion ---
  QuaternionData quat = sensor.getQuaternionData();
  if (quat.isDataValid)
  {
    Serial.print("Quaternion:    ");
    Serial.print("q0=");
    Serial.print(quat.q0, 4);
    Serial.print(", q1=");
    Serial.print(quat.q1, 4);
    Serial.print(", q2=");
    Serial.print(quat.q2, 4);
    Serial.print(", q3=");
    Serial.println(quat.q3, 4);
  }
  else
  {
    Serial.println("Failed to read quaternion data.");
  }
  // --- Đọc dữ liệu độ chính xác GPS ---
  GpsAccuracyData accuracy = sensor.getGpsAccuracy();
  if (accuracy.isDataValid)
  {
    Serial.print("GPS Accuracy:  ");
    Serial.print("Sats=");
    Serial.print(accuracy.numSatellites);
    Serial.print(", PDOP=");
    Serial.print(accuracy.pdop, 2);
    Serial.print(", HDOP=");
    Serial.print(accuracy.hdop, 2);
    Serial.print(", VDOP=");
    Serial.println(accuracy.vdop, 2);
  }
  else
  {
    Serial.println("Failed to read GPS accuracy data.");
  }

  // --- Đọc thời gian chip ---
  OnChipTime chipTime = sensor.getOnChipTime();
  if (chipTime.isDataValid)
  {
    Serial.print("Chip Time:     ");
    Serial.print(chipTime.year, DEC);
    Serial.print("/");
    Serial.print(chipTime.month, DEC);
    Serial.print("/");
    Serial.print(chipTime.day, DEC);
    Serial.print(" ");
    Serial.print(chipTime.hour, DEC);
    Serial.print(":");
    Serial.print(chipTime.minute, DEC);
    Serial.print(":");
    Serial.print(chipTime.second, DEC);
    Serial.print(".");
    Serial.println(chipTime.millisecond, DEC);
  }
  else
  {
    Serial.println("Failed to read chip time.");
  }
  // --- Đọc trạng thái công tắc hiệu chuẩn ---
  bool calibSwitchValid;
  uint16_t calibSwitchStatus = sensor.getCalibrationSwitchStatus(calibSwitchValid);
  if (calibSwitchValid)
  {
    Serial.print("Calibration Switch Status: 0x");
    Serial.println(calibSwitchStatus, HEX);
  }
  else
  {
    Serial.println("Failed to read calibration switch status.");
  }

  // --- Đọc độ trễ phản hồi dữ liệu ---
  bool delayValid;
  uint16_t dataDelay = sensor.getDataResponseDelay(delayValid);
  if (delayValid)
  {
    Serial.print("Data Response Delay: ");
    Serial.print(dataDelay);
    Serial.println(" us");
  }
  else
  {
    Serial.println("Failed to read data response delay.");
  }

  // --- Đọc tất cả các offset cảm biến ---
  bool offsetsValid;
  SensorOffsets offsets = sensor.getAllSensorOffsets(offsetsValid);
  if (offsetsValid)
  {
    Serial.println("Sensor Offsets:");
    Serial.print("  Accel: X=");
    Serial.print(offsets.accelOffsetX_g, 4);
    Serial.print(" g, Y=");
    Serial.print(offsets.accelOffsetY_g, 4);
    Serial.print(" g, Z=");
    Serial.print(offsets.accelOffsetZ_g, 4);
    Serial.println(" g");
    Serial.print("  Gyro:  X=");
    Serial.print(offsets.angularVelOffsetX_dps, 4);
    Serial.print(" dps, Y=");
    Serial.print(offsets.angularVelOffsetY_dps, 4);
    Serial.print(" dps, Z=");
    Serial.print(offsets.angularVelOffsetZ_dps, 4);
    Serial.println(" dps");
    Serial.print("  Mag:   X=");
    Serial.print(offsets.magOffsetX_LSB);
    Serial.print(" LSB, Y=");
    Serial.print(offsets.magOffsetY_LSB);
    Serial.print(" LSB, Z=");
    Serial.print(offsets.magOffsetZ_LSB);
    Serial.println(" LSB");
  }

  Serial.println("------------------------------------");
  delay(1000); // Wait a second before the next read
}