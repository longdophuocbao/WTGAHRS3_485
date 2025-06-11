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
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // RXD2 and TXD2 are the pins for RS485 communication
  // Modbus communication runs at 115200 baud
  // Node uses Serial (UART0)
  node.begin(SENSOR_SLAVE_ID, RS485);
  delay(500); // Wait for ModbusMaster to initialize)

  // If you are using RS485, you might need to set the transmission
  // control pins. Attach them to the ModbusMaster object.
  // node.preTransmission(preTransmission);
  // node.postTransmission(postTransmission);

  Serial.println("WTGAHRS3 485 - Read All Data Example");
}

void loop()
{
  // --- Read Acceleration ---
  AccelerationData accel = sensor.getAccelerationData();
  if (accel.isDataValid)
  {
    Serial.print("Accel (m/s^2): ");
    Serial.print("X=");
    Serial.print(accel.accelX, 4);
    Serial.print(", Y=");
    Serial.print(accel.accelY, 4);
    Serial.print(", Z=");
    Serial.println(accel.accelZ, 4);
  }
  else
  {
    Serial.println("Failed to read acceleration data.");
  }

  // --- Read Angular Velocity ---
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

  // --- Read Magnetic Field ---
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

  // --- Read GPS Motion Data ---
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

  Serial.println("------------------------------------");
  delay(1000); // Wait a second before the next read
}