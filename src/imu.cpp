#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "imu.h"
#include "motion.h"
#include "maze.h"
#include "robot.h"

float gyroZ_offset = 0.0f;

void setupImu()
{
  // Đánh thức ICM-20602.
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(10);

  // Chọn gyro ±250 độ/giây, tương ứng raw / 131.
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Hiệu chuẩn gyro khi xe đứng yên.
  long sumZ = 0;
  int validReadings = 0;

  for (int i = 0; i < 500; i++)
  {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);

    if (Wire.requestFrom(ICM_ADDR, (uint8_t)2, (uint8_t)true) == 2)
    {
      uint8_t high = Wire.read();
      uint8_t low = Wire.read();
      int16_t gz = (high << 8) | low;

      sumZ += gz;
      validReadings++;
    }
    delay(2);
  }

  if (validReadings > 0)
  {
    gyroZ_offset = (float)sumZ / validReadings;
  }
  else
  {
    gyroZ_offset = 0.0f;
  }
}

float getGyroZRate()
{
  static float lastValidRate = 0.0f;

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);

  if (Wire.requestFrom(ICM_ADDR, (uint8_t)2, (uint8_t)true) == 2)
  {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    int16_t gz = (high << 8) | low;

    lastValidRate = ((float)gz - gyroZ_offset) / 131.0f;
  }

  return lastValidRate;
}
