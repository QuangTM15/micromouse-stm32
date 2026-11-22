#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "imu.h"
#include "motion.h"
#include "maze.h"
#include "robot.h"

VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;
uint16_t currentDistL = 0;
uint16_t currentDistR = 0;

bool initSensor(VL53L0X &sensor, uint8_t xshutPin,
                uint8_t newAddr)
{
  digitalWrite(xshutPin, HIGH);
  delay(20);

  sensor.setTimeout(1000);
  if (!sensor.init())
  {
    return false;
  }

  sensor.setAddress(newAddr);
  delay(10);
  sensor.setMeasurementTimingBudget(33000);
  sensor.startContinuous();
  return true;
}

void setupSensors()
{
  Wire.setSCL(I2C_SCL_PIN);
  Wire.setSDA(I2C_SDA_PIN);
  Wire.begin();
  Wire.setClock(50000);

  pinMode(U3_XSHUT_PIN, OUTPUT);
  pinMode(U4_XSHUT_PIN, OUTPUT);
  pinMode(U5_XSHUT_PIN, OUTPUT);
  pinMode(U6_XSHUT_PIN, OUTPUT);
  pinMode(U2_XSHUT_PIN, OUTPUT);

  digitalWrite(U3_XSHUT_PIN, LOW);
  digitalWrite(U4_XSHUT_PIN, LOW);
  digitalWrite(U5_XSHUT_PIN, LOW);
  digitalWrite(U6_XSHUT_PIN, LOW);
  digitalWrite(U2_XSHUT_PIN, LOW);
  delay(100);

  initSensor(sensorRight, U4_XSHUT_PIN, U4_I2C_ADDR);
  initSensor(sensorLeft, U5_XSHUT_PIN, U5_I2C_ADDR);
  initSensor(sensorFront, U6_XSHUT_PIN, U6_I2C_ADDR);

  setupImu();
}
