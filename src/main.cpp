
#include <Arduino.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <algorithm>

#include "config.h"
#include "motion.h"
#include "motion_config.h"

// =====================================================
// CONFIGURATION
// =====================================================

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 200;

constexpr size_t COMMAND_BUFFER_SIZE = 64;

// =====================================================
// GLOBAL VARIABLES
// =====================================================

bool motionReady = false;
bool errorPrinted = false;

uint32_t lastPrintMs = 0;
uint32_t commandStartMs = 0;

MotionState previousState =
    MotionState::Uninitialized;

char commandBuffer[COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0;

// =====================================================
// MOTION STATE NAME
// =====================================================

const char *stateName(MotionState state)
{
  switch (state)
  {
  case MotionState::Uninitialized:
    return "UNINITIALIZED";

  case MotionState::Initializing:
    return "INITIALIZING";

  case MotionState::Idle:
    return "IDLE";

  case MotionState::Moving:
    return "MOVING";

  case MotionState::Turning:
    return "TURNING";

  case MotionState::Stopping:
    return "STOPPING";

  case MotionState::Settling:
    return "SETTLING";

  case MotionState::Completed:
    return "COMPLETED";

  case MotionState::Error:
    return "ERROR";

  default:
    return "UNKNOWN";
  }
}

// =====================================================
// READ PWM FROM TIM2
// =====================================================

int readLeftPwm()
{
  const uint32_t period = TIM2->ARR + 1U;

  if (period == 0U)
  {
    return 0;
  }

  const int64_t difference =
      static_cast<int64_t>(TIM2->CCR2) -
      static_cast<int64_t>(TIM2->CCR3);

  int pwm = static_cast<int>(
      difference * 255 / period);

  if (board::reverseLeftMotor)
  {
    pwm = -pwm;
  }

  return pwm;
}

int readRightPwm()
{
  const uint32_t period = TIM2->ARR + 1U;

  if (period == 0U)
  {
    return 0;
  }

  // New right motor mapping:
  // PA3 = TIM2 CH4 (PWM)
  // PA4 = GPIO (direction)

  const int duty = std::clamp(
      static_cast<int>(
          static_cast<uint64_t>(TIM2->CCR4) *
          255U / period),
      0,
      255);

  const bool in4High =
      digitalRead(board::rightIn2) == HIGH;

  // DRV8833:
  // IN4 LOW  -> forward PWM
  // IN4 HIGH -> reverse via inverted PWM
  int pwm = in4High ? duty - 255 : duty;

  if (board::reverseRightMotor)
  {
    pwm = -pwm;
  }

  return pwm;
}

// =====================================================
// PRINT MOTOR PWM
// =====================================================

void printMotorPwm()
{
  const bool enabled =
      digitalRead(board::motorSleep) == HIGH;

  Serial.print(" PWM_L=");
  Serial.print(readLeftPwm());

  Serial.print(" PWM_R=");
  Serial.print(readRightPwm());

  Serial.print(" ENABLE=");
  Serial.print(enabled ? 1 : 0);
}

// =====================================================
// PRINT TELEMETRY
// =====================================================

void printTelemetry()
{
  const MotionStatus status = motionGetStatus();

  const MotionEncoderData encoder =
      motionGetEncoderData();

  Serial.print("[MOTION]");

  Serial.print(" t=");
  Serial.print(millis() - commandStartMs);

  Serial.print(" state=");
  Serial.print(stateName(status.state));

  Serial.print(" L=");
  Serial.print(static_cast<long>(encoder.leftCounts));

  Serial.print(" R=");
  Serial.print(static_cast<long>(encoder.rightCounts));

  Serial.print(" Lmm=");
  Serial.print(encoder.leftMm, 2);

  Serial.print(" Rmm=");
  Serial.print(encoder.rightMm, 2);

  Serial.print(" DIST=");
  Serial.print(status.travelMm, 2);

  Serial.print(" DIFF=");
  Serial.print(
      encoder.leftMm - encoder.rightMm,
      2);

  Serial.print(" YAW=");
  Serial.print(status.headingDeg, 2);

  Serial.print(" GYRO=");
  Serial.print(status.rateDegPerSec, 2);

  printMotorPwm();

  Serial.print(" FAULT=");
  Serial.println(static_cast<int>(status.fault));
}

// =====================================================
// PID CONFIGURATION
// =====================================================

void printPidConfig()
{
  Serial.println();
  Serial.println("========== MOTION CONFIG ==========");

  Serial.print("Heading KP: ");
  Serial.println(kMotionConfig.headingPid.kp, 3);

  Serial.print("Heading KI: ");
  Serial.println(kMotionConfig.headingPid.ki, 3);

  Serial.print("Heading KD: ");
  Serial.println(kMotionConfig.headingPid.kd, 3);

  Serial.print("Straight PWM: ");
  Serial.println(kMotionConfig.straightBasePwm);

  Serial.print("Max Motor PWM: ");
  Serial.println(kMotionConfig.maxMotorPwm);

  Serial.print("Left Counts/Rev: ");
  Serial.println(kMotionConfig.leftCountsPerRev);

  Serial.print("Right Counts/Rev: ");
  Serial.println(kMotionConfig.rightCountsPerRev);

  Serial.println("===================================");
}

// =====================================================
// COMMAND HELP
// =====================================================

void printHelp()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println(" MICROMOUSE FULL MOTION TEST");
  Serial.println("======================================");

  Serial.println(" f       : Continuous forward");
  Serial.println(" b       : Continuous backward");
  Serial.println(" d 100   : Forward 100 counts");
  Serial.println(" b 100   : Backward 100 counts");

  Serial.println(" l 90    : Turn left 90 degrees");
  Serial.println(" r 90    : Turn right 90 degrees");

  Serial.println(" h 3     : Heading offset +3 degrees");
  Serial.println(" h 0     : Clear heading offset");

  Serial.println(" s       : Stop motion");
  Serial.println(" x       : Emergency stop");

  Serial.println(" p       : Print telemetry");
  Serial.println(" c       : Print PID config");
  Serial.println(" ?       : Show commands");

  Serial.println();
  Serial.println("NOTE:");
  Serial.println("Continuous commands have no timeout.");
  Serial.println("Send 's' to stop.");
  Serial.println("======================================");
}

// =====================================================
// ARGUMENT UTILITIES
// =====================================================

bool isBlank(const char *text)
{
  while (*text != '\0')
  {
    if (!std::isspace(
            static_cast<unsigned char>(*text)))
    {
      return false;
    }

    ++text;
  }

  return true;
}

bool parseNumber(const char *text, float &value)
{
  while (std::isspace(
      static_cast<unsigned char>(*text)))
  {
    ++text;
  }

  if (*text == '\0')
  {
    return false;
  }

  char *end = nullptr;

  value = std::strtof(text, &end);

  if (end == text || !std::isfinite(value))
  {
    return false;
  }

  while (std::isspace(
      static_cast<unsigned char>(*end)))
  {
    ++end;
  }

  return *end == '\0';
}

bool parseCounts(const char *text, int32_t &value)
{
  float number = 0.0f;

  if (!parseNumber(text, number))
  {
    return false;
  }

  if (number < -1000000.0f ||
      number > 1000000.0f)
  {
    return false;
  }

  const int32_t converted =
      static_cast<int32_t>(number);

  if (static_cast<float>(converted) != number)
  {
    return false;
  }

  value = converted;

  return true;
}

// =====================================================
// REPORT COMMAND
// =====================================================

void reportCommand(bool accepted)
{
  Serial.print("[COMMAND] ");

  if (accepted)
  {
    Serial.println("ACCEPTED");

    commandStartMs = millis();
    lastPrintMs = commandStartMs;

    printTelemetry();
  }
  else
  {
    Serial.println("REJECTED");
    Serial.println(
        "[INFO] Check Motion state and target");
  }
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================

void executeCommand(char *line)
{
  while (std::isspace(
      static_cast<unsigned char>(*line)))
  {
    ++line;
  }

  if (*line == '\0')
  {
    return;
  }

  const char command =
      static_cast<char>(
          std::tolower(
              static_cast<unsigned char>(*line)));

  const char *argument = line + 1;

  Serial.println();

  // =================================================
  // STOP
  // =================================================

  if (command == 's')
  {
    Serial.println("[COMMAND] STOP");

    motionStop();

    printTelemetry();
    return;
  }

  // =================================================
  // EMERGENCY STOP
  // =================================================

  if (command == 'x')
  {
    Serial.println("[COMMAND] EMERGENCY STOP");

    motionEmergencyStop();

    printTelemetry();
    return;
  }

  // =================================================
  // INFORMATION
  // =================================================

  if (command == 'p')
  {
    printTelemetry();
    return;
  }

  if (command == 'c')
  {
    printPidConfig();
    return;
  }

  if (command == '?')
  {
    printHelp();
    return;
  }

  // =================================================
  // CHECK READINESS
  // =================================================

  if (!motionReady)
  {
    Serial.println("[ERROR] Motion not ready");
    return;
  }

  if (motionHasError())
  {
    Serial.println("[ERROR] Motion is in ERROR state");
    Serial.println("[INFO] Reset MCU to restart");
    return;
  }

  // =================================================
  // F: CONTINUOUS FORWARD
  // =================================================

  if (command == 'f')
  {
    if (!isBlank(argument))
    {
      Serial.println("[ERROR] Use: f");
      return;
    }

    Serial.println("[COMMAND] FORWARD CONTINUOUS");

    reportCommand(motionForward());
    return;
  }

  // =================================================
  // B: CONTINUOUS BACKWARD OR DISTANCE
  // =================================================

  if (command == 'b')
  {
    // No argument: continuous backward.
    if (isBlank(argument))
    {
      Serial.println(
          "[COMMAND] BACKWARD CONTINUOUS");

      reportCommand(motionBackward());
      return;
    }

    // With argument: backward by counts.
    int32_t counts = 0;

    if (!parseCounts(argument, counts) ||
        counts <= 0)
    {
      Serial.println(
          "[ERROR] Use: b OR b <positive_counts>");
      return;
    }

    Serial.print("[COMMAND] BACKWARD COUNTS=");
    Serial.println(counts);

    reportCommand(motionMoveCounts(-counts));
    return;
  }

  // =================================================
  // D: FORWARD DISTANCE
  // =================================================

  if (command == 'd')
  {
    int32_t counts = 0;

    if (!parseCounts(argument, counts) ||
        counts <= 0)
    {
      Serial.println(
          "[ERROR] Use: d <positive_counts>");
      return;
    }

    Serial.print("[COMMAND] FORWARD COUNTS=");
    Serial.println(counts);

    reportCommand(motionMoveCounts(counts));
    return;
  }

  // =================================================
  // L: TURN LEFT
  // =================================================

  if (command == 'l')
  {
    float angle = 0.0f;

    if (!parseNumber(argument, angle) ||
        angle <= 0.0f)
    {
      Serial.println(
          "[ERROR] Use: l <positive_angle>");
      return;
    }

    Serial.print("[COMMAND] TURN LEFT ");
    Serial.println(angle);

    reportCommand(motionTurn(angle));
    return;
  }

  // =================================================
  // R: TURN RIGHT
  // =================================================

  if (command == 'r')
  {
    float angle = 0.0f;

    if (!parseNumber(argument, angle) ||
        angle <= 0.0f)
    {
      Serial.println(
          "[ERROR] Use: r <positive_angle>");
      return;
    }

    Serial.print("[COMMAND] TURN RIGHT ");
    Serial.println(angle);

    reportCommand(motionTurn(-angle));
    return;
  }

  // =================================================
  // H: HEADING OFFSET
  // =================================================

  if (command == 'h')
  {
    float offset = 0.0f;

    if (!parseNumber(argument, offset))
    {
      Serial.println(
          "[ERROR] Use: h <degrees>");
      return;
    }

    Serial.print("[COMMAND] HEADING OFFSET=");
    Serial.println(offset);

    const bool accepted =
        motionSetHeadingOffset(offset);

    Serial.println(
        accepted
            ? "[COMMAND] ACCEPTED"
            : "[COMMAND] REJECTED");

    return;
  }

  Serial.println("[ERROR] Unknown command");
  printHelp();
}

// =====================================================
// NON-BLOCKING SERIAL INPUT
// =====================================================

void processSerial()
{
  while (Serial.available() > 0)
  {
    const char received =
        static_cast<char>(Serial.read());

    if (received == '\r' || received == '\n')
    {
      if (commandLength > 0)
      {
        commandBuffer[commandLength] = '\0';

        executeCommand(commandBuffer);

        commandLength = 0;
      }

      continue;
    }

    if (commandLength >=
        COMMAND_BUFFER_SIZE - 1)
    {
      commandLength = 0;

      Serial.println(
          "[ERROR] Command too long");

      continue;
    }

    commandBuffer[commandLength++] = received;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(SERIAL_BAUD);

  const uint32_t serialStart = millis();

  while (!Serial &&
         millis() - serialStart < 5000)
  {
    delay(10);
  }

  Serial.println();
  Serial.println("======================================");
  Serial.println(" MICROMOUSE FULL MOTION TEST");
  Serial.println(" STM32F411CEU6");
  Serial.println("======================================");

  Serial.println("[INIT] Starting Motion...");

  const bool accepted = motionInit();

  if (!accepted)
  {
    Serial.println(
        "[ERROR] Motion initialization failed");
  }

  printHelp();

  Serial.println();
  Serial.println("[SYSTEM] Waiting for IMU calibration");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  // =============================================
  // MOTION UPDATE
  // =============================================

  motionUpdate();

  // =============================================
  // SERIAL COMMANDS
  // =============================================

  processSerial();

  const uint32_t now = millis();

  const MotionStatus status =
      motionGetStatus();

  // =============================================
  // STATE CHANGE
  // =============================================

  if (status.state != previousState)
  {
    previousState = status.state;

    Serial.println();

    Serial.print("[STATE] ");
    Serial.println(stateName(status.state));

    if (status.state == MotionState::Idle &&
        !motionReady)
    {
      motionReady = true;

      Serial.println("[INIT] Motion READY");
      Serial.println("[INIT] IMU calibration PASS");
      Serial.println("[READY] Enter a command");
    }

    if (status.state == MotionState::Completed)
    {
      Serial.println();
      Serial.println("===== TARGET REACHED =====");

      printTelemetry();

      Serial.println(
          "[READY] Enter next command");
    }

    if (status.state == MotionState::Error)
    {
      Serial.println();
      Serial.println("===== MOTION ERROR =====");

      printTelemetry();

      errorPrinted = true;
    }
  }

  // =============================================
  // ERROR
  // =============================================

  if (motionHasError() && !errorPrinted)
  {
    errorPrinted = true;
    printTelemetry();
  }

  // =============================================
  // CONTINUOUS TELEMETRY
  // =============================================

  if (motionIsBusy() &&
      motionReady &&
      now - lastPrintMs >= PRINT_INTERVAL_MS)
  {
    lastPrintMs = now;
    printTelemetry();
  }

  // No automatic timeout.
  // No automatic stop.
}
