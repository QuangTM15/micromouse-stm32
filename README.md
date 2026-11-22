# Micromouse firmware

STM32F411CEU6 firmware migrated from `legacy/floodfill_final_luimanh_4x4.ino`. The legacy sketch remains the behavioral reference.

## Build and upload

Run `pio run -e micromouse` to build. Connect the ST-Link and run `pio run -e micromouse -t upload` when ready to flash. The monitor baud rate is 115200; the original source does not identify the physical Serial transport, so check the Arduino board configuration and wiring before use.

## Modules

- `config.h`: original constants and pin mappings.
- `motor`: motor control, braking, coasting, and turning PWM.
- `encoder`: interrupt handlers and atomic counter access.
- `tof`: ToF objects and shared I2C and sensor setup.
- `imu`: IMU initialization, calibration and gyro reading.
- `motion`: straight travel, turns and rear-wall alignment.
- `maze`: maze state, flood fill, wall updates and direction selection.
- `robot`: navigation state and loop coordination.
- `main`: setup and loop entry points.

The combined sensor setup calls `setupImu()` at the original point in the sequence. The original loop is called through `robotLoop()`. Function declarations and external global declarations were added for separate C++ translation units. No hardware validation is implied by a successful build.
