# STM32 MPU6050 IMU with FreeRTOS 

## Overview
This project demonstrates the complete bring-up of an MPU-6050 IMU on an 
STM32
Nucleo-G491RE board, progressing from bare-metal sensor access to a 
stable,
reset-safe FreeRTOS-based task architecture.

---

## Hardware Used
- STM32 Nucleo-G491RE
- MPU-6050 (3-axis accelerometer + 3-axis gyroscope)
- USB (power + UART logging)

---

## Software & Tools
- STM32CubeIDE
- STM32CubeMX
- STM32 HAL drivers
- FreeRTOS (via CMSIS-RTOS v2)
- UART terminal for logging

---

## Bare-Metal MPU-6050 Bring-up

### Objectives
- Verify I2C communication on real hardware
- Wake up the MPU-6050 from sleep
- Read accelerometer and gyroscope data
- Convert raw values into physical units
- Compute roll and pitch from accelerometer data

### Key Steps
- I2C bus scan confirmed MPU-6050 at address `0x68`
- WHO_AM_I register read successfully (`0x72`)
- Sensor wake-up via `PWR_MGMT_1`
- Accelerometer scaling to g units (±2g → 16384 LSB/g)
- Gyroscope scaling to dps (±250 dps → 131 LSB/dps)
- Roll and pitch calculated using gravity-based equations

### Outcome
- Stable accelerometer and gyroscope readings
- Correct roll and pitch behavior when physically tilting the board
- Reliable UART logging pipeline

---

## FreeRTOS Integration

### Objectives
- Re-architect the bare-metal design into a multitasking RTOS system
- Separate concerns using tasks
- Use RTOS queues for data flow
- Protect shared resources
- Achieve reset-stable behavior

---

### Task Architecture

IMU_Task
-> (Queue)
Orientation_Task
-> (Queue)
Logger_Task


### Task Responsibilities
| Task | Function |
|----|----|
| IMU_Task | Periodic I2C read of accelerometer & gyroscope |
| Orientation_Task | Roll & pitch computation |
| Logger_Task | UART logging (protected by mutex) |

---

### RTOS Concepts Applied
- FreeRTOS task creation and prioritization
- Queue-based inter-task communication
- UART mutex for thread-safe logging
- Increased FreeRTOS heap size
- `configASSERT()` to detect silent RTOS failures
- Reset-stable scheduler behavior

---

### Runtime Output Example

UART ALIVE
WHO_AM_I = 0x72
ROLL=-10.25 deg | PITCH=-0.16 deg
ROLL=-10.87 deg | PITCH=-1.40 deg


---

## Key Learnings
- Real I2C sensor bring-up on STM32
- Importance of waking sensors from sleep
- Raw-to-physical unit conversion
- Roll & pitch estimation fundamentals
- Proper RTOS task decomposition
- Why UART logging must be mutex-protected
- Why heap sizing matters in FreeRTOS
- How reset behavior exposes real RTOS bugs

---
 

This repository represents a **stable baseline** for further work on 
sensor
fusion, real-time behavior, and CAN-based communication.

---

## Next Steps
- Fixed-rate IMU sampling using `vTaskDelayUntil`
- Gyroscope integration and yaw drift analysis
- Complementary filter (accelerometer + gyroscope fusion)
- Optional CAN telemetry output

