/* Board and motor constants for the header-only TMC5160 library.
 * Copy this file together with tmc5160.h and edit it for another board.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define BENCH_NODE_ID 42U
#define BENCH_FEEDBACK_SUBJECT_ID 1042U
#define BENCH_FEEDBACK_PERIOD_MS 50U
#define BENCH_COMMAND_TIMEOUT_MS 3000U
#define BENCH_POSITION_SPEED_RAD_S 0.25
#define BENCH_ACCELERATION_RAD_S2 0.5

#include "main.h"
#include "spi.h"

#define ROBOT_MOTOR_NEMA14 14
#define ROBOT_MOTOR_NEMA17 17
#define ROBOT_MOTOR_NEMA23 23
#define ROBOT_MOTOR_NEMA34 34
#define ROBOT_MOTOR_TYPE ROBOT_MOTOR_NEMA34

/* SPI and GPIO. DRV_ENN is active low; SPI_MODE is high for SPI.
 * SD_MODE is low for the internal ramp generator.
 */
#define ROBOT_TMC_SPI_HANDLE       (&hspi1)
#define ROBOT_TMC_CS_PORT          GPIOA
#define ROBOT_TMC_CS_PIN           GPIO_PIN_4
#define ROBOT_TMC_ENABLE_PORT      DRV_EN_GPIO_Port
#define ROBOT_TMC_ENABLE_PIN       DRV_EN_Pin
#define ROBOT_TMC_SPI_MODE_PORT    SPI_MODE_GPIO_Port
#define ROBOT_TMC_SPI_MODE_PIN     SPI_MODE_Pin
#define ROBOT_TMC_SD_MODE_PORT     SD_MODE_GPIO_Port
#define ROBOT_TMC_SD_MODE_PIN      SD_MODE_Pin
#define ROBOT_TMC_STEP_PORT        GPIOA
#define ROBOT_TMC_STEP_PIN         GPIO_PIN_8
#define ROBOT_TMC_DIR_PORT         DIR_GPIO_Port
#define ROBOT_TMC_DIR_PIN          DIR_Pin

/* The original hardware uses the TMC internal 12 MHz clock. */
#define ROBOT_TMC_CLOCK_HZ         12000000U
#define ROBOT_MOTOR_FULL_STEPS     200U
#define ROBOT_MOTOR_MICROSTEPS     256U
#define ROBOT_MOTOR_REVERSE        0U

#if ROBOT_MOTOR_TYPE == ROBOT_MOTOR_NEMA14
#define ROBOT_MOTOR_GEAR_RATIO     19.203208
#elif ROBOT_MOTOR_TYPE == ROBOT_MOTOR_NEMA17 || ROBOT_MOTOR_TYPE == ROBOT_MOTOR_NEMA23
#define ROBOT_MOTOR_GEAR_RATIO     50.0
#elif ROBOT_MOTOR_TYPE == ROBOT_MOTOR_NEMA34
#define ROBOT_MOTOR_GEAR_RATIO     1.0
#else
#error Unsupported ROBOT_MOTOR_TYPE
#endif

/* Current values from the original example. Tune for the motor and RSENSE.
 * GLOBAL_SCALER=0 means full scale.
 */
#define ROBOT_TMC_IHOLD            0U
#define ROBOT_TMC_IRUN             0U
#define ROBOT_TMC_IHOLD_DELAY      0U
#define ROBOT_TMC_GLOBAL_SCALER    33U
#define ROBOT_TMC_POWERDOWN_DELAY  10U

/* Original startup profile. The motor API converts physical units using
 * CLOCK_HZ, motor steps, microsteps and gearing above.
 */
#define ROBOT_TMC_GCONF           0x00000004U
#define ROBOT_TMC_CHOPCONF        0x000000C3U
/* StealthChop carrier at each output: fCLK / 410 = ~29.3 kHz at 12 MHz.
 * Datasheet effective chopper frequency is twice the output carrier (~58.5 kHz).
 */
#define ROBOT_TMC_PWM_FREQ        3U
/* PWMCONF is write-only: retain reset defaults for all other fields. */
#define ROBOT_TMC_PWMCONF         ((0xC40C001EUL & ~(3UL << 16)) | (ROBOT_TMC_PWM_FREQ << 16))
#define ROBOT_TMC_PWM_THRESHOLD   200U
#define ROBOT_TMC_VSTART          10U
#define ROBOT_TMC_VSTOP           10U
#define ROBOT_TMC_VMAX_STEPS_S    1000000.0
#define ROBOT_TMC_V1_STEPS_S      500000.0
#define ROBOT_TMC_A1             28192U
#define ROBOT_TMC_AMAX            9096U
#define ROBOT_TMC_DMAX            9096U
#define ROBOT_TMC_D1             28192U

#endif /* ROBOT_CONFIG_H */
