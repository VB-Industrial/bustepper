#include "bench.h"
#include "bench_units.hpp"
#include "fdcan.h"
#include "robot_config.h"
// Preserve the supplied header verbatim, but inhibit ENN during its init sequence.
// The reference header enables the power stage before finishing initialization.
static bool allow_enable = false;
static bool armed = false;
static uint32_t spi_errors = 0;
static bool spi_fault = false;
static HAL_StatusTypeDef spi_status = HAL_OK;
static HAL_StatusTypeDef bench_tmc_spi(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx,
                                       uint16_t size, uint32_t timeout) {
    spi_status = HAL_SPI_TransmitReceive(spi, tx, rx, size, timeout > 20 ? 20 : timeout);
    if (spi_status != HAL_OK) {
        ++spi_errors;
        spi_fault = true;
        allow_enable = false;
        armed = false;
        HAL_GPIO_WritePin(ROBOT_TMC_ENABLE_PORT, ROBOT_TMC_ENABLE_PIN, GPIO_PIN_SET);
    }
    return spi_status;
}
static void bench_tmc_gpio(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    if (port == ROBOT_TMC_ENABLE_PORT && pin == ROBOT_TMC_ENABLE_PIN && state == GPIO_PIN_RESET &&
        !allow_enable)
        state = GPIO_PIN_SET;
    HAL_GPIO_WritePin(port, pin, state);
}
#define HAL_SPI_TransmitReceive bench_tmc_spi
#define HAL_GPIO_WritePin bench_tmc_gpio
#include "tmc5160.h"
#undef HAL_GPIO_WritePin
#undef HAL_SPI_TransmitReceive
#include "cyphal/allocators/sys/sys_allocator.h"
#include "cyphal/cyphal.h"
#include "cyphal/node/node_info_handler.h"
#include "cyphal/node/registers_handler.hpp"
#include "cyphal/providers/G4CAN.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include <climits>
#include <cmath>
#include <memory>
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(Feedback, reg_udral_physics_kinematics_rotation_Planar_0_1)
using Value = uavcan_register_Value_1_0;
using Response = RegisterAccessResponse::Type;
static std::shared_ptr<CyphalInterface> bus;
alignas(std::max_align_t) static std::byte
    buffer[sizeof(CyphalInterface) + sizeof(G4CAN) + sizeof(SystemAllocator)];
static uint64_t micros() {
    static uint32_t previous = 0;
    static uint64_t elapsed = 0;
    uint32_t now = HAL_GetTick();
    elapsed += uint32_t(now - previous);
    previous = now;
    return elapsed * 1000;
}
static UtilityConfig utilities(micros, [] {
    tmc5160_disarm();
    Error_Handler();
});
static int mode = 0, error = 0; // 0 idle/hold, 1 velocity, 2 position; last rejected write reason
static int ihold = ROBOT_TMC_IHOLD, irun = ROBOT_TMC_IRUN, delay = ROBOT_TMC_IHOLD_DELAY,
           scaler = ROBOT_TMC_GLOBAL_SCALER;
static double gear = ROBOT_MOTOR_GEAR_RATIO, speed = BENCH_POSITION_SPEED_RAD_S,
              acceleration = BENCH_ACCELERATION_RAD_S2;
static double velocity_command = 0;
static uint32_t last_command = 0, timeout_ms = BENCH_COMMAND_TIMEOUT_MS;
static double scale() { return gear / ROBOT_MOTOR_GEAR_RATIO; }
static bool stationary() { return tmc5160_read_actual_velocity() == 0; }
static void current() { tmc5160_set_current(ihold, irun, delay); }
static void real(Value &out, double n) {
    out._tag_ = 13;
    out.real32.value.count = 1;
    out.real32.value.elements[0] = n;
}
static void integer(Value &out, int32_t n) {
    out._tag_ = 5;
    out.integer32.value.count = 1;
    out.integer32.value.elements[0] = n;
}
static bool number(const Value &in, double &n) {
    switch (in._tag_) {
    case 5:
        if (in.integer32.value.count == 1) {
            n = in.integer32.value.elements[0];
            return true;
        }
        break;
    case 6:
        if (in.integer16.value.count == 1) {
            n = in.integer16.value.elements[0];
            return true;
        }
        break;
    case 7:
        if (in.integer8.value.count == 1) {
            n = in.integer8.value.elements[0];
            return true;
        }
        break;
    case 8:
        if (in.natural64.value.count == 1) {
            n = in.natural64.value.elements[0];
            return std::isfinite(n);
        }
        break;
    case 9:
        if (in.natural32.value.count == 1) {
            n = in.natural32.value.elements[0];
            return true;
        }
        break;
    case 10:
        if (in.natural16.value.count == 1) {
            n = in.natural16.value.elements[0];
            return true;
        }
        break;
    case 11:
        if (in.natural8.value.count == 1) {
            n = in.natural8.value.elements[0];
            return true;
        }
        break;
    case 4:
        if (in.integer64.value.count == 1) {
            n = in.integer64.value.elements[0];
            return std::isfinite(n);
        }
        break;
    case 12:
        if (in.real64.value.count == 1) {
            n = in.real64.value.elements[0];
            return std::isfinite(n);
        }
        break;
    case 13:
        if (in.real32.value.count == 1) {
            n = in.real32.value.elements[0];
            return std::isfinite(n);
        }
        break;
    case 14:
        if (in.real16.value.count == 1) {
            n = in.real16.value.elements[0];
            return std::isfinite(n);
        }
        break;
    }
    return false;
}
static void stop() {
    tmc5160_stop();
    mode = 0;
    velocity_command = 0;
}
static bool motion_ready() {
    if (!armed) {
        error = 2;
        return false;
    }
    return true;
}
static double steps_per_rad(double ratio) {
    return bench_units::steps_per_radian(ratio, ROBOT_MOTOR_FULL_STEPS, ROBOT_MOTOR_MICROSTEPS);
}
static bool valid_speed(double n) {
    return bench_units::velocity_fits(n, steps_per_rad(gear), ROBOT_TMC_CLOCK_HZ);
}
static void position(double radians) {
    if (!bench_units::position_fits(radians, steps_per_rad(gear))) {
        error = 1;
        return;
    }
    if (!motion_ready())
        return;
    tmc5160_set_pos(radians * scale());
    tmc5160_set_velocity(speed * scale());
    mode = 2;
}
// All settings are volatile, like the bench commands; no EEPROM or calibration.
static RegisterDefinition setting(const char *name, double *value, double low, double high,
                                  bool stopped = false) {
    return {name, [=](const Value &in, Value &out, Response &r) {
                double n;
                r._mutable = true;
                if (in._tag_ != 0) {
                    error = 0;
                    if (!number(in, n) || n < low || n > high)
                        error = 1;
                    else if (stopped && (armed || !stationary()))
                        error = 3;
                    else if (!bench_units::acceleration_fits(
                                 value == &acceleration ? n : acceleration,
                                 steps_per_rad(value == &gear ? n : gear), ROBOT_TMC_CLOCK_HZ))
                        error = 1;
                    else if ((value == &speed || value == &gear) &&
                             !valid_speed(value == &speed ? n : speed * (n / gear)))
                        error = 1;
                    else {
                        *value = n;
                        tmc5160_set_acceleration(acceleration * scale());
                        if (mode == 2)
                            tmc5160_set_velocity(speed * scale());
                    }
                }
                real(out, *value);
            }};
}
static RegisterDefinition current_setting(const char *name, int *value, int low, int high) {
    return {name, [=](const Value &in, Value &out, Response &r) {
                double n;
                r._mutable = true;
                if (in._tag_ != 0) {
                    error = 0;
                    if (!number(in, n) || n < low || n > high || std::floor(n) != n ||
                        (value == &scaler && n != 0 && n < 32))
                        error = 1;
                    else {
                        *value = int(n);
                        if (value == &scaler)
                            tmc5160_set_global_scaler(scaler);
                        else
                            current();
                    }
                }
                integer(out, *value);
            }};
}
extern "C" void bench_init() {
    tmc5160_init();
    // The library remains unchanged; apply the bench's carrier-frequency setting.
    // PWMCONF is write-only. Never derive its other fields from an SPI read.
    tmc5160_set_pwm_config(ROBOT_TMC_PWMCONF);
    tmc5160_disarm();
    stop();
    current();
    tmc5160_set_acceleration(acceleration);
    FDCAN_FilterTypeDef filter{};
    auto f = canardMakeFilterForServices(BENCH_NODE_ID);
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = f.extended_can_id;
    filter.FilterID2 = f.extended_mask;
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT) != HAL_OK ||
        HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK ||
        HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();
    bus = std::shared_ptr<CyphalInterface>(CyphalInterface::create_bss<G4CAN, SystemAllocator>(
        buffer, BENCH_NODE_ID, &hfdcan1, 100, utilities));
    new NodeInfoReader(bus, "org.bustepper.bench", {1, 0}, {1, 0}, {0, 1}, 0);
    new RegistersHandler<20>(
        {{{"arm",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (!number(in, n) || (n != 0 && n != 1))
                       error = 1;
                   else {
                       stop();
                       if (n == 0) {
                           tmc5160_disarm();
                           allow_enable = false;
                           armed = false;
                       } else if (!stationary())
                           error = 3;
                       else if ((tmc5160_read_io_status() >> 24) != 0x30 || spi_status != HAL_OK)
                           error = 4;
                       else {
                           spi_fault = false;
                           current();
                           if (!spi_fault) {
                               allow_enable = true;
                               tmc5160_arm();
                               armed = !spi_fault;
                           }
                           if (spi_fault)
                               error = 4;
                       }
                   }
               }
               integer(out, armed);
           }},
          {"stop",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (number(in, n) && n == 1)
                       stop();
                   else
                       error = 1;
               }
               integer(out, mode == 0);
           }},
          {"move",
           [](const Value &in, Value &out, Response &r) { // microsteps per second
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (!number(in, n) ||
                       !valid_speed(n * 2 * 3.141592653589793 /
                                    (gear * ROBOT_MOTOR_FULL_STEPS * ROBOT_MOTOR_MICROSTEPS)))
                       error = 1;
                   else if (motion_ready()) {
                       velocity_command = n * 2 * 3.141592653589793 /
                                          (gear * ROBOT_MOTOR_FULL_STEPS * ROBOT_MOTOR_MICROSTEPS);
                       tmc5160_move(velocity_command * scale());
                       last_command = HAL_GetTick();
                       mode = 1;
                   }
               }
               real(out, velocity_command * gear * ROBOT_MOTOR_FULL_STEPS * ROBOT_MOTOR_MICROSTEPS /
                             (2 * 3.141592653589793));
           }},
          {"move_rad",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (!number(in, n) || !valid_speed(n))
                       error = 1;
                   else if (motion_ready()) {
                       velocity_command = n;
                       tmc5160_move(n * scale());
                       last_command = HAL_GetTick();
                       mode = 1;
                   }
               }
               real(out, velocity_command);
           }},
          {"pos_set",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (number(in, n))
                       position(n);
                   else
                       error = 1;
               }
               real(out, tmc5160_steps_to_radians(tmc5160_read_target_position()) / scale());
           }},
          {"tmc_pos_set",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (!number(in, n) || n < INT32_MIN || n > INT32_MAX || std::floor(n) != n)
                       error = 1;
                   else if (motion_ready()) {
                       tmc5160_set_ramp_mode(TMC5160_MODE_POSITION);
                       tmc5160_set_target_position(int32_t(n));
                       tmc5160_set_velocity(speed * scale());
                       mode = 2;
                   }
               }
               integer(out, tmc5160_read_target_position());
           }},
          {"pos_steps", [](const Value &, Value &out,
                           Response &) { integer(out, tmc5160_read_actual_position()); }},
          {"pos_get",
           [](const Value &, Value &out, Response &) { real(out, tmc5160_get_pos() / scale()); }},
          {"velocity", [](const Value &, Value &out,
                          Response &) { real(out, tmc5160_get_velocity() / scale()); }},
          {"zero_set",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (!number(in, n) || n != 1)
                       error = 1;
                   else if (!stationary())
                       error = 3;
                   else {
                       tmc5160_set_zero();
                       mode = 0;
                       velocity_command = 0;
                   }
               }
               integer(out, tmc5160_read_actual_position());
           }},
          setting("gear_ratio", &gear, 0.001, 100000, true),
          setting("vel_set", &speed, 0.000001, 100000),
          setting("acceleration", &acceleration, 0.000001, 100000),
          current_setting("ihold", &ihold, 0, 31),
          current_setting("irun", &irun, 0, 31),
          current_setting("ihold_delay", &delay, 0, 15),
          current_setting("global_scaler", &scaler, 0, 255),
          {"command_timeout_ms",
           [](const Value &in, Value &out, Response &r) {
               double n;
               r._mutable = true;
               if (in._tag_ != 0) {
                   error = 0;
                   if (!number(in, n) || n < 0 || n > 3600000 || floor(n) != n)
                       error = 1;
                   else
                       timeout_ms = uint32_t(n);
               }
               integer(out, timeout_ms);
           }},
          {"status",
           [](const Value &, Value &out, Response &) {
               out._tag_ = 5;
               out.integer32.value.count = 6;
               auto *v = out.integer32.value.elements;
               v[0] = armed;
               v[1] = mode;
               v[2] = error;
               v[3] = HAL_GetTick();
               v[4] = spi_errors;
               v[5] = spi_status;
           }},
          {"tmc_diag",
           [](const Value &, Value &out, Response &) {
               out._tag_ = 5;
               out.integer32.value.count = 10;
               auto *v = out.integer32.value.elements;
               const uint8_t regs[] = {TMC5160_GSTAT,      TMC5160_DRV_STATUS,    TMC5160_IOIN,
                                       TMC5160_IHOLD_IRUN, TMC5160_GLOBAL_SCALER, TMC5160_RAMPMODE,
                                       TMC5160_RAMP_STAT,  TMC5160_CHOPCONF, TMC5160_PWMCONF, TMC5160_GCONF};
               for (unsigned i = 0; i < 10; ++i)
                   v[i] = regs[i] == TMC5160_PWMCONF ? static_cast<int32_t>(ROBOT_TMC_PWMCONF)
                                                   : tmc5160_read_reg(regs[i]);
           }}}},
        bus);
}
extern "C" void bench_loop() {
    bus->loop();
    if (spi_fault) {
        error = 4;
        mode = 0;
        velocity_command = 0;
    }
    uint32_t now = HAL_GetTick();
    if (mode == 1 && timeout_ms && uint32_t(now - last_command) >= timeout_ms)
        stop();
    static uint32_t last_feedback = 0;
    if (uint32_t(now - last_feedback) >= BENCH_FEEDBACK_PERIOD_MS) {
        last_feedback = now;
        static CanardTransferID feedback_tid = 0;
        reg_udral_physics_kinematics_rotation_Planar_0_1 feedback{};
        feedback.angular_position.radian = tmc5160_get_pos() / scale();
        feedback.angular_velocity.radian_per_second = tmc5160_get_velocity() / scale();
        // No acceleration sensor/estimate; match the RUKA feedback convention.
        feedback.angular_acceleration.radian_per_second_per_second = 0;
        if (!spi_fault)
            bus->send_msg<Feedback>(&feedback, BENCH_FEEDBACK_SUBJECT_ID, &feedback_tid);
    }
    static uint32_t last_heartbeat = 0;
    if (uint32_t(now - last_heartbeat) >= 1000) {
        last_heartbeat = now;
        static CanardTransferID tid = 0;
        uavcan_node_Heartbeat_1_0 h{};
        h.uptime = now / 1000;
        h.mode.value = 0;
        bus->send_msg<HBeat>(&h, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &tid);
    }
}
