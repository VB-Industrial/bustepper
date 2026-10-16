// Test the real, unmodified header and robot_config.h, replacing only Cube HAL.
#define __MAIN_H
#define __SPI_H__
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>
struct GPIO_TypeDef { unsigned id; };
struct SPI_HandleTypeDef {};
static GPIO_TypeDef gpio_a{0}, gpio_c{1};
#define GPIOA (&gpio_a)
#define GPIO_PIN_4 16U
#define GPIO_PIN_8 256U
#define DRV_EN_GPIO_Port (&gpio_c)
#define DRV_EN_Pin 32U
#define SPI_MODE_GPIO_Port GPIOA
#define SPI_MODE_Pin 2048U
#define SD_MODE_GPIO_Port GPIOA
#define SD_MODE_Pin 4096U
#define DIR_GPIO_Port GPIOA
#define DIR_Pin 512U
#define HAL_MAX_DELAY UINT32_MAX
static SPI_HandleTypeDef hspi1;
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
static bool cs_high = true;
static GPIO_PinState enable = GPIO_PIN_SET;
static unsigned cs_releases = 0;
static std::array<uint32_t, 128> registers{};
static std::vector<std::array<uint8_t, 5>> frames;
static uint32_t pending_response = 0;
static void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    if (port == GPIOA && pin == GPIO_PIN_4) {
        cs_high = state == GPIO_PIN_SET;
        if (cs_high) ++cs_releases;
    }
    if (port == DRV_EN_GPIO_Port && pin == DRV_EN_Pin) enable = state;
}
static void HAL_Delay(uint32_t) {}
static int HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t* tx, uint8_t* rx,
                                  uint16_t size, uint32_t) {
    assert(!cs_high && size == 5);
    frames.push_back({tx[0], tx[1], tx[2], tx[3], tx[4]});
    rx[0] = 0;
    for (unsigned i = 1; i < 5; ++i) rx[i] = uint8_t(pending_response >> (8 * (4-i)));
    unsigned address = tx[0] & 0x7f;
    if (tx[0] & 0x80) {
        registers[address] = uint32_t(tx[1]) << 24 | uint32_t(tx[2]) << 16 |
                             uint32_t(tx[3]) << 8 | tx[4];
    }
    pending_response = registers[address];
    return 0;
}
#include "tmc5160.h"
static bool near(double a, double b, double tolerance) { return std::abs(a-b) <= tolerance; }
int main() {
    // A read must clock the requested register twice, with CS released between frames.
    registers[TMC5160_IOIN] = 0x30123456;
    pending_response = 0xdeadbeef;
    assert(tmc5160_read_io_status() == 0x30123456);
    assert(frames.size() == 2 && cs_releases == 2 && cs_high);
    assert(frames[0][0] == TMC5160_IOIN && frames[1][0] == TMC5160_IOIN);

    tmc5160_set_target_position(-123456);
    assert(registers[TMC5160_XTARGET] == uint32_t(-123456));
    assert(frames.back()[0] == (0x80 | TMC5160_XTARGET));
    assert(tmc5160_read_target_position() == -123456);

    // Current updates must affect IHOLD_IRUN alone and pack all three fields.
    auto before = registers;
    tmc5160_set_current(2, 9, 4);
    assert(registers[TMC5160_IHOLD_IRUN] == 0x040902);
    for (unsigned i = 0; i < registers.size(); ++i)
        if (i != TMC5160_IHOLD_IRUN) assert(registers[i] == before[i]);
    tmc5160_set_current(7, 9, 4);
    assert(registers[TMC5160_IHOLD_IRUN] == 0x040907);
    tmc5160_set_current(7, 12, 4);
    assert(registers[TMC5160_IHOLD_IRUN] == 0x040c07);
    tmc5160_set_global_scaler(128);
    assert(registers[TMC5160_GLOBAL_SCALER] == 128);

    const double steps_rev = 200.0 * 256 * ROBOT_MOTOR_GEAR_RATIO;
    assert(tmc5160_radians_to_steps(TMC5160_TWO_PI) == int32_t(std::round(steps_rev)));
    assert(tmc5160_radians_to_steps(-TMC5160_TWO_PI) == -int32_t(std::round(steps_rev)));
    assert(near(tmc5160_steps_to_radians(123456), 123456 * TMC5160_TWO_PI / steps_rev, 1e-12));

    tmc5160_move(-0.1);
    assert(registers[TMC5160_RAMPMODE] == TMC5160_MODE_VELOCITY_NEGATIVE);
    const auto expected_v = uint32_t(std::round(0.1 * steps_rev / TMC5160_TWO_PI * 16777216 / 12000000));
    assert(registers[TMC5160_VMAX] == expected_v);
    tmc5160_move(0.1);
    assert(registers[TMC5160_RAMPMODE] == TMC5160_MODE_VELOCITY_POSITIVE);
    assert(registers[TMC5160_VMAX] == expected_v);
    registers[TMC5160_VACTUAL] = 0x00ffff9c; // signed 24-bit -100
    assert(tmc5160_read_actual_velocity() == -100);
    assert(near(tmc5160_get_velocity(), -100.0 * 12000000 / 16777216 * TMC5160_TWO_PI / steps_rev, 1e-12));

    tmc5160_set_acceleration(0.5);
    const auto expected_a = uint32_t(std::round(0.5 * steps_rev / TMC5160_TWO_PI * 2199023255552.0 / (12000000.0 * 12000000)));
    for (auto reg : {TMC5160_A1, TMC5160_AMAX, TMC5160_DMAX, TMC5160_D1})
        assert(registers[reg] == expected_a);
    tmc5160_set_pos(-0.25);
    assert(registers[TMC5160_RAMPMODE] == TMC5160_MODE_POSITION);
    assert(int32_t(registers[TMC5160_XTARGET]) == int32_t(std::round(-0.25 * steps_rev / TMC5160_TWO_PI)));
    registers[TMC5160_XACTUAL] = uint32_t(-123456);
    assert(near(tmc5160_get_pos(), -123456 * TMC5160_TWO_PI / steps_rev, 1e-12));
    tmc5160_stop();
    assert(registers[TMC5160_VMAX] == 0);
    tmc5160_set_zero();
    assert(registers[TMC5160_RAMPMODE] == TMC5160_MODE_HOLD);
    assert(registers[TMC5160_XACTUAL] == 0 && registers[TMC5160_XTARGET] == 0);
    tmc5160_arm(); assert(enable == GPIO_PIN_RESET);
    tmc5160_disarm(); assert(enable == GPIO_PIN_SET);
    tmc5160_init();
    assert(registers[TMC5160_IHOLD_IRUN] == (ROBOT_TMC_IHOLD_DELAY << 16 | ROBOT_TMC_IRUN << 8 | ROBOT_TMC_IHOLD));
    assert(registers[TMC5160_RAMPMODE] == TMC5160_MODE_HOLD);
}
