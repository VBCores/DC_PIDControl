#include "main.h"
#include "tim.h"
#include "dac.h"
#include "fdcan.h"

#include <utils.h>
#include <encoders/dc_incremental/dc_incremental.hpp>
#include <motors/dc/dc.h>

uint64_t micros_64() {
    return micros_k * 1000 + __HAL_TIM_GetCounter(&htim7);
}

DCIncrementalEncoder encoder(&htim8, 48);
DCMotorController motor(
    DCDriverConfig{
        .is_small = true,
        .nSLEEP_pin = Sleep_Pin,
        .nSLEEP_GPIOx = Sleep_GPIO_Port,
        .IN1_channel = TIM_CHANNEL_1,
        .IN2_channel = TIM_CHANNEL_2,
        .timer = &htim1,
        .max_pwm = 1000,
        .dac_channel_ = DAC_CHANNEL_1,
        .dac = &hdac1,
        .Rsense = 0.1,
        .common = {
            .gear_ratio = 189
        }
    },
    PIDConfig{
        .p_gain = 1,
        .i_gain = 1,
        .d_gain = 1,
        .integral_error_lim = 3,
        .tolerance = 0.05
    },
    0.5,
    0.5
);

#ifdef __cplusplus
extern "C" {
#endif

void motor_control() {
    static uint64_t now = 0;
    float dt = (micros_64() - now) / 1000000.0f;
    motor.regulate(encoder, dt);
    now = micros_64();
}

[[noreturn]] void main_cpp(void) {
    HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim2))
    HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim7))
    HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim3))

    HAL_IMPORTANT(encoder.init())

    HAL_IMPORTANT(motor.init())
    HAL_IMPORTANT(motor.set_Ipeak(1))
    HAL_IMPORTANT(motor.start())

    while (true) {

    }
}
#ifdef __cplusplus
}
#endif
