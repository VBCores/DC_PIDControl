#include "main.h"
#include "tim.h"
#include "dac.h"
#include "fdcan.h"

#include <utils.hpp>
#include <encoders/dc_incremental/dc_incremental.hpp>
#include <motors/dc/dc.h>

micros __attribute__((optimize("O0"))) micros_64() {
    return (micros)(millis_k * 1000u + __HAL_TIM_GetCounter(&htim7));
}

DCIncrementalEncoder encoder(&htim8, 48);
DCMotorController motor(
    DCDriverConfig{
        .nSLEEP_pin = Sleep_Pin,
        .nSLEEP_GPIOx = Sleep_GPIO_Port,
        .IN1_channel = TIM_CHANNEL_1,
        .IN2_channel = TIM_CHANNEL_2,
        .timer = &htim1,
        .min_pwm = 200,
        .max_pwm = 999,
        .dac_channel_ = DAC_CHANNEL_1,
        .dac = &hdac1,
        .Rsense = 0.01f,
        .common = {
            .gear_ratio = 189
        }
    },
    PIDConfig{
        .multiplier = 1.0f,
        .p_gain = 3.1f,
        .i_gain = 2.05f,
        .d_gain = 1.95f,
        .integral_error_lim = 4.15,
        .tolerance = 0.025f
    },
    1.0f,
    0.7f
);

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void main_cpp(void) {
    HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim2))
    HAL_IMPORTANT(HAL_TIM_Base_Start_IT(&htim7))

    HAL_IMPORTANT(encoder.init())

    HAL_IMPORTANT(motor.init())
    HAL_IMPORTANT(motor.set_Ipeak(10))
    HAL_IMPORTANT(motor.start())
    motor.set_target_speed(1.5);

    static micros last_motor_regulation_mk;
    while (true) {
        micros now = micros_64();

        // 100 HZ, micros counter
        EACH_N_MICROS(now, last_motor_regulation_mk, MICROS_0_01S, {
            const float dt = diff / (float)MICROS_S;
            motor.update_speed(dt);
            motor.regulate(dt);
        })

        motor.update_angle(encoder);
    }
}
#ifdef __cplusplus
}
#endif
