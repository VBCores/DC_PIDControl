#include "main.h"
#include "tim.h"
#include "dac.h"
#include "fdcan.h"

#include <utils.h>
#include <encoders/dc_incremental/dc_incremental.hpp>
#include <motors/dc/dc.h>

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
            .gear_ratio = 179
        }
    },
    PIDConfig{
        .p_gain = 1,
        .i_gain = 1,
        .d_gain = 1,
        .integral_error_lim = 3,
        .tolerance = 0.05
    },
    1,
    1
);
volatile encoder_data value = 0;

#ifdef __cplusplus
extern "C" {
#endif
[[noreturn]] void main_cpp(void) {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim7);

    HAL_IMPORTANT(encoder.init())

    HAL_IMPORTANT(motor.init())
    HAL_IMPORTANT(motor.set_Ipeak(1))
    HAL_IMPORTANT(motor.start())

    while (true) {
        encoder.update_value();
        value = encoder.get_value();
    }
}
#ifdef __cplusplus
}
#endif
