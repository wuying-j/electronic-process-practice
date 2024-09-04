#include "rotate.h"
#include "stm32f1xx_hal.h" // STM32F1???HAL?

// ?????
TIM_HandleTypeDef htim2;

#define KP 1.0
#define KI 0.0
#define KD 0.1

// ???PWM
void PWM_Init(void)
{
    // ?? TIM2 ??
    __HAL_RCC_TIM2_CLK_ENABLE();

    // ?? TIM2
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 719; // ??????
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999; // PWM??
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // ??????
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

    // ?? PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

// ??????
void SetMotorSpeed(uint16_t left_speed, uint16_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_speed);
}





void RotateToAngle(float target_angle)
{
    float current_angle = 0;
    float error, last_error = 0;
    float integral = 0, derivative;
    float left_speed, right_speed;
    float control_signal;
    uint32_t start_time, elapsed_time;

    // ???????
    left_speed = 0;
    right_speed = 0;

    // ????????
    SetMotorSpeed(left_speed, right_speed);

    start_time = HAL_GetTick(); // ??????

    while (fabs(current_angle - target_angle) > 1.0) // ??1°??
    {
        elapsed_time = HAL_GetTick() - start_time; // ???????

        // ??????(??????????)
        current_angle = EstimateCurrentAngle();

        // ????
        error = target_angle - current_angle;
        integral += error;
        derivative = error - last_error;

        // ??PID????
        control_signal = KP * error + KI * integral + KD * derivative;

        // ??????
        if (error > 0) {
            left_speed = 50 + control_signal;
            right_speed = -50 + control_signal;
        } else {
            left_speed = -50 - control_signal;
            right_speed = 50 - control_signal;
        }

        // ????????
        if (left_speed > 100) left_speed = 100;
        if (left_speed < -100) left_speed = -100;
        if (right_speed > 100) right_speed = 100;
        if (right_speed < -100) right_speed = -100;

        // ??????
        SetMotorSpeed(left_speed, right_speed);

        // ???????
        last_error = error;

        // ????(??????)
        HAL_Delay(50);
    }

    // ????
    SetMotorSpeed(0, 0);
}

// ??????(????????)
float EstimateCurrentAngle()
{
    static unsigned long last_time = 0;
    unsigned long current_time = HAL_GetTick();
    float elapsed_time = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // ????????????
    float angle_change = (right_speed - left_speed) / WHEEL_BASE * elapsed_time;
    return angle_change;
}

