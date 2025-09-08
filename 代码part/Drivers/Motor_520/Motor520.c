#include "ti_msp_dl_config.h"
#include "Motor520.h"

/*
 * 减速电机转速设置：占空比
*/

int32_t PWM_Period_Count = 3200;
/*
 * 函数：设置占空比.
 * 输入：占空比百分比0~100%.
 * 输出：转换后的对应Counter_Compare_Value值.
 * 注意：PWM_Period_Count设置为对应的值.
 */
void Set_Duty(int16_t duty){
    uint32_t CompareValue;
    CompareValue = PWM_Period_Count * (100 - duty) / 100;
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_0_INDEX);
}

/*
 * 函数：设置频率.
 * 输入：期望的PWM频率.
 * 输出：转换后的对应PWM_Period_Count值.频率 = 主频 / PWM_Period_Count
 */
void Set_Freq(uint32_t freq){
    PWM_Period_Count = PWM_Motor_INST_CLK_FREQ / freq;
    DL_Timer_setLoadValue(PWM_Motor_INST, PWM_Period_Count);
}

void Set_MotorSpeed_R(int32_t Speed_L){
    uint32_t CompareValue;
    if(Speed_L > 0){
        DL_GPIO_setPins(GPIO_Motor_PORT, GPIO_Motor_AIN2_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT, GPIO_Motor_AIN1_PIN);
        CompareValue = PWM_Period_Count * (100 - Speed_L) / 100;
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_0_INDEX);
    }else{
        DL_GPIO_setPins(GPIO_Motor_PORT, GPIO_Motor_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT, GPIO_Motor_AIN2_PIN);
        CompareValue = PWM_Period_Count * (100 + Speed_L) / 100;
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_0_INDEX);
    }
}
void Set_MotorSpeed_L(int32_t Speed_R){
    uint32_t CompareValue;
    if(Speed_R > 0){
        DL_GPIO_setPins(GPIO_Motor_PORT, GPIO_Motor_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT, GPIO_Motor_BIN1_PIN);
        CompareValue = PWM_Period_Count * (100 - Speed_R) / 100;
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_1_INDEX);
    }else{
        DL_GPIO_setPins(GPIO_Motor_PORT, GPIO_Motor_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT, GPIO_Motor_BIN2_PIN);
        CompareValue = PWM_Period_Count * (100 + Speed_R) / 100;
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_1_INDEX);
}
}


