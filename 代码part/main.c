/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "main.h"
#include <stdlib.h> // 添加abs函数需要的头文件
#include <math.h>  // 包含 fabs 函数



uint8_t OLED_MPU6050[32];
uint8_t OLED[32];
float Yaw, Roll, Pitch;
int16_t Speed_R,Speed_L;
int16_t count;
int left_speed,right_speed;
float Currurt_Yaw, Currurt_Roll, Currurt_Pitch;
uint16_t Case;              //状态
uint8_t KeyNum;             //按键情况
int16_t Serial_RxPacket[100];				
uint8_t Serial_RxFlag;
int16_t GetMessage_X1 = 170,
        GetMessage_Y1 = 128,
        GetMessage_X2 = 160+15,
        GetMessage_Y2 = 120-5,
        GetMessage_IF = 0;
#define UART0_PACKET_SIZE (5)   
uint8_t RxPacket[UART0_PACKET_SIZE];
#define BASE_SPEED      20
volatile uint8_t pen_state = 0;   // 落笔状态机: 0-空闲 1-等待稳定 2-执行落笔
volatile uint16_t stable_count = 0; // 连续稳定计数
uint8_t enable_aiming = 0;  // 瞄准控制标志：0-禁用 1-启用
uint8_t FinishLine = 0;
int8_t COUNT = 0;
int8_t a;


/*=====================================*/
/*
 * 跑直线代码，根据陀螺仪Yaw值 
*/
float Err_Sum = 0;
float Err, Err_Dif, Last_Err, PID;
#define Kp  2
#define Kd  0
void Run_Line(void){
    // Pitch = pitch - Currurt_Pitch;
    // Roll = roll - Currurt_Roll;
    //p i d
    int16_t measure = Roll;
    int16_t calcu = 0;
	Err = measure - calcu;
	Err_Dif = Err - Last_Err;
	Last_Err = Err;
	PID =  Kp * Err + Kd * Err_Dif;
    Set_MotorSpeed_L(BASE_SPEED + PID + 2);  // 左电机速度
    Set_MotorSpeed_R(BASE_SPEED - PID);  // 右电机速度
}
/*=====================================*/

/*=====================================*/

/*=====================================*/
/*pid循迹*/
typedef struct {
    bool L2;
    bool L1;
    bool M;
    bool R1;
    bool R2;
} SensorState;

// PID控制参数
#define KP 20.0f    // 比例系数 (需调试)
#define KI 0.0f     // 积分系数 (需调试)
#define KD 15.0f     // 微分系数 (需调试)

// 全局变量
int16_t Speed_Left = BASE_SPEED;
int16_t Speed_Right = BASE_SPEED;
static float integral = 0;       // 积分项
static float last_error = 0;      // 上次误差

void Follow_Line(void) {
    if (FinishLine <= 0) {
        Set_MotorSpeed_L(1);
        Set_MotorSpeed_R(1);
    }else{
    // 1. 读取传感器状态
    SensorState s = {
        .L2 = DL_GPIO_readPins(GPIOB, GPIO_Follow_L_2_PIN) != 0,
        .L1 = DL_GPIO_readPins(GPIOB, GPIO_Follow_L_1_PIN) != 0,
        .M  = DL_GPIO_readPins(GPIOA, GPIO_Follow_MID_PIN) != 0,
        .R1 = DL_GPIO_readPins(GPIOA, GPIO_Follow_R_1_PIN) != 0,
        .R2 = DL_GPIO_readPins(GPIOB, GPIO_Follow_R_2_PIN) != 0
    };

    // 2. 特殊场景处理：左直角弯
    if (s.L2 && s.L1) {
        // 左直角弯处理
        delay_cycles(2000000);
        Set_MotorSpeed_L(1);
        Set_MotorSpeed_R(1);
        delay_cycles(20000000);

        s.L1 = DL_GPIO_readPins(GPIOB, GPIO_Follow_L_1_PIN) != 0;
        // 递减控制参数
        int start_speed = 40;
        const int end_speed = 25;
        const int total_steps = 20; 
        const int step_cycles = 1500000; 
        // if(KeyNum == 4){
        //     Stepper_RunX(0,1);
        //     Stepper_SetDir_X(0);
        //     Stepper_SetSpeed_X(30);
        //     Stepper_RunSteps_X(90000);
        // }
        Set_MotorSpeed_R(60);
        Set_MotorSpeed_L(-10);
        delay_cycles(8000000);
        // 速度递减执行
        while (!s.L1) {
            //  if(start_speed > end_speed){
            //     Set_MotorSpeed_R(start_speed--);
            //     delay_cycles(10000000);
            //  }else {
            //     Set_MotorSpeed_R(35);
            //  }
            s.L1 = DL_GPIO_readPins(GPIOB, GPIO_Follow_L_1_PIN) != 0;
            Set_MotorSpeed_L(-10);
            Set_MotorSpeed_R(30);
            // Set_MotorSpeed_L(-15);
            s.L1 = DL_GPIO_readPins(GPIOB, GPIO_Follow_L_1_PIN) != 0;
        }
        FinishLine--;
        
        

        Set_MotorSpeed_L(1);
        Set_MotorSpeed_R(1);
        delay_cycles(20000000);
        
        // if(((FinishLine == 3)||(FinishLine == 7))&&(KeyNum == 4)){
        //     Stepper_SetDir_X(0);
        //     Stepper_SetSpeed_X(20);
        //     Stepper_RunSteps_X(1000);
        // }else if (((FinishLine == 2)||(FinishLine == 6))&&(KeyNum == 4)) {
        //     Stepper_SetDir_X(0);
        //     Stepper_SetSpeed_X(20);
        //     Stepper_RunSteps_X(2000);
        // }else if (((FinishLine == 1)||(FinishLine == 5))&&(KeyNum == 4)) {
        //     Stepper_SetDir_X(0);
        //     Stepper_SetSpeed_X(20);
        //     Stepper_RunSteps_X(2000);
        // }else if (((FinishLine == 8))&&(KeyNum == 4)) {
        //     Stepper_SetDir_X(1);
        //     Stepper_SetSpeed_X(20);
        //     Stepper_RunSteps_X(5000);
        // }
        // FinishLine++;
    }

    // 3. 计算传感器偏差值（-2.0到+2.0范围）
    float error = 0;
    uint8_t active_sensors = 0;
    
    if (s.L2) { error -= 2.0f; active_sensors++; }
    if (s.L1) { error -= 1.0f; active_sensors++; }
    if (s.M)  { error += 0.0f; active_sensors++; }
    if (s.R1) { error += 1.0f; active_sensors++; }
    if (s.R2) { error += 2.0f; active_sensors++; }

    // 4. 处理无传感器检测的情况
    if (active_sensors == 0) {
        // 使用上次误差的衰减值
        error = last_error * 0.7f;
    } else {
        error /= active_sensors;  // 计算加权平均值
    }

    // 5. PID计算
    integral += error;  // 积分项累加
    float derivative = error - last_error;  // 微分项计算
    float output = KP * error + KI * integral + KD * derivative;
    last_error = error;  // 更新误差记录

    // 6. 应用PID输出到电机
    Speed_Left = BASE_SPEED + (int16_t)output;
    Speed_Right = BASE_SPEED - (int16_t)output;
    
    // 7. 电机速度限幅
    #define MAX_DELTA 35  // 最大速度差
    if (Speed_Left - BASE_SPEED > MAX_DELTA) Speed_Left = BASE_SPEED + MAX_DELTA;
    if (BASE_SPEED - Speed_Left > MAX_DELTA) Speed_Left = BASE_SPEED - MAX_DELTA;
    if (Speed_Right - BASE_SPEED > MAX_DELTA) Speed_Right = BASE_SPEED + MAX_DELTA;
    if (BASE_SPEED - Speed_Right > MAX_DELTA) Speed_Right = BASE_SPEED - MAX_DELTA;
    if(Speed_Left == 0){
        Speed_Left = 1;
    }else if (Speed_Right == 0) {
        Speed_Right = 1;
    }
    // 8. 设置电机速度
    Set_MotorSpeed_L(Speed_Left);
    Set_MotorSpeed_R(Speed_Right);

    // 9. 调试信息显示
    OLED_ShowNum(1, 4, s.M, 3, 16);
    OLED_ShowNum(5*8, 2, (int)(error * 10), 3, 16);  // 显示放大10倍的误差
    OLED_ShowNum(5*8, 4, (int)output, 3, 16);        // 显示PID输出
    OLED_ShowNum(5*8, 6, Speed_Left, 3, 16);         // 显示左轮速度
}
}
/*=====================================================================*/








/*=====================================*/
/*
 * 获得按键输入函数
*/
void GetKeyNum(void){
    if (!DL_GPIO_readPins(GPIO_KEY_KEY1_PORT, GPIO_KEY_KEY1_PIN)){
        KeyNum = 1;
    }else if (!DL_GPIO_readPins(GPIO_KEY_KEY2_PORT, GPIO_KEY_KEY2_PIN)) {
        KeyNum = 2;
    }else if (!DL_GPIO_readPins(GPIO_KEY_KEY3_PORT, GPIO_KEY_KEY3_PIN)) {
        KeyNum = 3;
    }else if (!DL_GPIO_readPins(GPIO_KEY_KEY4_PORT, GPIO_KEY_KEY4_PIN)) {
        KeyNum = 4;
    }    
}
/*=====================================*/
void sendString(const char *str) {
  while (*str != '\0') {
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)*str);
    str++;
  }
}
/*=====================================*/
// PID控制参数 - 需要根据实际系统调整
float KP_X = 1.1f;    // X轴比例系数
float KI_X = 0.0001f;    // X轴积分系数 (新增)0.002f
float KD_X = 25.0f;     // X轴微分系数0.1f
float KP_Y = 0.9f;     // Y轴比例系数
float KI_Y = 0.0f;    // Y轴积分系数 (新增)
float KD_Y = 15.0f;     // Y轴微分系数
int16_t DEADZONE = 3;     // 死区阈值(避免微小震荡)
#define MAX_INTEGRAL 50.0f  // 积分限幅值 (新增)

// 全局变量保存PID状态
static int16_t last_error_x = 0;
static int16_t last_error_y = 0;
static float integral_x = 0.0f;  // X轴积分项 (新增)
static float integral_y = 0.0f;  // Y轴积分项 (新增)
float control_x = 0;
float control_y = 0;
int16_t error_x = 0;
int16_t error_y = 0;
volatile uint8_t searching = 0;  // 0-未搜索 1-搜索中

void Stepper_PD_Control(int8_t po)
{
    switch (po) {
        case 1:DEADZONE = 3;
        case 2:DEADZONE = 5;
    }
    if(GetMessage_IF){
        if(searching) {
            searching = 2;  // 退出搜索状态
            // 停止云台运动
            Stepper_RunX(0, 0);
            Stepper_RunY(0, 1);
        }
    // 计算X轴误差
    error_x = GetMessage_X2 - GetMessage_X1;
    // 计算Y轴误差
    error_y = GetMessage_Y2 - GetMessage_Y1;
    // ======== 积分项处理 ========
    // X轴积分项 - 仅在死区外累积
    if(abs(error_x) > DEADZONE) {  // 使用 fabs 而不是 fabsf
        integral_x += error_x;
        // 积分限幅防止饱和
        if(integral_x > MAX_INTEGRAL) integral_x = MAX_INTEGRAL;
        if(integral_x < -MAX_INTEGRAL) integral_x = -MAX_INTEGRAL;
    } else {
        integral_x *= 0.8f; // 死区内逐渐衰减积分项
    }
    
    // Y轴积分项 - 仅在死区外累积
    if(abs(error_y)  > DEADZONE) {  // 使用 fabs 而不是 fabsf
        integral_y += error_y;
        // 积分限幅防止饱和
        if(integral_y > MAX_INTEGRAL) integral_y = MAX_INTEGRAL;
        if(integral_y < -MAX_INTEGRAL) integral_y = -MAX_INTEGRAL;
    } else {
        integral_y *= 0.9f; // 死区内逐渐衰减积分项
    }
    
    // ================================
    
    // X轴PID计算
    int16_t derivative_x = error_x - last_error_x;
    control_x = KP_X * error_x + KI_X * integral_x + KD_X * derivative_x;
    
    // Y轴PID计算
    int16_t derivative_y = error_y - last_error_y;
    control_y = KP_Y * error_y + KI_Y * integral_y + KD_Y * derivative_y;
    
    // 更新历史误差
    last_error_x = error_x;
    last_error_y = error_y;
    
    // X轴电机控制
    // 修改Y轴电机控制部分：
    if(abs(error_y) < DEADZONE) {
        Stepper_RunY(0, 0);
    } else {
        uint8_t dir_y = (control_y > 0) ? 1 : 0;
        float abs_control = fabs(control_y);
        
        // // 确保最小输出能驱动电机
        // if(15.0f < abs_control < 20.0f) {
        //     abs_control = 20.0f;  // 设置最小驱动值
        // }
        int32_t speed_y;
        if(abs_control > 30){
            speed_y = 30;
        }else {
            speed_y = (int32_t)(abs_control);
        }
        Stepper_RunY(speed_y, dir_y);
        sprintf((char*)OLED, "Y:%d",speed_y);
        OLED_ShowString(40, 2, OLED, 16);
    }
    
    // X轴电机控制
    if(abs(error_x) < DEADZONE) {
        Stepper_RunX(0, 0);
    } else {
        uint8_t dir_x = (control_x > 0) ? 0 : 1;
        float abs_control = fabs(control_x);
        
        // // 确保最小输出能驱动电机
        // if(10.0f < abs_control < 20.0f) {
        //     abs_control = 20.0f;  // 设置最小驱动值
        // }
        int32_t speed_x;
        if(abs_control > 30){
            speed_x = 30;
        }else {
            speed_x = (int32_t)(abs_control);
        }
        Stepper_RunX(speed_x, dir_x);
        sprintf((char*)OLED, "X:%d", speed_x);
        OLED_ShowString(0, 2, OLED, 16);
    }
    }
    // else if(){
    //     // 如果尚未开始搜索，则启动搜索
    //     if(!searching) {
    //         searching = 1;  // 进入搜索状态
    //         Stepper_RunX(20, 0);  // 开始向右移动
    //     }
    //      // 重置PID参数，防止累积误差
    //     integral_x = 0;
    //     integral_y = 0;
    //     last_error_x = 0;
    //     last_error_y = 0;
    // }
    
    
}
/*=====================================*/




int main(void)
{

    SYSCFG_DL_init();
    SysTick_Init();

    MPU6050_Init();
    OLED_Init();
    Stepper_Init();
    // Ultrasonic_Init();
    // BNO08X_Init();
    // WIT_Init();
    // VL53L0X_Init();
    // LSM6DSV16X_Init();
    // IMU660RB_Init();
    sprintf((char *)OLED, "%d", 1);
    OLED_ShowString(5*8,0,OLED,16);
    /* Don't remove this! */
    Interrupt_Init();

    /* MPU6050校准 */
    /*=====================================*/
    // int i;
    // float Temp_Yaw = 90;
    // while(Temp_Yaw - yaw != 0){
    //     delay_cycles(80000000*3);
    //     sprintf((char *)OLED_MPU6050, "%-6.1f", Temp_Yaw - yaw);
    //     OLED_ShowString(5*8,4,OLED_MPU6050,16); 
    //     Temp_Yaw =  yaw;
    // }
    // Currurt_Roll = roll;
    // Currurt_Yaw = yaw;
    // Currurt_Pitch = pitch;
    // for(i=1;i<=120;i++){
    //     OLED_ShowNum(1, 2, i, 3, 16);
    //     delay_cycles(8000000);
    //     Currurt_Roll = roll;
    //     Currurt_Yaw = yaw;
    //     Currurt_Pitch = pitch;
    // }
    OLED_Clear();
    /*==================================================*/
    
    // NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    // DL_TimerG_startCounter(TIMER_0_INST);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    DL_TimerA_startCounter(PWM_Motor_INST);

    // NVIC_EnableIRQ(Stepper_Timer_INST_INT_IRQN);
    // DL_TimerG_startCounter(Stepper_Timer_INST);

    // NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    // NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    NVIC_EnableIRQ(Stepper_X_INST_INT_IRQN); //使能中断
    NVIC_EnableIRQ(Stepper_Y_INST_INT_IRQN); //使能中断

    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&RxPacket[0]));
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, UART0_PACKET_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    DL_GPIO_clearPins(GPIO_PEN_PORT, GPIO_PEN_PIN_0_PIN);
    DL_GPIO_clearPins(GPIO_LED_LED1_PORT, GPIO_LED_LED1_PIN);
    while (1) 
    {
        // DL_GPIO_clearPins(GPIO_PEN_PORT, GPIO_PEN_PIN_0_PIN);
        // Run_Line();
        // Follow_Line();

        // Stepper_RunSteps_X(10);
        // delay_cycles(80000000);
        // Stepper_PD_Control();
        // Stepper_RunX(10,1);
        //delay_cycles(20000000);
        // Stepper_RunY(20,1);
        // Set_MotorSpeed_L(15);
        // Set_MotorSpeed_R(15);
        // DrawRectangle(5000,2500);
        // sendString("HelloWorld");
        // DL_UART_Main_transmitData(UART_0_INST, 2);
        // DL_UART_Main_transmitData(UART_0_INST, 1);
        // OLED_ShowNum(1,2,count,3,16);

        sprintf((char*)OLED, "X:%d Y:%d IF:%d", GetMessage_X1, GetMessage_Y1, GetMessage_IF);
        OLED_ShowString(0, 0, OLED, 16);
        

        // sprintf((char *)OLED_MPU6050, "%6.1f", control_x);
        // OLED_ShowString(15*6,6,OLED_MPU6050,8);
        GetKeyNum();
        sprintf((char *)OLED_MPU6050, "%d", KeyNum);
        OLED_ShowString(15*6,5,OLED_MPU6050,8);
        sprintf((char *)OLED_MPU6050, "%d", FinishLine);
        OLED_ShowString(15*6,3,OLED_MPU6050,8);
        sprintf((char *)OLED_MPU6050, "%d", COUNT);
        OLED_ShowString(15*6,7,OLED_MPU6050,8);

        
        
        // 按键处理：按下按键1时启用瞄准
        if (KeyNum == 1) {
            
            enable_aiming = 1;    // 启用瞄准
            pen_state = 1;         // 进入等待稳定状态
            stable_count = 0;      // 重置稳定计数器
            KeyNum = 5;            // 防止重复触发
            
            // 重置积分项和历史误差
            integral_x = 0;
            integral_y = 0;
            last_error_x = 0;
            last_error_y = 0;
            // Currurt_Pitch = pitch;
            // Currurt_Roll = roll + 3;
            // Currurt_Yaw = yaw;
            
        }
        
        // 只有当瞄准启用时才执行PID控制
        if (enable_aiming) {
            Stepper_PD_Control(1);
            
            // 状态机处理落笔流程
            switch (pen_state) {
                case 1: // 等待稳定状态
                    if (abs(error_x) < 8 && abs(error_y) < 8 && 
                        fabs(control_x) < 5.0f && fabs(control_y) < 5.0f) {
                        if (++stable_count >= 10) {
                            pen_state = 2;  // 满足条件，进入落笔状态
                        }
                    } else {
                        stable_count = 0;  // 不稳定则重置计数
                    }
                    break;
                    
                case 2: // 执行落笔动作
                    DL_GPIO_setPins(GPIO_PEN_PORT, GPIO_PEN_PIN_0_PIN);
                    delay_cycles(80000000);  // 落笔保持时间
                    DL_GPIO_clearPins(GPIO_PEN_PORT, GPIO_PEN_PIN_0_PIN);
                    pen_state = 0;  // 返回空闲状态
                    enable_aiming = 0; // 完成落笔后禁用瞄准
                    
                    // 重置积分项防止累积
                    integral_x = 0;
                    integral_y = 0;
                    break;
            }
        }
        else if(KeyNum == 2){
            integral = 0;      // 重置积分项
            last_error = 0;    // 重置上次误差
            delay_cycles(20000000);
            KeyNum = 6;
            DL_GPIO_setPins(GPIO_LED_LED1_PORT, GPIO_LED_LED1_PIN);
        }
        else if(KeyNum == 3){
            FinishLine += 4;
            delay_cycles(40000000);
            KeyNum = 7;
        }
        else if(KeyNum == 6){
            
            Follow_Line();
        }
        else if(KeyNum == 5){
            COUNT += 1;
            KeyNum = 0;
            delay_cycles(16000000);
        }
        if(KeyNum == 4){
            integral = 0;      // 重置积分项
            last_error = 0;    // 重置上次误差
            delay_cycles(20000000);
            // Follow_Line();
            Stepper_PD_Control(1);
            DL_GPIO_setPins(GPIO_PEN_PORT, GPIO_PEN_PIN_0_PIN);
        }

        

            // Currurt_Pitch = pitch;
            // Currurt_Roll = roll;
            // Currurt_Yaw = yaw;
        // }if(KeyNum == 1){
        //     KeyNum = 6;
        //     KP_X = KP_X + 0.01;
        //     // Currurt_Pitch = pitch;
        //     // Currurt_Roll = roll;
        //     // Currurt_Yaw = yaw;
        // }
        // Pitch = pitch - Currurt_Pitch;
        // sprintf((char *)OLED_MPU6050, "%-6.1f", Pitch);
        // OLED_ShowString(5*8,0,OLED_MPU6050,16);
        // Roll = roll - Currurt_Roll;
        // sprintf((char *)OLED_MPU6050, "%-6.1f", Roll);
        // OLED_ShowString(5*8,2,OLED_MPU6050,16);
        // Yaw = yaw - Currurt_Yaw;
        // sprintf((char *)OLED_MPU6050, "%-6.1f", Yaw);
        // OLED_ShowString(5*8,4,OLED_MPU6050,16);     
        // sprintf((char *)OLED_MPU6050, "%6d", accel[0]);
        // OLED_ShowString(15*6,0,OLED_MPU6050,8);
        // sprintf((char *)OLED_MPU6050, "%6d", accel[1]);
        // OLED_ShowString(15*6,1,OLED_MPU6050,8);
        // sprintf((char *)OLED_MPU6050, "%6d", accel[2]);
        // OLED_ShowString(15*6,2,OLED_MPU6050,8);     
        // sprintf((char *)OLED_MPU6050, "%6d", gyro[0]);
        // OLED_ShowString(15*6,5,OLED_MPU6050,8);
        // sprintf((char *)OLED_MPU6050, "%6d", gyro[1]);
        // OLED_ShowString(15*6,6,OLED_MPU6050,8);
        // sprintf((char *)OLED_MPU6050, "%6d", gyro[2]);
        // OLED_ShowString(15*6,7,OLED_MPU6050,8);
        // sprintf((char *)OLED, "%6d", KeyNum);
        // OLED_ShowString(1,6,OLED,16);
        // // if(KeyNum == 2){
        // //     KeyNum = 6;
        // //     Case = 1;
        // // }



    }
}


volatile uint8_t UART0_DMA_FINISH_FLAG = 0;  // DMA完成标志
void UART_0_INST_IRQHandler(void){
    static uint8_t DataCount = 0;  // 数据字节计数器
    static uint8_t RxState = 0;		//定义示当前状态机状态的静态变量
	static uint8_t pRxPacket = 0;	//定义表示当前接收数据位置的静态变量
    int16_t RxData;
    uint8_t err_buff[3];
    uint8_t u3_ch;

    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    // case DL_UART_MAIN_IIDX_DMA_DONE_RX:
    //     OLED_ShowNum(0,0,1,3,16);
    //     DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
        
    //     // 检查起始字节
    //     if(1) {
    //         // 解析数据包
    //         GetMessage_X1 = RxPacket[1];
    //         GetMessage_Y1 = RxPacket[2];
    //         GetMessage_X2 = RxPacket[3];
    //         GetMessage_Y2 = RxPacket[4];
    //     }
        
    //     // 设置完成标志（主循环中处理）
    //     UART0_DMA_FINISH_FLAG = 1;
        
    //     // 重新配置DMA
    //     DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
    //     DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&RxPacket[0]);
    //     DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, UART0_PACKET_SIZE);
    //     DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    //     break;
    





    // switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    //     case DL_UART_MAIN_IIDX_DMA_DONE_RX:
    //         // UART0_DMA_FINISH_FLAG = 1;
    //         DL_DMA_disableChannel(DMA,DMA_CH0_CHAN_ID);
    //         if(!DL_UART_isRXFIFOEmpty(UART_0_INST)){
    //             while(!DL_UART_isRXFIFOEmpty(UART_0_INST)){
    //                 u3_ch = DL_UART_receiveData(UART_0_INST);
    //                 SYSCFG_DL_DMA_init();
    //                 DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
    //                 DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&RxPacket[0]);
    //                 DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, UART0_PACKET_SIZE);
    //             }
    //         }
    //         else if(RxPacket[0] != 0xA3){
    //             SYSCFG_DL_DMA_init();
    //             DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
    //             DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&RxPacket[0]);
    //             DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, UART0_PACKET_SIZE);
    //         }
    //         DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    //         break;






        case DL_UART_MAIN_IIDX_RX:
            // OLED_ShowNum(8,0,2,3,16);
            RxData = DL_UART_receiveData(UART_0_INST); // 必须读取数据寄存器!!!
            switch (RxState) {
                case 0:
                    if(RxData == 0xA3){
                        RxState = 1;
                        DataCount = 0;
                        pRxPacket = 0;
                    }
                    break;
                case 1:
                    Serial_RxPacket[pRxPacket++] = RxData;
                    if (++DataCount >= 4)  // 已接收10字节数据
				         RxState = 2;       // 进入包尾检查
			        break;
			    
			    case 2: // 检查包尾1 (0xB3)
			        if (1)
				        RxState = 3;
			        else 
				        RxState = 0; // 包尾错误，重置状态机
			        break;
			    
			    case 3: // 检查包尾2 (0xC3)
			        RxState = 0;  // 无论是否成功都重置状态机
			        if (1)
			        {
                         //count++;
				    // 成功接收完整数据包
				        Serial_RxFlag = 1;
                        GetMessage_X1 = Serial_RxPacket[0];
                        GetMessage_Y1 = Serial_RxPacket[1];
                        GetMessage_IF = Serial_RxPacket[2];
                        // GetMessage_Y2 = Serial_RxPacket[4];
                        // GetMessage_5 = Serial_RxPacket[4];
                        // GetMessage_6 = Serial_RxPacket[5];
                        // GetMessage_7 = Serial_RxPacket[6];
                        // GetMessage_8 = Serial_RxPacket[7];
                        // GetMessage_9 = Serial_RxPacket[8];
                        // GetMessage_10 = Serial_RxPacket[9];
                    }   
                    break; 
            }
            break;
        default:
            break;
    
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
 }
}