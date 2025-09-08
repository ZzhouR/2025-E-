#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
#include "imu660rb.h"
#include "Motor520.h"
#include "Stepper.h"


uint8_t enable_group1_irq = 0;
int16_t encoder1_count;
int16_t encoder2_count;
int16_t Speed_L;
int16_t Speed_R;
uint8_t KeyNum;

void Interrupt_Init(void)
{
    if(enable_group1_irq)
    {
        NVIC_EnableIRQ(1);
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        #if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
        case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOA))
            {
                #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOA)
                case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
                    Read_Quad();
                    break;
                #endif

                #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOA)
                case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                    Read_LSM6DSV16X();
                    break;
                #endif

                #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
                case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                    Read_VL53L0X();
                    break;
                #endif

                #if (defined GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT) && (GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT == GPIOA)
                case GPIO_IMU660RB_PIN_IMU660RB_INT1_IIDX:
                    Read_IMU660RB();
                    break;
                #endif

                default:
                    break;
            }
        #endif
        // #if defined GPIO_KEY_GPIOA_INT_IIDX
        // case GPIO_KEY_GPIOA_INT_IIDX:
        //     switch (DL_GPIO_getPendingInterrupt(GPIOA)){
        //             case GPIO_KEY_KEY1_IIDX:
        //                 KeyNum = 1;
        //                 break;

        //             case GPIO_KEY_KEY2_IIDX:
        //                 KeyNum = 2;
        //                 break;

        //             case GPIO_KEY_KEY4_IIDX:
        //                 KeyNum = 4;
        //                 break;

        //         default:
        //             break;
        //     }
        // #endif


        #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOB))
            {
                #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOB)
                case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
                    Read_Quad();
                    break;
                #endif

                #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
                case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                    Read_LSM6DSV16X();
                    break;
                #endif

                #if defined(Encoder1_A1_IIDX)
                case Encoder1_A1_IIDX:
                    // 处理编码器1中断
                    if (DL_GPIO_readPins(GPIOB, Encoder1_B1_PIN)) {
                        encoder1_count--; // B=高电平 -> 反转
                    } else {
                        encoder1_count++; // B=低电平 -> 正转
                    }
                    DL_GPIO_clearInterruptStatus(GPIOB, Encoder1_A1_PIN);
                    break;
                #endif

                #if defined(Encoder2_A2_IIDX)
                case Encoder2_A2_IIDX:
                    // 处理编码器2中断
                    if (DL_GPIO_readPins(GPIOB, Encoder2_B2_PIN)) {
                        encoder2_count--;
                    } else {
                        encoder2_count++;
                    }
                    DL_GPIO_clearInterruptStatus(GPIOB, Encoder2_A2_PIN);
                    break;
                #endif

                // #if defined (GPIO_KEY_KEY3_IIDX)
                // case GPIO_KEY_KEY3_IIDX:
                //     KeyNum = 3;
                //     break;
                // #endif

                // #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
                // case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                //     Read_VL53L0X();
                //     break;
                // #endif

                // #if (defined GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT) && (GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT == GPIOB)
                // case GPIO_IMU660RB_PIN_IMU660RB_INT1_IIDX:
                //     Read_IMU660RB();
                //     break;
                // #endif

                default:
                    break;
            }
        #endif

        

        #if defined GPIO_MPU6050_INT_IIDX
            case GPIO_MPU6050_INT_IIDX:
                Read_Quad();
                break;
        #endif

        #if defined GPIO_LSM6DSV16X_INT_IIDX
            case GPIO_LSM6DSV16X_INT_IIDX:
                Read_LSM6DSV16X();
                break;
        #endif

        #if defined GPIO_VL53L0X_INT_IIDX
            case GPIO_VL53L0X_INT_IIDX:
                Read_VL53L0X();
                break;
        #endif

        #if defined GPIO_IMU660RB_INT_IIDX
            case GPIO_IMU660RB_INT_IIDX:
                Read_IMU660RB();
                break;
        #endif
    }
}

#define BASE_SPEED      20
void TIMER_0_INST_IRQHandler(void) {

}



volatile StepperControl stepper_L;
volatile StepperControl stepper_R;
// void Stepper_L_INST_IRQHandler(void){
//         if (DL_TimerG_getPendingInterrupt(Stepper_L_INST)) {
//         if(stepper_L.steps_remaining > 0) {
//             // 减少剩余步数
//             stepper_L.steps_remaining--;
            
//             // 计算当前步在总步数中的位置
//             uint32_t current_step = stepper_L.total_steps - stepper_L.steps_remaining;
            
//             // 加速阶段
//             if (current_step < stepper_L.decel_start && 
//                 stepper_L.current_speed < stepper_L.target_speed) {
//                 // 计算新速度: v = v0 + a * t (t = 1/v)
//                 uint32_t new_speed = stepper_L.current_speed + 
//                                     (stepper_L.acceleration * 1000) / 
//                                     (stepper_L.current_speed * 1000);
                
//                 // 限制最大速度
//                 if (new_speed > stepper_L.target_speed) 
//                     new_speed = stepper_L.target_speed;
                
//                 stepper_L.current_speed = new_speed;
//                 Stepper_SetSpeed_L(new_speed);
//             }
//             // 减速阶段
//             else if (current_step >= stepper_L.decel_start && 
//                      stepper_L.current_speed > stepper_L.min_speed) {
//                 // 计算新速度: v = v0 - a * t (t = 1/v)
//                 uint32_t new_speed = stepper_L.current_speed - 
//                                     (stepper_L.acceleration * 1000) / 
//                                     (stepper_L.current_speed * 1000);
                
//                 // 限制最小速度
//                 if (new_speed < stepper_L.min_speed) 
//                     new_speed = stepper_L.min_speed;
                
//                 stepper_L.current_speed = new_speed;
//                 Stepper_SetSpeed_L(new_speed);
//             }
            
//             // 检查是否完成
//             if(stepper_L.steps_remaining == 0) {
//                 DL_TimerG_stopCounter(Stepper_L_INST); // 运动完成
//             }
//         }
//         }
// }
// void Stepper_R_INST_IRQHandler(void){
//     if (DL_TimerG_getPendingInterrupt(Stepper_R_INST)) {
//         if(stepper_R.steps_remaining > 0) {
//             // 减少剩余步数
//             stepper_R.steps_remaining--;
            
//             // 计算当前步在总步数中的位置
//             uint32_t current_step = stepper_R.total_steps - stepper_R.steps_remaining;
            
//             // 加速阶段
//             if (current_step < stepper_L.decel_start && 
//                 stepper_R.current_speed < stepper_R.target_speed) {
//                 // 计算新速度: v = v0 + a * t (t = 1/v)
//                 uint32_t new_speed = stepper_R.current_speed + 
//                                     (stepper_R.acceleration * 1000) / 
//                                     (stepper_R.current_speed * 1000);
                
//                 // 限制最大速度
//                 if (new_speed > stepper_R.target_speed) 
//                     new_speed = stepper_R.target_speed;
                
//                 stepper_R.current_speed = new_speed;
//                 Stepper_SetSpeed_L(new_speed);
//             }
//             // 减速阶段
//             else if (current_step >= stepper_R.decel_start && 
//                      stepper_R.current_speed > stepper_R.min_speed) {
//                 // 计算新速度: v = v0 - a * t (t = 1/v)
//                 uint32_t new_speed = stepper_R.current_speed - 
//                                     (stepper_R.acceleration * 1000) / 
//                                     (stepper_R.current_speed * 1000);
                
//                 // 限制最小速度
//                 if (new_speed < stepper_R.min_speed) 
//                     new_speed = stepper_R.min_speed;
                
//                 stepper_R.current_speed = new_speed;
//                 Stepper_SetSpeed_L(new_speed);
//             }
            
//             // 检查是否完成
//             if(stepper_R.steps_remaining == 0) {
//                 DL_TimerG_stopCounter(Stepper_R_INST); // 运动完成
//             }
//         }
//     }
// }
