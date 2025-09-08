################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Drivers/Motor_520/%.o: ../Drivers/Motor_520/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TI_MSP/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/IMU660RB/Fusion" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/IMU660RB" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/LSM6DSV16X" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/VL53L0X" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/WIT" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/BNO08X_UART_RVC" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/Ultrasonic_GPIO" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/Ultrasonic_Capture" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/OLED_Hardware_I2C" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/OLED_Hardware_SPI" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/OLED_Software_I2C" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/OLED_Software_SPI" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/MPU6050" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Debug" -I"D:/TI_MSP/mspm0_sdk_2_05_01_00/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSP/mspm0_sdk_2_05_01_00/source" -I"C:/Users/24506/workspace_ccstheia/mspm0-modules/Drivers/MSPM0" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"Drivers/Motor_520/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


