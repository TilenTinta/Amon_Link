################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/NRF24L01/NRF24L01.c 

OBJS += \
./User/NRF24L01/NRF24L01.o 

C_DEPS += \
./User/NRF24L01/NRF24L01.d 


# Each subdirectory must supply rules for building sources it contributes
User/NRF24L01/%.o: ../User/NRF24L01/%.c
	@	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"C:\DATA\Projects\Amon_Link\CH32V003F4P6\Debug" -I"C:\DATA\Projects\Amon_Link\CH32V003F4P6\Core" -I"C:\DATA\Projects\Amon_Link\CH32V003F4P6\User" -I"C:\DATA\Projects\Amon_Link\CH32V003F4P6\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

