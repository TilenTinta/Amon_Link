################################################################################
# MRS Version: 2.2.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/NRF24L01/NRF24L01.c 

C_DEPS += \
./User/NRF24L01/NRF24L01.d 

OBJS += \
./User/NRF24L01/NRF24L01.o 


EXPANDS += \
./User/NRF24L01/NRF24L01.c.253r.expand 



# Each subdirectory must supply rules for building sources it contributes
User/NRF24L01/%.o: ../User/NRF24L01/%.c
	@	riscv-wch-elf-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Debug" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Core" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/User" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

