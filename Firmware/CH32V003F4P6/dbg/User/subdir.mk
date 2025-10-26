################################################################################
# MRS Version: 2.2.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/bootloader.c \
../User/ch32v00x_it.c \
../User/main.c \
../User/system_ch32v00x.c 

C_DEPS += \
./User/bootloader.d \
./User/ch32v00x_it.d \
./User/main.d \
./User/system_ch32v00x.d 

OBJS += \
./User/bootloader.o \
./User/ch32v00x_it.o \
./User/main.o \
./User/system_ch32v00x.o 


EXPANDS += \
./User/bootloader.c.234r.expand \
./User/ch32v00x_it.c.234r.expand \
./User/main.c.234r.expand \
./User/system_ch32v00x.c.234r.expand 



# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Debug" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Core" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/User" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

