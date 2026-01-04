################################################################################
# MRS Version: 2.3.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/DataTranscode/data_transcode.c 

C_DEPS += \
./User/DataTranscode/data_transcode.d 

OBJS += \
./User/DataTranscode/data_transcode.o 

DIR_OBJS += \
./User/DataTranscode/*.o \

DIR_DEPS += \
./User/DataTranscode/*.d \

DIR_EXPANDS += \
./User/DataTranscode/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/DataTranscode/%.o: ../User/DataTranscode/%.c
	@	riscv-wch-elf-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Debug" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Core" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/User" -I"c:/DATA/Projects/Amon_Link/Firmware/CH32V003F4P6/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

