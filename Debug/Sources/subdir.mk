################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Events.c \
../Sources/FFT_UT.c \
../Sources/FIFO.c \
../Sources/Flash.c \
../Sources/LEDs.c \
../Sources/PIT.c \
../Sources/UART.c \
../Sources/main.c \
../Sources/packet.c 

OBJS += \
./Sources/Events.o \
./Sources/FFT_UT.o \
./Sources/FIFO.o \
./Sources/Flash.o \
./Sources/LEDs.o \
./Sources/PIT.o \
./Sources/UART.o \
./Sources/main.o \
./Sources/packet.o 

C_DEPS += \
./Sources/Events.d \
./Sources/FFT_UT.d \
./Sources/FIFO.d \
./Sources/Flash.d \
./Sources/LEDs.d \
./Sources/PIT.d \
./Sources/UART.d \
./Sources/main.d \
./Sources/packet.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\13113117\Desktop\Project13113117\UTS_EmbeddedSoftware\Library" -I"C:/Users/13113117/Desktop/Project13113117/UTS_EmbeddedSoftware/Static_Code/IO_Map" -I"C:/Users/13113117/Desktop/Project13113117/UTS_EmbeddedSoftware/Sources" -I"C:/Users/13113117/Desktop/Project13113117/UTS_EmbeddedSoftware/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


