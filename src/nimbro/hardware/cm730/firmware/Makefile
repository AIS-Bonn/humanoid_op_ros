# Makefile for CM730 firmware
# Modified by zerom for WinARM 08/2010
# Modified by Philipp Allgeuer for the NimbRo-OP 08/2015

# The target device can be changed by calling 'make DEVICE=CM740' for example
DEVICE = CM730

M3_OPTS = -mcpu=cortex-m3 -mthumb
COMPILE_OPTS = $(M3_OPTS) -Wall -g -Os -fno-common -fno-strict-aliasing -DFORCE_$(DEVICE)
# Need in the line above: -fno-strict-aliasing OR -Wno-strict-aliasing
INCLUDE_DIRS = -I. -Istm32f10x_lib/inc -ICM730_HW/inc -ICM730_APP/inc
LIBRARY_DIRS = -Lstm32f10x_lib

TCHAIN_PREFIX=arm-none-eabi-

CC = $(TCHAIN_PREFIX)gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS) 

CXX = $(TCHAIN_PREFIX)g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS) 

AS = $(TCHAIN_PREFIX)gcc
ASFLAGS = $(COMPILE_OPTS) -c 

LD = $(TCHAIN_PREFIX)gcc
LDFLAGS = $(M3_OPTS) -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T stm32.ld 

OBJCP = $(TCHAIN_PREFIX)objcopy
OBJCPFLAGS_HEX = -O ihex
OBJCPFLAGS_BIN = -O binary

OBJDUMP = $(TCHAIN_PREFIX)objdump
OBJDUMPFLAGS = -h -S -C -D

AR = $(TCHAIN_PREFIX)ar
ARFLAGS = cr

MAIN_OUT = CM730
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_HEX = $(MAIN_OUT).hex
MAIN_OUT_BIN = $(MAIN_OUT).bin
MAIN_OUT_LSS = $(MAIN_OUT).lss

# all

all: $(MAIN_OUT_ELF) $(MAIN_OUT_HEX) $(MAIN_OUT_BIN) $(MAIN_OUT_LSS)
	@echo
	@echo Compiled the firmware for the device $(DEVICE)!

# main

MAIN_OBJS = \
 CM730_APP/src/stm32f10x_it.o \
 CM730_APP/src/main.o \
 CM730_APP/src/serial.o \
 CM730_APP/src/isr.o \
 CM730_APP/src/zigbee.o \
 CM730_APP/src/zgb_hal.o \
 CM730_APP/src/CM_DXL_COM.o \
 CM730_APP/src/compass.o \
 CM730_HW/src/button.o \
 CM730_HW/src/led.o \
 CM730_HW/src/system_init.o \
 CM730_HW/src/system_func.o \
 CM730_HW/src/adc.o \
 CM730_HW/src/usart.o \
 CM730_HW/src/sound.o \
 CM730_HW/src/gyro_acc.o

$(MAIN_OUT_ELF): $(MAIN_OBJS) stm32f10x_lib/libstm32.a
	$(LD) $(LDFLAGS) $(MAIN_OBJS) stm32f10x_lib/libstm32.a --output $@

$(MAIN_OUT_HEX): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS_HEX) $< $@

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS_BIN) $< $@

$(MAIN_OUT_LSS): $(MAIN_OUT_ELF)
	$(OBJDUMP) $(OBJDUMPFLAGS) $< > $@

# libstm32.a

LIBSTM32_OUT = stm32f10x_lib/libstm32.a

LIBSTM32_OBJS = \
 stm32f10x_lib/src/stm32f10x_adc.o \
 stm32f10x_lib/src/stm32f10x_bkp.o \
 stm32f10x_lib/src/stm32f10x_can.o \
 stm32f10x_lib/src/stm32f10x_dma.o \
 stm32f10x_lib/src/stm32f10x_exti.o \
 stm32f10x_lib/src/stm32f10x_flash.o \
 stm32f10x_lib/src/stm32f10x_gpio.o \
 stm32f10x_lib/src/stm32f10x_i2c.o \
 stm32f10x_lib/src/stm32f10x_iwdg.o \
 stm32f10x_lib/src/stm32f10x_lib.o \
 stm32f10x_lib/src/stm32f10x_nvic.o \
 stm32f10x_lib/src/stm32f10x_pwr.o \
 stm32f10x_lib/src/stm32f10x_rcc.o \
 stm32f10x_lib/src/stm32f10x_rtc.o \
 stm32f10x_lib/src/stm32f10x_spi.o \
 stm32f10x_lib/src/stm32f10x_systick.o \
 stm32f10x_lib/src/stm32f10x_tim.o \
 stm32f10x_lib/src/stm32f10x_tim1.o \
 stm32f10x_lib/src/stm32f10x_usart.o \
 stm32f10x_lib/src/stm32f10x_wwdg.o \
 stm32f10x_lib/src/cortexm3_macro.o \
 stm32f10x_lib/src/stm32f10x_vector.o

$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(LIBSTM32_OBJS): stm32f10x_conf.h

clean:
	-rm -f $(MAIN_OBJS) $(LIBSTM32_OBJS) $(LIBSTM32_OUT) $(MAIN_OUT_ELF) $(MAIN_OUT_HEX) $(MAIN_OUT_BIN) $(MAIN_OUT_LSS) $(MAIN_OUT_ELF).map

cleanbuild:
	-rm -f $(MAIN_OBJS) $(LIBSTM32_OBJS) $(LIBSTM32_OUT) $(MAIN_OUT_BIN) $(MAIN_OUT_LSS) $(MAIN_OUT_ELF).map
# EOF
