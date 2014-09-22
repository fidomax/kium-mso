#/*
#    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.
#	
#
#    ***************************************************************************
#     *                                                                       *
#     *    FreeRTOS tutorial books are available in pdf and paperback.        *
#     *    Complete, revised, and edited pdf reference manuals are also       *
#     *    available.                                                         *
#     *                                                                       *
#     *    Purchasing FreeRTOS documentation will not only help you, by       *
#     *    ensuring you get running as quickly as possible and with an        *
#     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
#     *    the FreeRTOS project to continue with its mission of providing     *
#     *    professional grade, cross platform, de facto standard solutions    *
#     *    for microcontrollers - completely free of charge!                  *
#     *                                                                       *
#     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
#     *                                                                       *
#     *    Thank you for using FreeRTOS, and thank you for your support!      *
#     *                                                                       *
#    ***************************************************************************
#
#
#    This file is part of the FreeRTOS distribution.
#
#    FreeRTOS is free software; you can redistribute it and/or modify it under
#    the terms of the GNU General Public License (version 2) as published by the
#    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
#    >>>NOTE<<< The modification to the GPL is included to allow you to
#    distribute a combined work that includes FreeRTOS without being obliged to
#    provide the source code for proprietary components outside of the FreeRTOS
#    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
#    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#    more details. You should have received a copy of the GNU General Public
#    License and the FreeRTOS license exception along with FreeRTOS; if not it
#    can be viewed here: http://www.freertos.org/a00114.html and also obtained
#    by writing to Richard Barry, contact details for whom are available on the
#    FreeRTOS WEB site.
#
#    1 tab == 4 spaces!
#
#    http://www.FreeRTOS.org - Documentation, latest information, license and
#    contact details.
#
#    http://www.SafeRTOS.com - A version that is certified for use in safety
#    critical systems.
#
#    http://www.OpenRTOS.com - Commercial support, development, porting,
#    licensing and training services.
#*/

FREE_RTOS=/home/alex/Documents/FreeRTOSv8.0.1/FreeRTOS
RTOS_SOURCE_DIR=$(FREE_RTOS)/Source
DEMO_COMMON_DIR=$(FREE_RTOS)/Demo/Common/Minimal
DEMO_INCLUDE_DIR=$(FREE_RTOS)/Demo/Common/include

OUTPUT = RTOS_Kernel

# Output directories
BIN = bin
OBJ = obj

OUTPUT := $(BIN)/$(OUTPUT)
ELF_OUTPUT:=$(OUTPUT).elf
HEX_OUTPUT:=$(OUTPUT).hex
BIN_OUTPUT:=$(OUTPUT).bin


CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
LDSCRIPT=atmel-rom.ld

LINKER_FLAGS=-mthumb -nostartfiles -Xlinker -o$(ELF_OUTPUT) -Xlinker -M -Xlinker -Map=rtosdemo.map

DEBUG=-g
OPTIM=-O0


VPATH +=$(RTOS_SOURCE_DIR) $(DEMO_COMMON_DIR) SrcAtmel ParTest ARM7_AT91SAM7A3 $(RTOS_SOURCE_DIR)/portable/MemMang boards peripherals/twi peripherals/spi peripherals/aic peripherals/pio peripherals/can peripherals/tc peripherals/pwmc peripherals/mezonin peripherals/pit utility peripherals/MSO_functions 
CFLAGS= $(DEBUG) \
		$(OPTIM) \
		-T$(LDSCRIPT) \
		-I . \
		-I $(RTOS_SOURCE_DIR)/include \
		-I peripherals \
		-I ARM7_AT91SAM7A3 \
		-I $(DEMO_INCLUDE_DIR) \
		-I ./SrcAtmel \
		-D SAM7_GCC \
		-D THUMB_INTERWORK \
		-mcpu=arm7tdmi \
		-D PACK_STRUCT_END=__attribute\(\(packed\)\) \
		-D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
		-fomit-frame-pointer \
		-mthumb-interwork \
		-fno-strict-aliasing \
		-Wall \
		-std=gnu11\
		-fno-dwarf2-cfi-asm
		
										
THUMB_SOURCE= \
		main.c \
		ParTest.c \
		BlockQ.c \
		blocktim.c \
		flash.c \
		integer.c \
		GenQTest.c \
		QPeek.c \
		dynamic.c \
		list.c \
		queue.c \
		tasks.c \
		port.c \
		heap_2.c \
		syscalls.c \
		twi.c \
		pio.c \
		pio_config.c \
		aic.c \
		tc.c \
		pit.c \
		can.c \
		pwmc.c \
		math.c \
		spi.c \
		mezonin.c \
		MSO_functions.c		

ARM_SOURCE= \
		portISR.c \
		my_isr.c \
		Cstartup_SAM7.c \


THUMB_OBJS = $(THUMB_SOURCE:.c=.o)
ARM_OBJS = $(ARM_SOURCE:.c=.o)
C_OBJECTS_THUMB = $(addprefix $(OBJ)/, $(THUMB_OBJS))
C_OBJECTS_ARM = $(addprefix $(OBJ)/, $(ARM_OBJS))


all: $(BIN) $(OBJ) $(BIN_OUTPUT)

$(BIN) $(OBJ):
	mkdir $@
	

$(BIN_OUTPUT) : $(HEX_OUTPUT)
	$(OBJCOPY) $(ELF_OUTPUT) -O binary $(BIN_OUTPUT)
	 
$(HEX_OUTPUT) : $(ELF_OUTPUT)
	$(OBJCOPY) $(ELF_OUTPUT) -O ihex $(HEX_OUTPUT)

$(ELF_OUTPUT) : $(C_OBJECTS_THUMB) $(C_OBJECTS_ARM) boot.s Makefile
	$(CC) $(CFLAGS) $(C_OBJECTS_ARM) $(C_OBJECTS_THUMB) $(LIBS) boot.s $(LINKER_FLAGS) 

$(C_OBJECTS_THUMB): $(OBJ)/%.o: %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) -mthumb $< -o $@

$(C_OBJECTS_ARM) : $(OBJ)/%.o: %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

clean :
	rm $(C_OBJECTS_THUMB)
	rm $(C_OBJECTS_ARM)
	touch Makefile
	rm $(ELF_OUTPUT)
	rm $(HEX_OUTPUT)
	
	
	




