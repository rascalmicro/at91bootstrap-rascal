# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support
# ----------------------------------------------------------------------------
# Copyright (c) 2008, Atmel Corporation
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
# 
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission. 
# 
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------------------------------------------------------------------------- 

#     Makefile for compiling at91bootstrap-project

#-------------------------------------------------------------------------------
#        User-modifiable options
#-------------------------------------------------------------------------------

# Optimization level, put in comment for debugging
ifeq ($(OP_BOOTSTRAP), on)
OPTIMIZATION = -march=armv5t -Os
else
OPTIMIZATION = -O2
endif

# AT91 library directory
AT91LIB = ../at91lib
EXTLIBS = ../external_libs

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#        Command lines parameters managment
#-------------------------------------------------------------------------------

# CHIP        (M)  chip name
# BOARD       (M)  board name
# ORIGIN      (M)  memory where the module will be read
# DESTINATION (M)  memory where the module will be copied
# BIN_SIZE    (M)  module size to read and copy
# FROM_ADDR   (M*) source memory address offset where the module will be read
# FILE_NAME   (M*) name of the file to copy (for FAT formated memory)
# DEST_ADDR   (M)  destination memory address where the module will be copied
# VDDMEMSEL   (O)  EBI power supply selection (1.8V, 3.3V)
# STR_DESCR   (O)  Module description (optional user data)
# BOOTNAME    (O)  Bootstrap binary name
# TRACE_LEVEL (O)  if not defined, none trace

# Notes :
# (M) mandatory parameter
# (O) optional parameter
# (M) mandatory parameter depending on the boot (SDCard boot or other)

# Default output file basename (if not defined by user)
#-------------------------------------------------------------------------------
BOOTNAME_BASE = boot-$(BOARD)-$(ORIGIN)2$(DESTINATION)

ifndef    FPGAINIT
FPGAINIT = 0
endif

ifeq ($(FPGAINIT),1)
BOOTNAME = $(BOOTNAME_BASE)-fpga
else
BOOTNAME = $(BOOTNAME_BASE)
endif

# if there is only one binary to copy, its size, its source address and its 
# destination address can be given as makefile argument
#-------------------------------------------------------------------------------
ifdef BIN_SIZE
CFLAGS += -DBIN_SIZE=$(BIN_SIZE)
endif

ifdef FROM_ADDR
CFLAGS += -DFROM_ADDR=$(FROM_ADDR)
endif

ifdef DEST_ADDR
CFLAGS += -DDEST_ADDR=$(DEST_ADDR)
endif

ifdef FILE_NAME
CFLAGS += -DFILE_NAME=$(FILE_NAME)
endif

ifdef STR_DESCR
CFLAGS += -DSTR_DESCR=$(STR_DESCR)
endif

# EBI power supply : EBI1V8 = 1.8V or EBI3V3 = 3.3V
#-------------------------------------------------------------------------------
ifeq ($(VDDMEMSEL),EBI1V8)
CFLAGS += -DVDDMEMSEL_EBI1V8
endif

ifeq ($(VDDMEMSEL),EBI3V3)
CFLAGS += -DVDDMEMSEL_EBI3V3
endif

#ifdef VDDMEMSEL
#CFLAGS += -DVDDMEMSEL=$(VDDMEMSEL)
#endif

# Special init for CAP9
#-------------------------------------------------------------------------------
ifeq ($(CHIP),at91cap9)
ifeq ($(FPGAINIT),1)
CFLAGS += -DFPGAINIT
endif
endif

# PATH_BOOT_H allows to use a special boot.h for specific project with several 
# modules to copy. If not defined, the normal boot.h of the at91bootstrap 
# project is used
#-------------------------------------------------------------------------------
ifndef PATH_BOOT_H
PATH_BOOT_H = .
endif

# PATH_OUTPUT_DIR allows to choose the directory where the BIN directory 
# will be created
#-------------------------------------------------------------------------------
ifdef PATH_OUTPUT_DIR
BIN = $(PATH_OUTPUT_DIR)/bin
OBJ = $(PATH_OUTPUT_DIR)/obj
endif

#-------------------------------------------------------------------------------
#        Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE= arm-none-eabi-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Wall -mlong-calls -ffunction-sections
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) 
CFLAGS += -D$(CHIP) 
CFLAGS += -DBOARD=$(BOARD)
CFLAGS += -DORIGIN_$(ORIGIN)
CFLAGS += -DDESTINATION_$(DESTINATION)
CFLAGS += -DTRACE_LEVEL=$(TRACE_LEVEL)

ASFLAGS = -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = -g $(OPTIMIZATION) -nostartfiles -Wl,--gc-sections,-Map=$(OUTPUT).map,--cref

#-------------------------------------------------------------------------------
#        Files
#-------------------------------------------------------------------------------

# Includes paths
INCLUDES =  -I$(PATH_BOOT_H)
INCLUDES += -I$(AT91LIB)
INCLUDES += -I$(AT91LIB)/boards/$(BOARD) 
INCLUDES += -I$(AT91LIB)/peripherals 
INCLUDES += -I$(AT91LIB)/components 
INCLUDES += -I$(AT91LIB)/memories
INCLUDES += -I$(EXTLIBS)/fat

# Sources path
VPATH += $(AT91LIB)/utility
VPATH += $(AT91LIB)/peripherals 
VPATH += $(AT91LIB)/peripherals/pio 
VPATH += $(AT91LIB)/peripherals/aic
VPATH += $(AT91LIB)/peripherals/dbgu
VPATH += $(AT91LIB)/boards/$(BOARD) 
VPATH += $(AT91LIB)/boards/$(BOARD)/$(CHIP)
VPATH += $(AT91LIB)/peripherals/cp15

# Objects built from C source files
C_OBJECTS = main.o
C_OBJECTS += pio.o 
C_OBJECTS += aic.o
C_OBJECTS += board_memories.o 
C_OBJECTS += board_lowlevel.o
C_OBJECTS += stdio.o
C_OBJECTS += string.o
C_OBJECTS += dbgu.o
C_OBJECTS += trace.o
C_OBJECTS += util.o

ifeq ($(CHIP),at91cap9)
ifeq ($(FPGAINIT),1)
C_OBJECTS += board_fpga_init.o
endif
endif

# Objects built from Assembly source files
ASM_OBJECTS = board_cstartup.o
ASM_OBJECTS += cp15_asm.o

#-------------------------------------------------------------------------------
# MEMORY SOURCE : Dataflash
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),dataflash)

# Cf board.h to get BOARD_AT45_A connection parameters
ifndef SLOT
SLOT:=SLOT_A
endif

CFLAGS += -DAT45_$(SLOT)

VPATH += $(AT91LIB)/memories/spi-flash

C_OBJECTS += BootAT45.o
C_OBJECTS += spid.o 
C_OBJECTS += at45.o 
C_OBJECTS += at45d.o
C_OBJECTS += math.o

MEMORIES = sram

endif
#-------------------------------------------------------------------------------
# MEMORY SOURCE : Serialflash
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),serialflash)

# Cf board.h to get BOARD_AT26_A connection parameters
ifndef SLOT
SLOT:=SLOT_A
endif

CFLAGS += -DAT26_$(SLOT)

VPATH += $(AT91LIB)/memories//spi-flash

C_OBJECTS += BootAT26.o 
C_OBJECTS += spid.o
C_OBJECTS += at26.o
C_OBJECTS += at26d.o

MEMORIES = sram

endif
#-------------------------------------------------------------------------------
# MEMORY SOURCE : Nandflash
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),nandflash)

VPATH += $(AT91LIB)/memories/nandflash

C_OBJECTS += BootNand.o
C_OBJECTS += RawNandFlash.o
C_OBJECTS += EccNandFlash.o
C_OBJECTS += SkipBlockNandFlash.o
C_OBJECTS += NandFlashModel.o
C_OBJECTS += NandFlashModelList.o
C_OBJECTS += NandSpareScheme.o
C_OBJECTS += hamming.o

MEMORIES = sram

endif
#-------------------------------------------------------------------------------
# MEMORY SOURCE : SDcard
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),sdcard)

INCLUDES += -I. 

VPATH += $(AT91LIB)/memories
VPATH += $(AT91LIB)/memories/sdmmc
VPATH += $(AT91LIB)/peripherals/mci
VPATH += $(EXTLIBS)/fat/fatfs
VPATH += $(EXTLIBS)/fat/fatfs/src

C_OBJECTS += BootSdcard.o
C_OBJECTS += diskio.o 
C_OBJECTS += tff.o
C_OBJECTS += Media.o
C_OBJECTS += MEDSdcard.o
C_OBJECTS += mci.o
C_OBJECTS += sdmmc_mci.o

MEMORIES = sram

endif
#-------------------------------------------------------------------------------
# MEMORY SOURCE : norflash
# Code is eXecuted In Place: MEMORIES = norflash
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),norflash)

#the following flag is used in board_lowlevel.c file
CFLAGS += -Dnorflash

VPATH += $(AT91LIB)/memories/norflash

C_OBJECTS += BootNor.o

MEMORIES = norflash

endif

#-------------------------------------------------------------------------------
# MEMORY SOURCE : eeprom
#-------------------------------------------------------------------------------
ifeq ($(ORIGIN),eeprom)

#the following flag is used in board_lowlevel.c file
CFLAGS += -Deeprom

VPATH += $(AT91LIB)/peripherals/twi
VPATH += $(AT91LIB)/drivers/twi
VPATH += $(AT91LIB)/drivers/async

C_OBJECTS += BootEeprom.o
C_OBJECTS += twi.o
C_OBJECTS += twid.o
C_OBJECTS += async.o
C_OBJECTS += math.o

MEMORIES = sram

endif

#-------------------------------------------------------------------------------
# Optimization bootstrap : at91sam9260
#-------------------------------------------------------------------------------
ifeq ($(OP_BOOTSTRAP), on)
C_INTERWORK_FLAGS += $(CFLAGS)
C_INTERWORK_FLAGS += -mthumb-interwork
CFLAGS += -mthumb
CFLAGS += -DOP_BOOTSTRAP_$(OP_BOOTSTRAP)
endif

ifeq ($(OP_BOOTSTRAP), on)
C_INTERWORK_OBJECTS = cp15.o
else
C_OBJECTS += cp15.o
endif
#===============================================================================

OUTPUT      := $(BIN)/$(BOOTNAME)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------
# Append OBJ and BIN directories to filenames
C_OBJECTS   := $(addprefix $(OBJ)/, $(C_OBJECTS))
ASM_OBJECTS := $(addprefix $(OBJ)/, $(ASM_OBJECTS))
C_INTERWORK_OBJECTS   := $(addprefix $(OBJ)/, $(C_INTERWORK_OBJECTS))

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	@mkdir $@

$(MEMORIES): $(ASM_OBJECTS) $(C_OBJECTS) $(C_INTERWORK_OBJECTS)
	echo "**************************************************************************"
	echo "***** Boot from $(ORIGIN) to $(DESTINATION) (execute in $(MEMORIES)) *****"
	echo "--------------------------------------------------------------------------"
	$(CC) $(LDFLAGS) -T"$(AT91LIB)/boards/$(BOARD)/$(CHIP)/$(MEMORIES).lds" -o $(OUTPUT).elf $^
	$(OBJCOPY) -O binary $(OUTPUT).elf $(OUTPUT).bin
	$(SIZE) $(ASM_OBJECTS) $(C_OBJECTS) $(OUTPUT).elf

$(C_OBJECTS): $(OBJ)/%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

$(C_INTERWORK_OBJECTS): $(OBJ)/%.o: %.c Makefile
	$(CC) $(C_INTERWORK_FLAGS) -c -o $@ $<

$(ASM_OBJECTS): $(OBJ)/%.o: %.S Makefile
	$(CC) $(ASFLAGS) -c -o $@ $<
	
debug_sram: sram
	perl ../resources/gdb/debug.pl $(OUTPUT).elf	

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.elf $(BIN)/*.map


