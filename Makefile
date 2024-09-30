
PROJECT := $(notdir $(CURDIR))

$(PROJECT)_SOURCE := $(wildcard src/*.c)  \
    $(TREMO_SDK_PATH)/platform/system/printf-stdarg.c  \
    $(TREMO_SDK_PATH)/platform/system/system_cm4.c  \
    $(TREMO_SDK_PATH)/platform/system/startup_cm4.S \
    $(wildcard $(TREMO_SDK_PATH)/drivers/peripheral/src/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/lora/system/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/lora/system/crypto/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/lora/radio/sx126x/*.c)  \
    $(wildcard $(TREMO_SDK_PATH)/lora/driver/*.c) \
    $(wildcard $(TREMO_SDK_PATH)/lora/mac/*.c) \
    $(TREMO_SDK_PATH)/lora/mac/region/Region.c \
    $(TREMO_SDK_PATH)/lora/mac/region/RegionCommon.c \
    $(TREMO_SDK_PATH)/lora/mac/region/RegionAS923.c

$(PROJECT)_INC_PATH := inc \
    $(TREMO_SDK_PATH)/platform/CMSIS \
    $(TREMO_SDK_PATH)/platform/common \
    $(TREMO_SDK_PATH)/platform/system \
    $(TREMO_SDK_PATH)/drivers/crypto/inc \
    $(TREMO_SDK_PATH)/drivers/peripheral/inc \
    $(TREMO_SDK_PATH)/lora/driver \
    $(TREMO_SDK_PATH)/lora/system \
    $(TREMO_SDK_PATH)/lora/system/crypto \
    $(TREMO_SDK_PATH)/lora/radio \
    $(TREMO_SDK_PATH)/lora/radio/sx126x \
    $(TREMO_SDK_PATH)/lora/mac \
    $(TREMO_SDK_PATH)/lora/mac/region

$(PROJECT)_CFLAGS  := -Wall -Os -ffunction-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fsingle-precision-constant -std=gnu99 -fno-builtin-printf -fno-builtin-sprintf -fno-builtin-snprintf
$(PROJECT)_DEFINES := -DCONFIG_DEBUG_UART=UART0 -DREGION_AS923

$(PROJECT)_LDFLAGS := -Wl,--gc-sections -Wl,--wrap=printf -Wl,--wrap=sprintf -Wl,--wrap=snprintf

$(PROJECT)_LIBS := $(TREMO_SDK_PATH)/drivers/crypto/lib/libcrypto.a

$(PROJECT)_LINK_LD := cfg/gcc.ld

# please change the settings to download the app
#SERIAL_PORT        :=
#SERIAL_BAUDRATE    :=
#$(PROJECT)_ADDRESS :=

##################################################################################################
include $(TREMO_SDK_PATH)/build/make/common.mk



