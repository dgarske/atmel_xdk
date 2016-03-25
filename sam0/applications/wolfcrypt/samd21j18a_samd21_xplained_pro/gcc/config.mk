#
# Copyright (c) 2011 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop
#

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = ../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m0plus

# Target part: none, sam3n4 or sam4l4aa
PART = samd21j18a

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = wolfcrypt_flash.elf
TARGET_SRAM = wolfcrypt_sram.elf

# List of C source files.
CSRCS = \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common2/services/delay/sam0/systick_counter.c      \
       sam0/applications/wolfcrypt/main.c                 \
       sam0/boards/samd21_xplained_pro/board_init.c       \
       sam0/drivers/port/port.c                           \
       sam0/drivers/sercom/i2c/i2c_sam0/i2c_master.c      \
       sam0/drivers/sercom/sercom.c                       \
       sam0/drivers/sercom/sercom_interrupt.c             \
       sam0/drivers/rtc/rtc_sam_d_r/rtc_count.c           \
       sam0/drivers/rtc/rtc_sam_d_r/rtc_count_interrupt.c \
       sam0/drivers/rtc/rtc_sam_d_r/rtc_calendar.c        \
       sam0/drivers/tcc/tcc.c                             \
       sam0/drivers/tcc/tcc_callback.c                    \
       sam0/drivers/sercom/usart/usart.c                  \
       sam0/drivers/sercom/usart/usart_interrupt.c        \
       sam0/drivers/system/clock/clock_samd21_r21_da/clock.c \
       sam0/drivers/system/clock/clock_samd21_r21_da/gclk.c \
       sam0/drivers/system/interrupt/system_interrupt.c   \
       sam0/drivers/system/pinmux/pinmux.c                \
       sam0/drivers/system/system.c                       \
       sam0/utils/cmsis/samd21/source/gcc/startup_samd21.c \
       sam0/utils/cmsis/samd21/source/system_samd21.c     \
       sam0/utils/stdio/read.c                            \
       sam0/utils/stdio/write.c                           \
       sam0/utils/syscalls/gcc/syscalls.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/services/serial                             \
       common2/services/delay                             \
       common/utils                                       \
       sam0/applications/wolfcrypt                        \
       sam0/applications/wolfcrypt/samd21j18a_samd21_xplained_pro \
       sam0/applications/wolfcrypt/samd21j18a_samd21_xplained_pro/gcc \
       sam0/boards                                        \
       sam0/boards/samd21_xplained_pro                    \
       sam0/drivers/port                                  \
       sam0/drivers/sercom                                \
       sam0/drivers/sercom/i2c                            \
       sam0/drivers/sercom/i2c/i2c_sam0                   \
       sam0/drivers/sercom/usart                          \
       sam0/drivers/rtc                                   \
       sam0/drivers/tcc                                   \
       sam0/drivers/system                                \
       sam0/drivers/system/clock                          \
       sam0/drivers/system/clock/clock_samd21_r21_da      \
       sam0/drivers/system/interrupt                      \
       sam0/drivers/system/interrupt/system_interrupt_samd21 \
       sam0/drivers/system/pinmux                         \
       sam0/drivers/system/power                          \
       sam0/drivers/system/power/power_sam_d_r            \
       sam0/drivers/system/reset                          \
       sam0/drivers/system/reset/reset_sam_d_r            \
       sam0/utils                                         \
       sam0/utils/cmsis/samd21/include                    \
       sam0/utils/cmsis/samd21/source                     \
       sam0/utils/header_files                            \
       sam0/utils/preprocessor                            \
       sam0/utils/stdio/stdio_serial                      \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM0l_math                         

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam0/utils/linker_scripts/samd21/gcc/samd21j18a_flash.ld
LINKER_SCRIPT_SRAM  = sam0/utils/linker_scripts/samd21/gcc/samd21j18a_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam0/boards/samd21_xplained_pro/debug_scripts/gcc/samd21_xplained_pro_flash.gdb
DEBUG_SCRIPT_SRAM  = sam0/boards/samd21_xplained_pro/debug_scripts/gcc/samd21_xplained_pro_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O0

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
       -D ARM_MATH_CM0PLUS=true                           \
       -D BOARD=SAMD21_XPLAINED_PRO                       \
       -D I2C_MASTER_CALLBACK_MODE=false                  \
       -D USART_CALLBACK_MODE=true                        \
       -D __SAMD21J18A__                                  \
       -D EXTINT_CALLBACK_MODE=true                       \
       -D SYSTICK_MODE                                    \
       -D TCC_ASYNC=true                                  \
       -D RTC_COUNT_ASYNC=true

# Copy CPPFLAGS to ASM flags
ASFLAGS += $(CPPFLAGS)

# Extra flags to use when linking
LDFLAGS = \
    -Xlinker --defsym=STACK_SIZE=0x4000 \
    -u _printf_float

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 


## ----------------------------------------------------------------------------- ##
## wolfSSL ##
## ----------------------------------------------------------------------------- ##
#wolfSSL source files
# core
CSRCS += thirdparty/wolfssl/wolfcrypt/src/random.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/logging.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/memory.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/wc_encrypt.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/wc_port.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/error.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/signature.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/hash.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/asn.c
# crypto
CSRCS += thirdparty/wolfssl/wolfcrypt/src/aes.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/dh.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/md5.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/hmac.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/rsa.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/sha.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/sha256.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/sha512.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/curve25519.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/ed25519.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/ecc.c
# math libraries
CSRCS += thirdparty/wolfssl/wolfcrypt/src/tfm.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/integer.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/fe_low_mem.c
CSRCS += thirdparty/wolfssl/wolfcrypt/src/ge_low_mem.c
# port
CSRCS += thirdparty/wolfssl/wolfcrypt/src/port/atmel/atmel.c

#wolfSSL includes
INC_PATH += thirdparty/wolfssl

#wolfSSL preprocessor (use the user_settings.h)
CPPFLAGS += -DWOLFSSL_USER_SETTINGS

#wolfTest
CSRCS += thirdparty/wolfssl/wolfcrypt/test/test.c
#wolfBenchmark
CSRCS += thirdparty/wolfssl/wolfcrypt/benchmark/benchmark.c
## ----------------------------------------------------------------------------- ##


## ----------------------------------------------------------------------------- ##
## CryptoAuthLib ##
## ----------------------------------------------------------------------------- ##
#CryptoAuthLib source files
CSRCS += thirdparty/cryptoauthlib/basic/atca_basic.c
CSRCS += thirdparty/cryptoauthlib/basic/atca_helpers.c
CSRCS += thirdparty/cryptoauthlib/tls/atcatls.c
CSRCS += thirdparty/cryptoauthlib/atca_iface.c
CSRCS += thirdparty/cryptoauthlib/atca_command.c
CSRCS += thirdparty/cryptoauthlib/atca_device.c
CSRCS += thirdparty/cryptoauthlib/atca_cfgs.c
CSRCS += thirdparty/cryptoauthlib/host/atca_host.c
CSRCS += thirdparty/cryptoauthlib/hal/atca_hal.c
CSRCS += thirdparty/cryptoauthlib/hal/hal_samd21_i2c_asf.c
CSRCS += thirdparty/cryptoauthlib/hal/hal_samd21_timer_asf.c
CSRCS += thirdparty/cryptoauthlib/certs/provision.c
CSRCS += thirdparty/cryptoauthlib/certs/cert_def_1_signer.c
CSRCS += thirdparty/cryptoauthlib/certs/cert_def_2_device.c
CSRCS += thirdparty/cryptoauthlib/crypto/atca_crypto_sw_sha1.c
CSRCS += thirdparty/cryptoauthlib/crypto/atca_crypto_sw_sha2.c
CSRCS += thirdparty/cryptoauthlib/crypto/hashes/sha1_routines.c
CSRCS += thirdparty/cryptoauthlib/crypto/hashes/sha2_routines.c
CSRCS += thirdparty/cryptoauthlib/atcacert/atcacert_date.c
CSRCS += thirdparty/cryptoauthlib/atcacert/atcacert_client.c
CSRCS += thirdparty/cryptoauthlib/atcacert/atcacert_def.c
CSRCS += thirdparty/cryptoauthlib/atcacert/atcacert_der.c
CSRCS += thirdparty/cryptoauthlib/atcacert/atcacert_host_hw.c
CSRCS += thirdparty/cryptoauthlib/atcacert/atcacert_host_sw.c

#CryptoAuthLib includes
INC_PATH += thirdparty/cryptoauthlib

#CryptoAuthLib preprocessor
CPPFLAGS += \
    -DATCAPRINTF \
    -DATCA_HAL_I2C
## ----------------------------------------------------------------------------- ##
