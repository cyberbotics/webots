# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# WEBOTS_HOME is a sine qua non condition to run this Makefile
ifndef WEBOTS_HOME
  $(error The WEBOTS_HOME environment variable is not defined.)
endif

# WEBOTS_HOME_PATH is commonly defined in the caller Makefile
# but as it is not necessary on non-windows OS, it seems safer to reconstruct it there if required
ifndef WEBOTS_HOME_PATH
  null :=
  space := $(null) $(null)
  WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
endif

TARGET=libepuck.a

# OS determination and OS specific commands
include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

LIBPATH = $(WEBOTS_HOME)/projects/robots/gctronic/e-puck/transfer

MICROCHIP_PATH = $(LIBPATH)/xc16
export PATH := $(MICROCHIP_PATH)/bin:$(PATH)

C_SOURCES = \
  libepuck.c \
  $(LIBPATH)/library/acc_gyro/e_lsm330.c \
  $(LIBPATH)/library/a_d/advance_ad_scan/e_acc.c \
  $(LIBPATH)/library/a_d/advance_ad_scan/e_ad_conv.c \
  $(LIBPATH)/library/a_d/advance_ad_scan/e_micro.c \
  $(LIBPATH)/library/a_d/advance_ad_scan/e_prox.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_calc_po3030k.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_calc_po6030k.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_calc_po8030d.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_common.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_po3030k_registers.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_po6030k_registers.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_po8030d_registers.c \
  $(LIBPATH)/library/camera/fast_2_timer/e_timers.c \
  $(LIBPATH)/library/I2C/e_I2C_master_module.c \
  $(LIBPATH)/library/I2C/e_I2C_protocol.c \
  $(LIBPATH)/library/motor_led/e_init_port.c \
  $(LIBPATH)/library/motor_led/advance_one_timer/e_agenda.c \
  $(LIBPATH)/library/motor_led/advance_one_timer/e_led.c \
  $(LIBPATH)/library/motor_led/advance_one_timer/e_motors.c \
  $(LIBPATH)/library/motor_led/advance_one_timer/e_remote_control.c \
  $(LIBPATH)/library/utility/utility.c

AS_SOURCES = \
  e_interrupt.s

SUFFIXES += .d

NODEPS = clean
SOURCES = $(C_SOURCES) $(AS_SOURCES)
OBJECTS = $(patsubst %.c,%.o,$(C_SOURCES)) $(patsubst %.s,%.o,$(AS_SOURCES))
DEPFILES = $(patsubst %.c,%.d,$(C_SOURCES))

CCFLAGS = -Wall -mcpu=30f6014A
CCINCLUDE = -I"$(LIBPATH)/library"

ASFLAGS = --processor=30F6014A
ASINCLUDE = -I"$(LIBPATH)/library/codec" -I"$(LIBPATH)/library/uart"

ifeq ($(OSTYPE),darwin)
  CC = $(MICROCHIP_PATH)/bin/xc16-gcc
  AS = $(MICROCHIP_PATH)/bin/xc16-as
  AR = $(MICROCHIP_PATH)/bin/xc16-ar
else
  CC = xc16-gcc
  AS = xc16-as
  AR = xc16-ar
endif

ifeq ($(OSTYPE),linux)
  ifeq ($(OSARCH),x86_64)
    ifneq ($(MAKECMDGOALS),clean)
      ifeq ($(shell $(MICROCHIP_PATH)/bin/xc16-gcc --version),)
        $(error xc16-gcc cannot be executed. Installing the gcc:i386 library may help: "sudo apt-get install gcc-4.9-base:i386 libgcc1:i386 libc6:i386")
      endif
    endif
  endif
endif

COF = $(TARGET:.hex=.cof)

release debug profile: $(TARGET)

$(TARGET): $(OBJECTS)
	$(AR) rcs $(TARGET) $(addsuffix ",$(addprefix ",$(OBJECTS)))

%.d: %.c
	$(CC) $(CCFLAGS) $(CCINCLUDE) -MM -MT "$(patsubst %.c,%.o,$<)" "$<" -MF "$@"

%.o: %.c
	$(CC) -c $(CCFLAGS) $(CCINCLUDE) -o "$@" "$<"

%.o: %.s
	$(AS) $(ASFLAGS) $(ASINCLUDE) -o "$@" "$<"

ifeq (0, $(words $(findstring $(MAKECMDGOALS), $(NODEPS))))
-include $(DEPFILES)
endif

clean:
	rm -fr $(TARGET) $(COF) $(addsuffix ",$(addprefix ",$(DEPFILES) $(OBJECTS)))
