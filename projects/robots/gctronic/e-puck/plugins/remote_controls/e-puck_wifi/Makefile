# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

CXX_SOURCES = $(wildcard *.cpp)

ifeq ($(OS),Windows_NT)
# on Windows, need to link with WinSock2
LIBRARIES=-lws2_32
endif

USE_C_API = true

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
