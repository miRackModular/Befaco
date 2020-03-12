V1_COMPAT=1
RACK_DIR ?= ../..

SOURCES += $(wildcard src/*.cpp)

DISTRIBUTABLES += $(wildcard LICENSE*) res

include $(RACK_DIR)/plugin.mk
