CXX ?= g++
RPLIDAR_SDK_ROOT ?= $(HOME)/rplidar_sdk
BUILD_DIR ?= build
TARGET := $(BUILD_DIR)/real_vehicle_lidar_agent
SDK_LIBRARY := $(RPLIDAR_SDK_ROOT)/output/Linux/Release/libsl_lidar_sdk.a

CPPFLAGS += -I$(RPLIDAR_SDK_ROOT)/sdk/include -I$(RPLIDAR_SDK_ROOT)/sdk/src
CXXFLAGS += -std=c++11 -O2 -Wall -Wextra -pedantic
LDLIBS += $(SDK_LIBRARY) -lpthread -lrt

.PHONY: all clean check-sdk

all: check-sdk $(TARGET)

check-sdk:
	@test -f "$(RPLIDAR_SDK_ROOT)/sdk/include/sl_lidar.h" || \
		(echo "RPLIDAR SDK headers not found at $(RPLIDAR_SDK_ROOT)"; exit 1)
	@test -f "$(SDK_LIBRARY)" || \
		(echo "Build the SDK first: make -C $(RPLIDAR_SDK_ROOT)"; exit 1)

$(TARGET): main.cpp
	@mkdir -p "$(BUILD_DIR)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS)

clean:
	rm -rf "$(BUILD_DIR)"
