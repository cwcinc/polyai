BUILD_DIR := build
TARGET := polyai
WASMTIME := wasmtime

ifndef WASI_SDK
$(error Please set the WASI_SDK environment variable, e.g. `export WASI_SDK=/path/to/wasi-sdk`)
endif

TOOLCHAIN_FILE := $(WASI_SDK)/share/cmake/wasi-sdk.cmake
CMAKE_FLAGS := -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN_FILE) -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

SRC_FILES := $(shell find src -type f)
EXECUTABLE := $(BUILD_DIR)/$(TARGET)

all: $(EXECUTABLE) run

$(BUILD_DIR)/build.ninja: CMakeLists.txt
	@echo "Configuring project using WASI SDK at: $(WASI_SDK)"
	cmake -S . -B $(BUILD_DIR) -G Ninja $(CMAKE_FLAGS)

$(EXECUTABLE): $(BUILD_DIR)/build.ninja $(SRC_FILES)
	@echo "Building $(TARGET)..."
	cmake --build $(BUILD_DIR)

run: $(EXECUTABLE)
	$(WASMTIME) $(EXECUTABLE)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all run clean
