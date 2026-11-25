# Host tests for AlpsHID/AlpsProtocol.hpp (the same code the kext runs).
#
#   make test                 unit tests under AddressSanitizer and UBSan
#   make fuzz                 build the libFuzzer targets
#   make run-fuzz             run every fuzzer for FUZZ_SECONDS (default 60) on the corpus in corpus/<target>
#
# Needs clang with libFuzzer for the fuzz targets (Linux: clang + libclang-rt-dev, macOS: Homebrew llvm;
# Apple's clang has no libFuzzer). The unit tests build with any C++17 compiler.

ifeq ($(origin CXX),default)
CXX := clang++
endif
FUZZ_SECONDS ?= 60
SRC_DIR := ../AlpsHID
BUILD := build

CXXFLAGS := -std=c++17 -O1 -g -Wall -Wextra -Werror -I$(SRC_DIR)
SANITIZE := -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer
HEADERS := $(SRC_DIR)/AlpsProtocol.hpp

FUZZERS := fuzz_input_report fuzz_feature_reply

.PHONY: test fuzz run-fuzz clean

test: $(BUILD)/unit
	./$(BUILD)/unit

$(BUILD)/unit: AlpsProtocolTests.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SANITIZE) $< -o $@

fuzz: $(addprefix $(BUILD)/,$(FUZZERS))

$(BUILD)/fuzz_%: fuzz_%.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SANITIZE) -fsanitize=fuzzer $< -o $@

run-fuzz: fuzz
	@for f in $(FUZZERS); do \
		mkdir -p corpus/$$f $(BUILD)/corpus/$$f; \
		echo "== $$f ($(FUZZ_SECONDS) s)"; \
		./$(BUILD)/$$f -max_total_time=$(FUZZ_SECONDS) -print_final_stats=1 $(BUILD)/corpus/$$f corpus/$$f || exit 1; \
	done

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
