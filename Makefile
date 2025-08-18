.PHONY: all main test clean

CXX := c++
NVCC := nvcc

INCLUDE_FLAGS := -Isrc -Ideps/doctest/doctest $(shell pkgconf --cflags eigen3) $(shell pkgconf --cflags sdl3) $(shell pkgconf --cflags benchmark)
CXXFLAGS := -Wall -Wextra -std=c++23 -g $(INCLUDE_FLAGS) -MMD -MP
LINK_FLAGS := $(shell pkgconf --libs eigen3) $(shell pkgconf --libs sdl3) $(shell pkgconf --libs benchmark)

SRC_DIR := src
TEST_DIR := test
BENCH_DIR := bench
BUILD_DIR := build

SRC_DIRS := $(sort $(shell find $(SRC_DIR) -type d))
TEST_DIRS := $(sort $(shell find $(TEST_DIR) -type d))
BUILD_SRC_DIRS := $(patsubst %,$(BUILD_DIR)/%,$(SRC_DIRS))
BUILD_TEST_DIRS := $(patsubst %,$(BUILD_DIR)/%,$(TEST_DIRS))
ALL_BUILD_DIRS := $(BUILD_DIR) $(BUILD_SRC_DIRS) $(BUILD_TEST_DIRS)

MAIN_SRCS := $(shell find $(SRC_DIR) -type f -name '*.cpp')
MAIN_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(MAIN_SRCS)) 

TEST_SRCS := $(shell find $(TEST_DIR) -type f -name '*.cpp')
TEST_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TEST_SRCS))


BENCH_SRCS := $(shell find $(BENCH_DIR) -type f -name '*.cpp')
BENCH_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(BENCH_SRCS))

MAIN_EXEC := $(BUILD_DIR)/main
TEST_EXEC := $(BUILD_DIR)/test_runner
BENCH_EXEC := $(BUILD_DIR)/bench_kdtree

-include $(patsubst %.o,%.d,$(MAIN_OBJS)) $(patsubst %.o,%.d,$(TEST_OBJS)) $(patsubst %.o,%.d,$(BENCH_OBJS))

all: main $(TEST_EXEC) $(BENCH_EXEC)

main: $(MAIN_EXEC)

test: $(TEST_EXEC)
	./$(TEST_EXEC)

bench: $(BENCH_EXEC)
	./$(BENCH_EXEC)

$(ALL_BUILD_DIRS):
	@mkdir -p $(ALL_BUILD_DIRS)

# executable files
$(MAIN_EXEC): $(MAIN_OBJS) | $(ALL_BUILD_DIRS)
	$(CXX) $(MAIN_OBJS) -o $@ $(LINK_FLAGS)

$(TEST_EXEC): $(TEST_OBJS) | $(ALL_BUILD_DIRS)
	$(CXX) $(TEST_OBJS) -o $@ $(LINK_FLAGS)

$(BENCH_EXEC): $(BENCH_OBJS) | $(ALL_BUILD_DIRS)
	$(CXX) $(BENCH_OBJS) -o $@ $(LINK_FLAGS)

# object files
$(BUILD_DIR)/%.o: %.cpp | $(ALL_BUILD_DIRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
