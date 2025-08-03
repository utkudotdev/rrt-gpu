.PHONY: main test clean a

CXX := c++
NVCC := nvcc

INCLUDE_FLAGS := -Isrc -Ideps/doctest/doctest $(shell pkgconf --cflags eigen3) $(shell pkgconf --cflags sdl3)
CXXFLAGS := -Wall -Wextra -std=c++23 -g $(INCLUDE_FLAGS)
LINK_FLAGS := $(shell pkgconf --libs eigen3) $(shell pkgconf --libs sdl3)

SRC_DIR := src
TEST_DIR := test
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

SRC_HEADERS := $(shell find $(SRC_DIR) -type f -name '*.hpp')

MAIN_EXEC := $(BUILD_DIR)/main
TEST_EXEC := $(BUILD_DIR)/test_runner

main: $(MAIN_EXEC)

$(ALL_BUILD_DIRS):
	@mkdir -p $(ALL_BUILD_DIRS)

# executable files
$(MAIN_EXEC): $(MAIN_OBJS) $(SRC_HEADERS) | $(ALL_BUILD_DIRS)
	$(CXX) $(MAIN_OBJS) -o $@ $(LINK_FLAGS)

$(TEST_EXEC): $(TEST_OBJS) $(SRC_HEADERS) | $(ALL_BUILD_DIRS)
	$(CXX) $(TEST_OBJS) -o $@ $(LINK_FLAGS)

# object files
$(BUILD_DIR)/%.o: %.cpp | $(ALL_BUILD_DIRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TEST_EXEC)
	./$(TEST_EXEC)

clean:
	rm -rf $(BUILD_DIR)
