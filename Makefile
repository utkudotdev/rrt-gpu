all: main_cpp main_cu

CFLAGS := -Wall -Wextra -std=c++17
EIGEN_FLAGS := $(shell pkgconf --cflags --libs eigen3)
SDL_FLAGS := $(shell pkgconf --cflags --libs sdl3)

main_cpp: main.cpp
	mkdir -p build
	g++ main.cpp $(CFLAGS) $(EIGEN_FLAGS) $(SDL_FLAGS) -o build/main_cpp

main_cu: main.cu
	mkdir -p build
	nvcc main.cu -o build/main_cu

clean:
	rm -rf build
