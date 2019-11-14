# Define compilation toolchain
CC     := g++

# Compile options
IDIR   := /usr/include/python2.7
CFLAGS := -I$(IDIR) -Wall

# Python library
LINK   := -lpython2.7

# Use C++11
STD    := -std=c++11

EKFSLAM: src/ekfslam.h src/ekfslam.cpp src/main.cpp
	mkdir -p bin
	$(CC) $(CFLAGS) $(LINK) $(STD) -o bin/$@ $^

clean:
	rm -rf bin
