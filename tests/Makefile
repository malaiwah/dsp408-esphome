CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I../components/dsp408

all: test_protocol

test_protocol: test_protocol.cpp ../components/dsp408/protocol.h
	$(CXX) $(CXXFLAGS) -o $@ $<

run: test_protocol
	./test_protocol

clean:
	rm -f test_protocol

.PHONY: all run clean
