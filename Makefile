CXX ?= g++
CXXFLAGS ?= -O3 -std=c++17 -Wall -Wextra -pthread
LDLIBS ?= -lX11 -lXext

.PHONY: all test clean
all: imx252_algo3_flow_v2

imx252_algo3_flow_v2: imx252_algo3_flow_v2.cpp imx252_flow_biquad.hpp
	$(CXX) $(CXXFLAGS) -o $@ imx252_algo3_flow_v2.cpp $(LDLIBS)

imx252_flow_biquad_test: imx252_flow_biquad_test.cpp imx252_flow_biquad.hpp
	$(CXX) $(CXXFLAGS) -o $@ imx252_flow_biquad_test.cpp

test: imx252_flow_biquad_test
	./imx252_flow_biquad_test

clean:
	rm -f imx252_algo3_flow_v2 imx252_flow_biquad_test
