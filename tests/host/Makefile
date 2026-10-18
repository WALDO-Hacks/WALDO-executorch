# Host tests of the WALDO firmware logic (no board needed):  make -C tests/host
W := ../../src/waldo
CXXFLAGS := -std=c++17 -O1 -Wall -I$(W)

test: test_orchestrator
	./test_orchestrator

test_orchestrator: test_orchestrator.cpp $(W)/orchestrator.cpp $(W)/tracker.cpp $(W)/motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f test_orchestrator
.PHONY: test clean
