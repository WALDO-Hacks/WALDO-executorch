# Host tests of the WALDO firmware logic (no board needed):  make -C tests/host
W := ../../src/waldo
CXXFLAGS := -std=c++17 -O1 -Wall -I$(W)

test: test_orchestrator test_tracking_loop test_runner test_view test_brain
	./test_orchestrator
	./test_tracking_loop
	./test_runner
	SIM_TICK_MS=100 ./test_runner   # the board's main loop: ~100 ms per pass while tracking
	./test_brain
	./test_view data/room_rot90.jpg 90 2 lcd_room.ppm 0 24 27 38 70 60 20 20   # open lcd_room.ppm to look

test_orchestrator: test_orchestrator.cpp $(W)/orchestrator.cpp $(W)/tracker.cpp $(W)/motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

# A person running past: lag, a pillar, and a sprint out of view.
test_runner: test_runner.cpp $(W)/orchestrator.cpp $(W)/tracker.cpp $(W)/motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

# The steering loop against lens FOVs of 40-100 deg (the tracker assumes 60).
test_tracking_loop: test_tracking_loop.cpp $(W)/tracker.cpp $(W)/motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

# The LCD: the firmware's JPEG decode, rotation, view.cpp and the board's display.c.
test_view: test_view.cpp display_host.c $(W)/view.cpp $(W)/jpeg_frame.c $(W)/tjpgd/tjpgd.c
	$(CC) -O1 -Istubs -I../../board/DevKit-E8 -c display_host.c -o display_host.o
	$(CC) -O1 -I$(W) -c $(W)/jpeg_frame.c -o jpeg_frame.o
	$(CC) -O1 -I$(W) -c $(W)/tjpgd/tjpgd.c -o tjpgd.o
	$(CXX) $(CXXFLAGS) -I../../board/DevKit-E8 -o $@ test_view.cpp $(W)/view.cpp display_host.o jpeg_frame.o tjpgd.o
	rm -f display_host.o jpeg_frame.o tjpgd.o

# Two-brain mailbox: the HE's writer (he/he_log.c, brain_link.h) against the HP's reader.
test_brain: test_brain.cpp $(W)/brain_hp.cpp ../../he/he_log.c
	$(CC) -O1 -Istubs/brain -Istubs -I../../src -I../../he -DBRAIN_MAILBOX='(&host_box)' \
	  -I$(W) -include stubs/brain_host_box_c.h -c ../../he/he_log.c -o he_log.o
	$(CXX) $(CXXFLAGS) -Istubs/brain -Istubs -I../../src -DBRAIN_MAILBOX='(&host_box)' \
	  -include stubs/brain_host_box.h -o $@ test_brain.cpp $(W)/brain_hp.cpp he_log.o
	rm -f he_log.o

clean:
	rm -f test_orchestrator test_tracking_loop test_runner test_view test_brain lcd_room.ppm
.PHONY: test clean
