# Host tests of the WALDO firmware logic (no board needed):  make -C tests/host
W := ../../src/waldo
CXXFLAGS := -std=c++17 -O1 -Wall -I$(W)

test: test_orchestrator test_view
	./test_orchestrator
	./test_view data/room_rot90.jpg 90 2 lcd_room.ppm 0 24 27 38 70 60 20 20   # open lcd_room.ppm to look

test_orchestrator: test_orchestrator.cpp $(W)/orchestrator.cpp $(W)/tracker.cpp $(W)/motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

# The LCD: the firmware's JPEG decode, rotation, view.cpp and the board's display.c.
test_view: test_view.cpp display_host.c $(W)/view.cpp $(W)/jpeg_frame.c $(W)/tjpgd/tjpgd.c
	$(CC) -O1 -Istubs -I../../board/DevKit-E8 -c display_host.c -o display_host.o
	$(CC) -O1 -I$(W) -c $(W)/jpeg_frame.c -o jpeg_frame.o
	$(CC) -O1 -I$(W) -c $(W)/tjpgd/tjpgd.c -o tjpgd.o
	$(CXX) $(CXXFLAGS) -I../../board/DevKit-E8 -o $@ test_view.cpp $(W)/view.cpp display_host.o jpeg_frame.o tjpgd.o
	rm -f display_host.o jpeg_frame.o tjpgd.o

clean:
	rm -f test_orchestrator test_view lcd_room.ppm
.PHONY: test clean
