LDFLAGS += -lpthread -lwiringPi -lroscpp -lrostime -lrosconsole -lroscpp_serialization -lboost_thread -lactionlib
OBJECTS := main.o ws-server.o ros_node.o rpi_gpio.o
CXXFLAGS ?= -Wall -Wextra
CXXFLAGS += -std=c++11
CXX ?= g++
INCLUDES := -I /opt/ros/noetic/include
LINKS := -L /opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib
HEADERS :=
EXECUTABLE := main

debug : CXXFLAGS += -Og -g -pg
release : CXXFLAGS += -O3 -fno-exceptions
debug : LDFLAGS +=
release : LDFLAGS += -O
debug : BUILD_TYPE := debug
release : BUILD_TYPE := release

debug: $(EXECUTABLE)
release: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) $(HEADERS)
	$(CXX) $(OBJECTS) $(INCLUDES) $(LINKS) $(LDFLAGS) -o $@

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -c

.PHONY: clean $(BOARD_LIB)

clean:
	$(RM) $(EXECUTABLE) $(OBJECTS)
