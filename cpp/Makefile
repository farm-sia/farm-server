LDFLAGS += -lpthread -lwiringPi -lroscpp -lrostime -lrosconsole
OBJECTS := main.o ws-server.o ros_node.o
CXXFLAGS ?= -Wall -Wextra
CXXFLAGS += -std=c++11
CXX ?= g++
INCLUDES := -I /opt/ros/noetic/include
LINKS := -L /opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib
HEADERS :=
EXECUTABLE := main

dummy: OBJECTS += rpi_gpio_dummy.o
debug: OBJECTS += rpi_gpio.o
release: OBJECTS += rpi_gpio.o

dummy : CXXFLAGS += -Og -g -pg
debug : CXXFLAGS += -Og -g -pg
release : CXXFLAGS += -O3 -fno-exceptions
dummy : LDFLAGS +=
debug : LDFLAGS +=
release : LDFLAGS += -O
dummy : BUILD_TYPE := debug
debug : BUILD_TYPE := debug
release : BUILD_TYPE := release

dummy: $(EXECUTABLE)
debug: $(EXECUTABLE)
release: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) $(HEADERS)
	$(CXX) $(OBJECTS) $(INCLUDES) $(LINKS) $(LDFLAGS) -o $@

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -c

.PHONY: clean $(BOARD_LIB)

clean:
	$(RM) $(EXECUTABLE) $(OBJECTS)
	$(MAKE) -C ../lib clean
