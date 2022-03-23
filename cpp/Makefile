LDFLAGS += -lpthread -lwiringPi
OBJECTS := main.o ws-server.o
CXXFLAGS ?= -Wall -Wextra
CXXFLAGS += -std=c++11
CXX ?= g++
INCLUDES :=

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
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -c

.PHONY: clean $(BOARD_LIB)

$(BOARD_LIB):
	$(MAKE) -C ../lib $(BUILD_TYPE)

clean:
	$(RM) $(EXECUTABLE) $(OBJECTS)
	$(MAKE) -C ../lib clean
