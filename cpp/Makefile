LDFLAGS += -lpthread -lwiringPi
OBJECTS := main.o ws-server.o rpi_gpio.o
CXXFLAGS ?= -Wall -Wextra
CXXFLAGS += -std=c++11
CXX ?= g++
INCLUDES :=

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
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -c

.PHONY: clean $(BOARD_LIB)

$(BOARD_LIB):
	$(MAKE) -C ../lib $(BUILD_TYPE)

clean:
	$(RM) $(EXECUTABLE) $(OBJECTS)
	$(MAKE) -C ../lib clean
