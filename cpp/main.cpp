#include "ws-server.hpp"
#include "rpi_gpio.hpp"

int main () {
	RpiGpio rpi_gpio = RpiGpio();
    WsServer ws_server = WsServer(5555);
}
