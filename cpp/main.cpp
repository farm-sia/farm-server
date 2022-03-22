#include "main.hpp"

RpiGpio* rpi_gpio = nullptr;
WsServer* ws_server = nullptr;

int main () {
	rpi_gpio = new RpiGpio();
    ws_server = new WsServer(5555);
}
