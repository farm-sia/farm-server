#include "main.hpp"

int main () {
	rpi_gpio = new RpiGpio();
    ws_server = new WsServer(5555);
}
