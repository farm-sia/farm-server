#include "main.hpp"

RpiGpio* rpi_gpio = nullptr;
WsServer* ws_server = nullptr;
RosNode* ros_node = nullptr;

int main (int argc, char** argv) {
	rpi_gpio = new RpiGpio();
    ros_node = new RosNode(argc, argv);
    ws_server = new WsServer(5555);
}
