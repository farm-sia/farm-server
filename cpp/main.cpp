#include "main.hpp"
#include <thread>

#include<unistd.h>

RpiGpio* rpi_gpio = nullptr;
WsServer* ws_server = nullptr;
RosNode* ros_node = nullptr;

void start_ws_thread () {
    try {
        ws_server = new WsServer();
        ws_server->run(5555);
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
    }
}

void start_ros_node_thread (int argc, char** argv) {
    ros_node = new RosNode(argc, argv);
}

int main (int argc, char** argv) {
	rpi_gpio = new RpiGpio();

    std::thread ws_thread(start_ws_thread);
    ws_thread.detach();

    std::thread ros_thread(start_ros_node_thread, argc, argv);
    ros_thread.join();
}
