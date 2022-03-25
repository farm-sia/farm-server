#pragma once

#include "ws-server.hpp"
#include "rpi_gpio.hpp"
#include "ros_node.hpp"

extern RpiGpio* rpi_gpio;
extern WsServer* ws_server;
extern RosNode* ros_node;
