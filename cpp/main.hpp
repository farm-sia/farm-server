#pragma once // schließt eine doppelte Inklusion dieser Datei aus

// Importieren aller nötigen Dateien, so dass aus der main.cpp Datei auf alle nötigen Funktionen und Klassen zugegriffen werden kann
#include "ws-server.hpp"
#include "rpi_gpio.hpp"
#include "ros_node.hpp"

// Deklarieren der globalen Variablen um die Pointer zu den Bestandteilen des Servers zu speichern
extern RpiGpio* rpi_gpio;
extern WsServer* ws_server;
extern RosNode* ros_node;
