// Importieren der Bibliotheken
#include "main.hpp" // zugehörige Header Datei (Deklariert globale Funktionen und Variablen)
#include <thread> // ermöglicht starten von neuen Threads mithilfe der cpp standard library

#include <unistd.h> // ermöglicht Zugriff auf POSIX API

// Globale Variablen, die den Pointer zu den einzelnen Bestandteilen des Servers speichern, um den Zugriff untereinander zu ermöglichen
RpiGpio* rpi_gpio = nullptr;
WsServer* ws_server = nullptr;
RosNode* ros_node = nullptr;

// Funktion, die den Thread für den WebSocket Server startet
void start_ws_thread () {
    try { // try catch Statement um Fehler zu beachten
        ws_server = new WsServer(); // neue Instanz der WsServer Klasse wird erstellt, der Pointer dazu wird in der globlen Variable gespeichert
        ws_server->run(5555); // der WebSocket Server wird mit dem Port 5555 gestartet
    } catch (websocketpp::exception const& e) {
        std::cout << e.what() << std::endl; // im Falle eines Fehlers wird dieser ausgegeben 
    }
}

// Funktion um die ROS Node zu starteb
void start_ros_node_thread (int argc, char** argv) {
    ros_node = new RosNode(argc, argv); // neue Instanz der Klasse RosNode wird erstellt und der zugehörige Pointer wird in der globalen Variable gespeichert
}

// Die main Funktion wird beim Starten des Programms ausgeführt
int main (int argc, char** argv) {
	rpi_gpio = new RpiGpio(); // eine neue Instanz der Klasse RpiGpio wird erstellt und der Pointer dazu wird in der globalen Varible gespeichert

    std::thread ws_thread(start_ws_thread); // ein neuer Thread wird gestartet und in diesem wird die Funktion start_ws_thread ausgeführt
    ws_thread.detach(); // der neue Thread wird detached, so dass der main Thread nicht blockiert wird

    std::thread ros_thread(start_ros_node_thread, argc, argv); // noch ein Thread wird gestartet und die Funktion start_ros_node_thread wird darin ausgeführt
    ros_thread.join(); // der neue Thread wird gejoined, so dass der main Thread blockiert wird
}
