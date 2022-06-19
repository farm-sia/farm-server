#pragma once // Sorgt dafür, dass diese Datei nur einmal importiert wird

// importieren der benötigten Dateien und Bibliotheken
#include <vector> // Vektoren aus der cpp Standard Library
#include <websocketpp/config/asio_no_tls.hpp> // zwei Teile der websocketpp Bibliothek für den WebSocket Server
#include <websocketpp/server.hpp>
#include "json.hpp" // Funktionen um JSON zu (de-)serialisieren
#include <set> // Sets aus der cpp Standard Library

// Deklarieren der WsServer Klasse
class WsServer {
// Typedefinition für den WebSocket server typ
typedef websocketpp::server<websocketpp::config::asio> server;
public:
	// Prototypen der Methoden, die in der ws-server.cpp definiert werden
    WsServer();
    void run(int port);
    void handle_msg(nlohmann::json msg);
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);
    void on_open(websocketpp::connection_hdl hdl);
    void on_close(websocketpp::connection_hdl hdl);
    void send(std::string packet_name, nlohmann::json content);
    void send(std::string packet_name);

    server ws_server;
    int port;
	// Typedefinition für die Menge an Verbindungen
    typedef std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> con_list;
    con_list m_connections;
};
