#pragma once

#include <vector>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "json.hpp"

class WsServer {
typedef websocketpp::server<websocketpp::config::asio> server;
public:
    WsServer();
    void run(int port);
    void handle_msg(nlohmann::json msg);
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);
    void on_open(websocketpp::connection_hdl hdl);
    void send(std::string packet_name, std::string content);

    server ws_server;
    int port;
    websocketpp::connection_hdl open_connection;
};
