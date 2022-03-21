#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>


class WsServer {
typedef websocketpp::server<websocketpp::config::asio> server;
public:
    WsServer(int port);

    void start_ws_server();
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);
    
    server ws_server;
    int port;
};
