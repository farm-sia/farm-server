#pragma once

#include <vector>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>
#include <string>
#include <sstream>

class WsServer {
typedef websocketpp::server<websocketpp::config::asio> server;
public:
    WsServer(int port);
	
	std::vector<std::string> get_msg_header(std::stringstream msg);
    void start_ws_server();
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg);
    
    server ws_server;
    int port;
};
