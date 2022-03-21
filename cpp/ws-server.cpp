#include "ws-server.hpp"

#include <iostream>

// Define a callback to handle incoming messages
void WsServer::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
    std::cout << "on_message called with hdl: " << hdl.lock().get()
              << " and message: " << msg->get_payload()
              << std::endl;

    // check for a special command to instruct the server to stop listening so
    // it can be cleanly exited.
    if (msg->get_payload() == "stop-listening") {
        ws_server.stop_listening();
        return;
    }

    try {
        ws_server.send(hdl, msg->get_payload(), msg->get_opcode());
    } catch (websocketpp::exception const & e) {
        std::cout << "Echo failed because: "
                  << "(" << e.what() << ")" << std::endl;
    }
}

WsServer::WsServer(int _port) {
    port = _port;
    start_ws_server();
}

void WsServer::start_ws_server() {
    // Create a server endpoint
    try {
        // Set logging settings
        //ws_server.set_access_channels(websocketpp::log::alevel::all);
        //ws_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        ws_server.init_asio();

        // Register our message handler
        using websocketpp::lib::placeholders::_1;
        using websocketpp::lib::placeholders::_2;
        ws_server.set_message_handler(websocketpp::lib::bind(&WsServer::on_message,this,_1,_2));

        // Listen on port
        ws_server.listen(port);

        // Start the server accept loop
        ws_server.start_accept();

        // Start the ASIO io_service run loop
        ws_server.run();
    } catch (websocketpp::exception const &e) {
        std::cout << e.what() << std::endl;
    } catch (...) {
        std::cout << "other exception" << std::endl;
    }
}
