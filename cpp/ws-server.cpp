#include "ws-server.hpp"
#include "main.hpp"

void WsServer::handle_msg(nlohmann::json msg) {
	std::string packet = msg["packet"].get<std::string>();
	if (packet == "steering") {
		if (msg.count("direction") > 0) {
			rpi_gpio->direction = msg["direction"].get<std::string>().front();
			std::cout << "set direction to " << msg["direction"].get<std::string>().front() << std::endl;
		}
		if (msg.count("speed") > 0) {
			rpi_gpio->speed = msg["speed"].get<float>();
			std::cout << "set speed to " << (msg["speed"].get<float>()) << std::endl;
		}
		rpi_gpio->update_pwm_pins();
	} else {
		std::cout << "[ws] got packet with invalid name" << std::endl;
	}
}

// Define a callback to handle incoming messages
void WsServer::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
    std::cout << "[ws] got message: " << msg->get_payload() << std::endl;

    // check for a special command to instruct the server to stop listening so
    // it can be cleanly exited.
    if (msg->get_payload() == "stop-listening") {
        ws_server.stop_listening();
        return;
    }
    try {
        nlohmann::json msg_json = nlohmann::json::parse(msg->get_payload());
        handle_msg(msg_json);
    } catch (...) {
        std::cout << "[ws] json parse failed at " << msg->get_payload() << std::endl;
    }
    /*try {
        ws_server.send(hdl, msg->get_payload(), msg->get_opcode());
    } catch (websocketpp::exception const & e) {
        std::cout << "Echo failed because: "
                  << "(" << e.what() << ")" << std::endl;
    }*/
}

WsServer::WsServer(int _port) {
    port = _port;
    start_ws_server();
}

void WsServer::start_ws_server() {
    // Create a server endpoint
    try {
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
        std::cout << "[ws] error: " << e.what() << std::endl;
    } catch (...) {
        std::cout << "[ws] other exception" << std::endl;
    }
}
