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
	} else if (packet == "set_goal") {
		ros_node->set_move_goal(msg["x"], msg["y"], 0);
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
}

void WsServer::on_open(websocketpp::connection_hdl hdl) {
    m_connections.insert(hdl);
}

void WsServer::on_close(websocketpp::connection_hdl hdl) {
    m_connections.erase(hdl);
}

void WsServer::send(std::string packet_name, nlohmann::json content) {
    //std::cout << "sending packet " << packet_name << " with content " << content.dump() << std::endl;
    content["packet"] = packet_name; 
    for (const websocketpp::connection_hdl& hdl : m_connections) {
        ws_server.send(hdl, content.dump(), websocketpp::frame::opcode::text);
    }
}

WsServer::WsServer() {
    // Initialize Asio
    ws_server.init_asio();

    // Register our message handler
    using websocketpp::lib::placeholders::_1;
    using websocketpp::lib::placeholders::_2;

    ws_server.clear_access_channels(websocketpp::log::alevel::all);
    ws_server.set_message_handler(websocketpp::lib::bind(&WsServer::on_message,this,_1,_2));
    ws_server.set_open_handler(bind(&WsServer::on_open,this,::_1));
    ws_server.set_close_handler(bind(&WsServer::on_close,this,::_1));
}

void WsServer::run(int port) {
    // Create a server endpoint
    try {
        // Listen on port
        ws_server.listen(port);

        // Start the server accept loop
        ws_server.start_accept();

        std::cout << "[ws] finished init of server, starting asio..." << std::endl;

        // Start the ASIO io_service run loop
        ws_server.run();
    } catch (websocketpp::exception const &e) {
        std::cout << "[ws] error: " << e.what() << std::endl;
    } catch (...) {
        std::cout << "[ws] other exception" << std::endl;
    }
}
