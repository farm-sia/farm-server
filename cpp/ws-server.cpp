// Importieren der eigenen Header Dateien
#include "ws-server.hpp"
#include "main.hpp"

// Methode, die eingehende WebSocket Nachrichten verarbeitet
void WsServer::handle_msg(nlohmann::json msg) {
	std::string packet = msg["packet"].get<std::string>(); // Zugreifen auf das Feld "packet", also den Namen/Typ des Packets
	if (packet == "steering") {
		rpi_gpio->update_pwm_pins(msg["l"].get<float>(), msg["r"].get<float>()); // Bei einem "steering" Packet: setze die PWM Motoren Pins auf die Werten, die in der Nachricht mitgesendet wurden
	} else if (packet == "set_goal") {
		ros_node->set_move_goal(msg["x"], msg["y"], 0); // Bei einem "set_goal" Packet wird die set_move_goal Methode mit den Parametern aus dem Packet aufgerufen
	} else {
		std::cout << "[ws] got packet with invalid name" << std::endl; // wenn der Packettyp nicht erkannt wird, wird ein Fehler in die Konsole geschrieben
	}
}

// Methode, die als Callback für eine eingehende WebSocket Nachricht genutzt wird
void WsServer::on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
	std::cout << "[ws] got message: " << msg->get_payload() << std::endl; // Schreibe Nachricht in die Konsole

	if (msg->get_payload() == "stop-listening") { // Stoppe den Server bei entsprechender Nachricht
		ws_server.stop_listening();
		return;
	}
	try { // Versuche die Nachricht zu einem JSON Objekt zu deserialisieren und damit die handle_msg Methode auszuführen
		nlohmann::json msg_json = nlohmann::json::parse(msg->get_payload());
		handle_msg(msg_json);
	} catch (...) { // Bei einem Fehler, schreibe diesen in die Konsole
		std::cout << "[ws] json parse failed at " << msg->get_payload() << std::endl;
	}
}

// Methode, die als Callback für eine neue WebSocket Verbindung benutzt wird
void WsServer::on_open(websocketpp::connection_hdl hdl) {
	m_connections.insert(hdl); // Füge die Verbindung zu der Menge an Verbindungen hinzu
}

// Methode, die als Callback für eine geschlossene WebSocket Verbindung benutzt wird
void WsServer::on_close(websocketpp::connection_hdl hdl) {
	m_connections.erase(hdl); // Lösche die Verbindung aus der Menge an Verbindungen
}

// Hilfsmethode zum Senden von Nachrichten mit leerem Inhalt
void WsServer::send(std::string packet_name) {
	nlohmann::json j;
	send(packet_name, j);
}

// Methode zum Senden von WebSocket Nachrichten
void WsServer::send(std::string packet_name, nlohmann::json content) {
	//std::cout << "sending packet " << packet_name << " with content " << content.dump() << std::endl;
	content["packet"] = packet_name; // Setzen des "packet" Feldes auf den Packetnamen
	for (const websocketpp::connection_hdl& hdl : m_connections) { // Gehe durch alle Verbindungen in der Menge an Verbindungen durch
		ws_server.send(hdl, content.dump(), websocketpp::frame::opcode::text); // und sende die Nachricht durch diese Verbindung an einen Client
	}
}

// Kontruktor der WsServer Klasse
WsServer::WsServer() {
	// Initialisiere AsIO (Asynchrones Input Output)
	ws_server.init_asio();

	// Importieren von Platzhalter Typen
	using websocketpp::lib::placeholders::_1;
	using websocketpp::lib::placeholders::_2;

	ws_server.clear_access_channels(websocketpp::log::alevel::all); // Setzte das Loglevel des WebSocket Servers
	// Registriere die drei oben schon definierten Handler für die entsprechenden Events
	ws_server.set_message_handler(websocketpp::lib::bind(&WsServer::on_message,this,_1,_2));
	ws_server.set_open_handler(bind(&WsServer::on_open,this,::_1));
	ws_server.set_close_handler(bind(&WsServer::on_close,this,::_1));
}

// Methode, die den WebSocket Server startet
void WsServer::run(int port) {
	try {
		ws_server.listen(port); // Starte den WebSocket Server mit dem übergebenen Port

		ws_server.start_accept(); // Startet die Event Loop des WebSocket Servers

		std::cout << "[ws] finished init of server, starting asio..." << std::endl;

		ws_server.run(); // Startet die AsIO Loop des Servers
	} catch (websocketpp::exception const &e) { // Bei einem Fehler während diesem Prozess, schreibe diesen in die Konsole
		std::cout << "[ws] error: " << e.what() << std::endl;
	} catch (...) {
		std::cout << "[ws] other exception" << std::endl;
	}
}
