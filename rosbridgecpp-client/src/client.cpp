#include "rosbridge_ws_client.hpp"

#include <unistd.h>

#include <string>
#include <iostream>

using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;

std::string elevatorTopic = "/chassis_elevator_joint_position_controller/command";
std::string feederTopic   = "/elevator_feeder_joint_transmission/command";

void publishMotorMotion(RosbridgeWsClient *rosbridgeClient, std::string motorTopic, double position)
{
	rapidjson::Document d;
	d.SetObject();
	d.AddMember("data", position, d.GetAllocator());
	std::cout << "pub " << motorTopic << " " << position << std::endl;
	rosbridgeClient->publish(motorTopic, d);
}

//  client.on_message = [](shared_ptr<WsClient::Connection> connection, shared_ptr<WsClient::InMessage> in_message) {
//    cout << "Client: Message received: \"" << in_message->string() << "\"" << endl;
//
//    cout << "Client: Sending close connection" << endl;
//    connection->send_close(1000);
//  };

void subTopic(std::shared_ptr<WsClient::Connection> conn, std::shared_ptr<WsClient::InMessage> msg) {
	std::cout << msg->string() << std::endl;
}

int main(int argc, char* argv[])
{
	RosbridgeWsClient rbc("localhost:9090");

	// It seems each subscriber / publisher needs its own client
	// Such declaration will last until the rosbridge server shut down
	rbc.addClient("rosbridge_test");
	rbc.advertise("rosbridge_test", elevatorTopic, "std_msgs/Float64");
	rbc.addClient("rosbridge_test2");
	rbc.advertise("rosbridge_test2", feederTopic, "std_msgs/Float64");
	rbc.addClient("rosbridge_test3");
	rbc.subscribe("rosbridge_test3", feederTopic, subTopic);

	sleep(1);

	double elevatorPos = 100.0;
	while (1) {
		publishMotorMotion(&rbc, elevatorTopic, elevatorPos);
		publishMotorMotion(&rbc, feederTopic, 20.0);
		sleep(1);
	}

	return 0;
}
