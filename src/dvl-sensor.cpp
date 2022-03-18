/**
 * @file   dvl-sensor.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 */

#include "dvl_a50/dvl-sensor.hpp"
using nlohmann::json;

namespace dvl_sensor {


DVL_A50::DVL_A50():
Node("dvl_a50_node"),
current_altitude(0.0),
old_altitude(0.0)
{
    timer_ros = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&DVL_A50::timerCallback, this)); 
    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", 10);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", 10);
    

    //--- TCP/IP SOCKET ---- 
    tcpSocket = new TCPSocket((char*)"192.168.194.95" , 16171);
    
    if(tcpSocket->Create() < 0)
    	std::cout << "Socket creation error" << std::endl;
   
    tcpSocket->SetRcvTimeout(400);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    first_time = std::chrono::steady_clock::now();
    first_time_error = first_time;
    while(fault != 0)
    {
        fault = tcpSocket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            std::cout << error << std::endl;
            std::cout << "Is the sensor on?" << std::endl;
            usleep(2000000);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
    	    double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
    	    if(dt >= 78.5) //Max time to set up
    	    {
    	        fault = -10;
    	        break;
    	    }
        }
        else if(error_code == 103)
        {
            std::cout << error << std::endl;
            std::cout << "No route to host, DVL might be booting?" << std::endl;
            usleep(2000000);
        }
    }  
    
    //std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    //double dt = std::chrono::duration<double>(current_time - first_time).count();
    //first_time = current_time;
    //std::cout << "time: " << dt << std::endl;
    
    if(fault == -10)
    {
        tcpSocket->Close();
        std::cout << "Turn the sensor on and try again!" << std::endl;
    }
    else
        std::cout << "DVL-A50 connected!" << std::endl;
    
    
    usleep(2000);

}

DVL_A50::~DVL_A50() {
    tcpSocket->Close();
    delete tcpSocket;
}


void DVL_A50::readData()
{
    
}


void DVL_A50::timerCallback()
{
    char *tempBuffer = new char[1];

    //tcpSocket->Receive(&tempBuffer[0]);
    std::string str; 
    
    if(fault == 0)
    {
	    while(tempBuffer[0] != '\n')
	    {
		tcpSocket->Receive(tempBuffer);
		str = str + tempBuffer[0];
	    }
			  
	    try
	    {
		json_data = json::parse(str);
		
		if (json_data.contains("altitude")) {
		
		    dvl.header.stamp = Node::now();
		    dvl.header.frame_id = "dvl_A50_report_link";
		    //std::cout << std::setw(4) << json_data << std::endl;
		
		    dvl.time = double(json_data["time"]);
		    dvl.velocity.x = double(json_data["vx"]);
		    dvl.velocity.y = double(json_data["vy"]);
		    dvl.velocity.z = double(json_data["vz"]);
		    dvl.fom = double(json_data["fom"]);
		    current_altitude = double(json_data["altitude"]);
		    dvl.velocity_valid = json_data["velocity_valid"];
		    
		    if(current_altitude >= 0.0 && dvl.velocity_valid)
		    {
		        dvl.altitude = current_altitude;
		        old_altitude = current_altitude;
		    }
		    else
		        dvl.altitude = old_altitude;


		    dvl.status = json_data["status"];
		    dvl.form = json_data["format"];
			
		    beam0.id = json_data["transducers"][0]["id"];
		    beam0.velocity = double(json_data["transducers"][0]["velocity"]);
		    beam0.distance = double(json_data["transducers"][0]["distance"]);
		    beam0.rssi = double(json_data["transducers"][0]["rssi"]);
		    beam0.nsd = double(json_data["transducers"][0]["nsd"]);
		    beam0.valid = json_data["transducers"][0]["beam_valid"];
			
		    beam1.id = json_data["transducers"][1]["id"];
		    beam1.velocity = double(json_data["transducers"][1]["velocity"]);
		    beam1.distance = double(json_data["transducers"][1]["distance"]);
		    beam1.rssi = double(json_data["transducers"][1]["rssi"]);
		    beam1.nsd = double(json_data["transducers"][1]["nsd"]);
		    beam1.valid = json_data["transducers"][1]["beam_valid"];
			
		    beam2.id = json_data["transducers"][2]["id"];
		    beam2.velocity = double(json_data["transducers"][2]["velocity"]);
		    beam2.distance = double(json_data["transducers"][2]["distance"]);
		    beam2.rssi = double(json_data["transducers"][2]["rssi"]);
		    beam2.nsd = double(json_data["transducers"][2]["nsd"]);
		    beam2.valid = json_data["transducers"][2]["beam_valid"];
			
		    beam3.id = json_data["transducers"][3]["id"];
		    beam3.velocity = double(json_data["transducers"][3]["velocity"]);
		    beam3.distance = double(json_data["transducers"][3]["distance"]);
		    beam3.rssi = double(json_data["transducers"][3]["rssi"]);
		    beam3.nsd = double(json_data["transducers"][3]["nsd"]);
		    beam3.valid = json_data["transducers"][3]["beam_valid"];
		    
		    dvl.beams = {beam0, beam1, beam2, beam3};
		    dvl_pub_report->publish(dvl);
		    

		}
		// find an entry
		if (json_data.contains("pitch")) {
		    //std::cout << std::setw(4) << json_data << std::endl;
		    DVLDeadReckoning.header.stamp = Node::now();
		    DVLDeadReckoning.header.frame_id = "dvl_A50_position_link";
		    DVLDeadReckoning.time = double(json_data["ts"]);
		    DVLDeadReckoning.position.x = double(json_data["x"]);
		    DVLDeadReckoning.position.y = double(json_data["y"]);
		    DVLDeadReckoning.position.z = double(json_data["z"]);
		    DVLDeadReckoning.pos_std = double(json_data["std"]);
		    DVLDeadReckoning.roll = double(json_data["roll"]);
		    DVLDeadReckoning.pitch = double(json_data["pitch"]);
		    DVLDeadReckoning.yaw = double(json_data["yaw"]);
		    DVLDeadReckoning.type = json_data["type"];
		    DVLDeadReckoning.status = json_data["status"];
		    DVLDeadReckoning.format = json_data["format"];
		    dvl_pub_pos->publish(DVLDeadReckoning);
		}

	     
	    	
	    }
	    catch(std::exception& e)
	    {
		std::cout << "Exception: " << e.what() << std::endl;
	    } 
	    
    } 
}

}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dvl_sensor::DVL_A50>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
