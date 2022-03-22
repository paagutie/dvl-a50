#ifndef DVL_A50_HPP
#define DVL_A50_HPP

/**
 * dvl-sensor.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       24/11/2021
 */

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "dvl_a50/tcpsocket.hpp"

#include <string>
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "dvl_msgs/msg/dvldr.hpp"

//Json Library
#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include <iomanip>

//from rov_msgs.msg import Control

//#include "sensor_msgs/msg/imu.hpp"
//#include "geometry_msgs/msg/vector3_stamped.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;

namespace dvl_sensor {

class DVL_A50: public rclcpp::Node
{
public:

    uint8_t ready = 0;
    uint8_t error = 0;

    DVL_A50();
    ~DVL_A50();

	//void init();


private:
    int fault = 1; 
    string delimiter = ",";
    double current_altitude;
    double old_altitude;
    std::string ip_address;
    TCPSocket *tcpSocket;
    
    nlohmann::json json_data;
    nlohmann::json json_position;
    
    std::chrono::steady_clock::time_point first_time;
    std::chrono::steady_clock::time_point first_time_error;
    
    
    // DVL message struct
    dvl_msgs::msg::DVLBeam beam0;
    dvl_msgs::msg::DVLBeam beam1;
    dvl_msgs::msg::DVLBeam beam2;
    dvl_msgs::msg::DVLBeam beam3;
    
    dvl_msgs::msg::DVLDR DVLDeadReckoning;
    dvl_msgs::msg::DVL dvl;



    rclcpp::TimerBase::SharedPtr timer_ros;
    rclcpp::Publisher<dvl_msgs::msg::DVL>::SharedPtr dvl_pub_report;
    rclcpp::Publisher<dvl_msgs::msg::DVLDR>::SharedPtr dvl_pub_pos;


    //rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    //rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_; 


    void timerCallback();

    void readData();
    //void topic_callback(const rov_msgs::msg::Control::SharedPtr msg);


};

} // namespace
#endif //DVL_A50_HPP
