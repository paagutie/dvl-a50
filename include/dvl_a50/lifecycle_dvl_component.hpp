#ifndef COMPOSITION__TALKER_COMPONENT_HPP_
#define COMPOSITION__TALKER_COMPONENT_HPP_

#include <chrono>
#include "dvl_a50/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"


#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "dvl_a50/tcpsocket.hpp"

#include <string>
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "dvl_msgs/msg/dvldr.hpp"

//Json Library
#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include <iomanip>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace composition
{

class LifecycleDVL : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit LifecycleDVL(const rclcpp::NodeOptions & options);
  ~LifecycleDVL();
  
  /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  
  
  /// Transition callback for state activating
  /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "active"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  
  
  /// Transition callback for state deactivating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  
  /// Transition callback for state cleaningup
  /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "unconfigured" state or stays
   * in "inactive".
   * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
   * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  
  /// Transition callback for state shutting down
  /**
   * on_shutdown callback is being called when the lifecycle node
   * enters the "shuttingdown" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "finalized" state or stays
   * in its current state.
   * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
   * TRANSITION_CALLBACK_FAILURE transitions to current state
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  
  

protected:
  void on_timer();

private:
  size_t count_;
  int fault = 1; 
  double current_altitude;
  double old_altitude;
  std::string ip_address;
  TCPSocket *tcpSocket;
    
  nlohmann::json json_data;
  nlohmann::json json_position;
    
  std::chrono::steady_clock::time_point first_time_loss;
  std::chrono::steady_clock::time_point first_time_error;
    
    
  // DVL message struct
  dvl_msgs::msg::DVLBeam beam0;
  dvl_msgs::msg::DVLBeam beam1;
  dvl_msgs::msg::DVLBeam beam2;
  dvl_msgs::msg::DVLBeam beam3;
    
  dvl_msgs::msg::DVLDR DVLDeadReckoning;
  dvl_msgs::msg::DVL dvl;
    
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<dvl_msgs::msg::DVL>> dvl_pub_report;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<dvl_msgs::msg::DVLDR>> dvl_pub_pos;
  
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  //rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_a
