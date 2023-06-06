# dvl-a50 

## Description
This repository contains a plugin for the use of the [Water Linked](https://store.waterlinked.com/product/dvl-a50/) DVL-A50 sensor in ROS2 with the advantage of making use of its new tools such as composition and Lifecycle management.

## Requirements
- [ROS2](https://docs.ros.org/en/galactic/Installation.html) - Galactic or newer
- Ubuntu 20.04 or newer

### Dependencies
- [dvl_msgs](https://github.com/paagutie/dvl_msgs)
- [JSON for Modern C++](https://github.com/nlohmann/json)


## Installation
- Clone the repositories and compile them:
```
$ source /opt/ros/galactic/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/paagutie/dvl_msgs.git
$ git clone --recurse-submodules https://github.com/paagutie/dvl-a50.git
$ cd ..
$ colcon build
```

### Usage
There are three ways to use this package. The first one uses a python script, the second one a node written in c++, for which the external library [Json](https://github.com/nlohmann/json) was used. These versions allow to run a node in a separate process with the benefits of process/fault isolation as well as easier debugging. The latest version uses [Lifecycle](https://index.ros.org/p/lifecycle/github-ros2-demos/) for node management and [composition](https://docs.ros.org/en/foxy/Tutorials/Composition.html) to increase efficiency. Thus it's possible to have more control over the TCP/IP socket configuration needed for communication. 

- First, find and set a static IP address (usually: 192.168.194.90) on your computer. 

#### Python
- To use the python script open a new terminal to run the node:
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run dvl_a50 dvl_a50.py --ros-args -p ip_address:='192.168.194.95'
```

#### C++ 
- To use the C++ node: 
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run dvl_a50 dvl_a50_sensor --ros-args -p dvl_ip_address:='192.168.2.95'
or
$ ros2 launch dvl_a50 dvl_a50.launch.py ip_address:='192.168.194.95'
```
#### Lifecycle management (deprecated)
ROS 2 introduces the concept of managed nodes, also called LifecycleNodes. Managed nodes contain a state machine with a set of predefined states. These states can be changed by invoking a transition id which indicates the succeeding consecutive state.

- The node must first be launched using composition. This allows multiple nodes to be executed in a single process with lower overhead and, optionally, more efficient communication (see [Intra Process Communication](https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html)). The idea of using composition is to be able to make use of its advantages when integrating more than one node, which is the case of a robotic system.

```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 launch dvl_a50 dvl_composition.launch.py ip_address:='192.168.194.95'
```
- Then in a new terminal the initial options can be viewed using Lifecycle. To know the available transitions:
```
$ source /opt/ros/galactic/setup.bash
$ ros2 lifecycle list /dvl_a50_node

- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown
```

- Now it's possible to configure the node to establish communication with the sensor via TCP/IP socket:
```
$ ros2 lifecycle set /dvl_a50_node configure
```
- To know the current transition state use:
```
$ ros2 lifecycle get /dvl_a50_node

inactive [2]
```

#### Available transitions for this node using Lifecycle management
```
$ ros2 lifecycle set /dvl_a50_node activate
$ ros2 lifecycle set /dvl_a50_node deactivate
$ ros2 lifecycle set /dvl_a50_node cleanup
$ ros2 lifecycle set /dvl_a50_node shutdown
```

## ROS2 Topics 
- `/dvl/data`
- `/dvl/position`
- `dvl/config/status`
- `dvl/command/response`
- `dvl/config/command`
#### Lifecycle management
- `/dvl_a50_node/transition_event`

