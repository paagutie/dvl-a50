cmake_minimum_required(VERSION 3.5)
project(dvl_a50)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)


set(node_plugins "")

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR} 
)
ament_python_install_package(scripts)


# Talker Setup
add_library(dvl_a50_component SHARED src/lifecycle_dvl_component.cpp)
target_compile_definitions(dvl_a50_component PRIVATE "COMPOSITION_BUILDING_DLL")
  
ament_target_dependencies(dvl_a50_component
  "rclcpp_components"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "dvl_msgs")
  
rclcpp_components_register_nodes(dvl_a50_component "composition::LifecycleDVL")
set(node_plugins "${node_plugins}composition::LifecycleDVL;$<TARGET_FILE:dvl_a50_component>\n")



add_executable(${PROJECT_NAME}_sensor src/dvl-sensor.cpp)
ament_target_dependencies(${PROJECT_NAME}_sensor rclcpp 
    "sensor_msgs" 
    "geometry_msgs" 
    "dvl_msgs")


#Install C++ targets
install(TARGETS
  dvl_a50_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(TARGETS
  ${PROJECT_NAME}_sensor
  DESTINATION lib/${PROJECT_NAME}
)

#Install Python scripts
#file(GLOB SCRIPT_FILES scripts/*)
install(PROGRAMS
  scripts/dvl_a50.py
  DESTINATION lib/${PROJECT_NAME}
)

#Install Launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
