cmake_minimum_required(VERSION 3.0.2)
project(nturt_state_controller)

find_package(catkin REQUIRED COMPONENTS
        nturt_ros_interface
        roscpp
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES nturt_state_controller
        CATKIN_DEPENDS nturt_ros_interface roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# libraries
# library: nturt_state_controller
add_library(${PROJECT_NAME}
        src/state_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        wiringPi
)

# build this package after "nturt_ros_interface" has already finished generating
add_dependencies(${PROJECT_NAME}
        nturt_ros_interface_generate_messages_cpp)

# applications
# executable: nturt_torque_controller_node
add_executable(${PROJECT_NAME}_node
        app/state_controller_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

# build this package after "nturt_ros_interface" has already finished generating
add_dependencies(${PROJECT_NAME}_node
        nturt_ros_interface_generate_messages_cpp)
