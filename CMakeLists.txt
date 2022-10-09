cmake_minimum_required(VERSION 3.0.2)
project(nturt_state_controller)

find_package(catkin REQUIRED COMPONENTS
        nturt_ros_interface
        roscpp
        std_msgs
)
find_package(wiringPi)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES nturt_state_controller
        CATKIN_DEPENDS nturt_ros_interface roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/state_controller.cpp
        app/state_controller_node.cpp
)

# to avoid error message not finding wiringPi
if(${wiringPi_FOUND})
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        wiringPi
)
else()
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
endif()
