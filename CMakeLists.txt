cmake_minimum_required(VERSION 3.0.2)
project(nturt_state_controller)

find_package(catkin REQUIRED COMPONENTS
        can_msgs
        nturt_can_parser
        roscpp
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES nturt_state_controller
        CATKIN_DEPENDS can_msgs nturt_can_parser roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/state_controller_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
      ${catkin_LIBRARIES}
      wiringPi
)
