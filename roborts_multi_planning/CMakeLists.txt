project(multi_planner)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        roborts_msgs
        )

catkin_package(
        #  INCLUDE_DIRS include
        CATKIN_DEPENDS roborts_msgs roscpp
        #  DEPENDS system_lib
)

## Generate added messages and services with any dependencies listed here


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        include
    ../../../devel/include
)
add_executable(${PROJECT_NAME}_service src/multi_planner_node.cpp
        src/multi_planner.cpp
        )
target_link_libraries(${PROJECT_NAME}_service
        ${catkin_LIBRARIES}
        )
