cmake_minimum_required(VERSION 2.8)

project(dual_manipulation_state_manager)


find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(state_manager src/main.cpp)
target_link_libraries(state_manager ${catkin_LIBRARIES})

install(TARGETS state_manager RUNTIME DESTINATION bin)
