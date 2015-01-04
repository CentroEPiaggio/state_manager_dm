cmake_minimum_required(VERSION 2.8)

project(dual_manipulation_state_manager)
ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(state_manager src/shared_memory.cpp src/getting_info_state.cpp src/planning_state.cpp src/starting_state.cpp src/ros_server.cpp src/steady_state.cpp src/moving_state.cpp src/main.cpp)
target_link_libraries(state_manager ${catkin_LIBRARIES})

install(TARGETS state_manager RUNTIME DESTINATION bin)
