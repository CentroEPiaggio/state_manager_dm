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


find_package(catkin REQUIRED COMPONENTS roscpp dual_manipulation_shared kdl_conversions)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_INSTALL_PREFIX}/include) #this line is to find lemon include files!!
link_directories(${catkin_LIBRARY_DIRS})

SET( STATES_FILES
src/getting_info_state.cpp
src/semantic_planning_state.cpp
src/starting_state.cpp
src/steady_state.cpp
src/ik_control_state.cpp
src/exit_state.cpp
)

SET( SUBSTATES_FILES
src/ik_moving_substate.cpp
src/ik_planning_substate.cpp
src/ik_grasping_substate.cpp
src/ik_checking_grasp_substate.cpp
)

add_executable(state_manager ${STATES_FILES} ${SUBSTATES_FILES} src/debug_utility.cpp src/shared_memory.cpp  src/ros_server.cpp src/main.cpp)
add_dependencies(state_manager dual_manipulation_shared_generate_messages_cpp)
target_link_libraries(state_manager ${catkin_LIBRARIES} lemon)

add_executable(test_draw tests/test_draw_state_machine.cpp)
target_link_libraries(test_draw lemon)

add_executable(test_conversion tests/test_semantic_to_cartesian.cpp src/shared_memory.cpp src/semantic_planning_state.cpp)
target_link_libraries(test_conversion  ${catkin_LIBRARIES} lemon)

install(TARGETS state_manager test_conversion test_draw RUNTIME DESTINATION bin)
