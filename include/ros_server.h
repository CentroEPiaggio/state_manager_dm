#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <ros/ros.h>
#include "state_machine.hpp"
#include "abstract_state.h"
#include "shared_memory.h"

#include <thread>

#include "dual_manipulation_shared/state_manager_service.h"

namespace dual_manipulation{
    namespace state_manager{

class ros_server
{
public:
    ros_server();
    ~ros_server();
    void join();
    shared_memory data;
private:
    void loop();
    std::thread loop_thread;
    
    ros::NodeHandle node;
    bool state_manager_ros_service(dual_manipulation_shared::state_manager_service::Request &req, dual_manipulation_shared::state_manager_service::Response &res);
    ros::ServiceServer service;
    
    state_machine<abstract_state<transition>*,transition_type> sm;
    abstract_state<transition>* current_state;
    std::map<transition,bool> transition_map;
};


    }
}
#endif // ROS_SERVER_H
