#ifndef GETTING_INFO_STATE_H
#define GETTING_INFO_STATE_H

#include <abstract_state.h>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

class getting_info_state : public abstract_state<transition>
{
public:
    getting_info_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
private:
    shared_memory& data_;
    bool fresh_data;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::ServiceClient planner_client, gui_target_client, scene_object_client;

    void get_start_position_from_vision(visualization_msgs::Marker& source_marker);
    void get_target_position_from_user(visualization_msgs::Marker& target_marker);
};

#endif // GETTING_INFO_STATE_H
