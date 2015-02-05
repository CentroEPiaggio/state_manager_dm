#include "getting_info_state.h"

getting_info_state::getting_info_state(shared_memory& data):data_(data)
{
    fresh_data = false;
}

std::map< transition, bool > getting_info_state::getResults()
{
    std::map< transition, bool > results;
    results[transition::got_info]=fresh_data;
    return results;
}

void getting_info_state::run()
{
    //NOTE: just a test

    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.2;
    object_pose.position.y = -0.2+1.129;
    object_pose.position.z = -0.2+1.106;
    object_pose.orientation.w = 1;
    object_pose.orientation.x = 0;
    object_pose.orientation.y = 0;
    object_pose.orientation.z = 0;

    data_.set_object_pose(object_pose);

    fresh_data = true;
}

bool getting_info_state::isComplete()
{
    return fresh_data;
}

std::string getting_info_state::get_type()
{
    return "getting_info_state";
}