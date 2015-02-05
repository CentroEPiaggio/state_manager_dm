#include "moving_state.h"
#include "ros_server.h"

moving_state::moving_state(shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    motion_executed = false;
}


std::map< transition, bool > moving_state::getResults()
{
    std::map< transition, bool > results;
    return results;
}

bool moving_state::isComplete()
{
    return motion_executed;
}

void moving_state::run()
{
    if(motion_executed) return;

    geometry_msgs::Pose ee_pose;
    data_.get_object_pose(ee_pose);

    srv.request.command = "execute";
    srv.request.ee_name = "right_hand";
    srv.request.time = 2;
    srv.request.ee_pose.push_back(ee_pose);

    if (client.call(srv))
    {
	ROS_INFO("Execute Request accepted: %d", (int)srv.response.ack);
	motion_executed = true;
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
}

std::string moving_state::get_type()
{
    return "moving_state";
}