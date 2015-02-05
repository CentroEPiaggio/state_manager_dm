#include "planning_state.h"

planning_state::planning_state(shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    plan_executed = false;
}

std::map< transition, bool > planning_state::getResults()
{
    std::map< transition, bool > results;
    return results;
}

void planning_state::run()
{
    //TODO: call the planner, not the ik_control

    if(plan_executed) return;

    geometry_msgs::Pose ee_pose;
    data_.get_object_pose(ee_pose);

    srv.request.command = "plan";
    srv.request.ee_name = "right_hand";
    srv.request.time = 2;
    srv.request.ee_pose.push_back(ee_pose);

    if (client.call(srv))
    {
	ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
	plan_executed = true;
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
}

bool planning_state::isComplete()
{
    return plan_executed;
}

std::string planning_state::get_type()
{
    return "planning_state";
}