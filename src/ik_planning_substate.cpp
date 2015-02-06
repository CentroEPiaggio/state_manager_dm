#include "ik_planning_substate.h"

ik_planning_substate::ik_planning_substate(ik_shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    plan_executed = false;
    seq=0;
    initialize = false;
}

std::map< ik_transition, bool > ik_planning_substate::getResults()
{
    std::map< ik_transition, bool > results;
    results[ik_transition::move]=plan_executed;
    plan_executed = false;
    return results;
}

void ik_planning_substate::run()
{
    //TODO: call the planner, not the ik_control

    if(plan_executed) return;

    if(!initialize)
    {
	seq=0;
	initialize = true;
    }

    if(seq>=data_.cartesian_plan->size())
    {
	initialize = false;
        return;
    }

    geometry_msgs::Pose ee_pose;

    srv.request.ee_pose.clear();

    for(auto item:data_.cartesian_plan->at(seq))
    {
	ee_pose=item.second;

	srv.request.command = "plan";
	srv.request.ee_name = item.first;
	srv.request.time = 2;
	srv.request.ee_pose.push_back(ee_pose);

	if (client.call(srv))
	{
	    ROS_INFO_STREAM("IK Request accepted: (" << (int)srv.response.ack << ") - seq: "<<seq);
	    plan_executed = true;
	}
	else
	{
	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	}
    }

    seq++;
}

bool ik_planning_substate::isComplete()
{
    return plan_executed;
}

std::string ik_planning_substate::get_type()
{
    return "ik_planning_substate";
}