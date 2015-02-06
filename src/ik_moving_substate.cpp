#include "ik_moving_substate.h"
#include "ros_server.h"

ik_moving_substate::ik_moving_substate(ik_shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    motion_executed = false;
    seq=0;
}


std::map< ik_transition, bool > ik_moving_substate::getResults()
{
    std::map< ik_transition, bool > results;
    results[ik_transition::plan]=motion_executed;
    motion_executed = false;
    return results;
}

bool ik_moving_substate::isComplete()
{
    return motion_executed;
}

void ik_moving_substate::run()
{
    if(motion_executed) return;

    geometry_msgs::Pose ee_pose;

    for(auto item:data_.cartesian_plan->at(seq))
    {
	ee_pose=item.second;

	srv.request.command = "execute";
	srv.request.ee_name = item.first;
	srv.request.time = 2;
	srv.request.ee_pose.push_back(ee_pose);

	if (client.call(srv))
	{
	    ROS_INFO_STREAM("IK Request accepted: (" << (int)srv.response.ack << ") - seq: "<<seq);
	    motion_executed = true;
	}
	else
	{
	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	}
    }
    
    seq++;
}

std::string ik_moving_substate::get_type()
{
    return "ik_moving_substate";
}