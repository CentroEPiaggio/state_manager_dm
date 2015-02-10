#include "ik_moving_substate.h"

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
    initialized = false;
    
    lsub = n.subscribe("/ik_control/left_hand/action_done",1,&ik_moving_substate::callback_l,this);
    rsub = n.subscribe("/ik_control/right_hand/action_done",1,&ik_moving_substate::callback_r,this);
    bimanualsub = n.subscribe("/ik_control/both_hands/action_done",1,&ik_moving_substate::callback_bimanual,this);
}

void ik_moving_substate::callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Exec : %s",str->data.c_str());
    motion_executed = true;
}

void ik_moving_substate::callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Exec : %s",str->data.c_str());
    motion_executed = true;
}

void ik_moving_substate::callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Both Hands IK Exec : %s",str->data.c_str());
    motion_executed = true;
}

std::map< ik_transition, bool > ik_moving_substate::getResults()
{
    std::map< ik_transition, bool > results;
    if(data_.seq_num == data_.cartesian_plan->size()-1)
    {
	results[ik_transition::done]=motion_executed;
    }
    else
    {
	results[ik_transition::plan]=motion_executed;
	data_.seq_num++;
    }
    motion_executed = false;
    initialized = false;
    return results;
}

bool ik_moving_substate::isComplete()
{
    if(data_.seq_num == data_.cartesian_plan->size()) motion_executed=true;
    return motion_executed;
}

void ik_moving_substate::run()
{
    if(!initialized)
    {
	initialized = true;
    }
    if(motion_executed) return;

    geometry_msgs::Pose ee_pose;

    auto item=data_.cartesian_plan->at(data_.seq_num);
    {
	ee_pose=item.second.cartesian_task;

	srv.request.command = "execute";
	srv.request.ee_name = item.first;
	srv.request.time = 0;
	srv.request.ee_pose.push_back(ee_pose);

	if (client.call(srv))
	{
	    ROS_INFO_STREAM("IK Exec Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.seq_num);
	}
	else
	{
// 	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	}
    }
}

std::string ik_moving_substate::get_type()
{
    return "ik_moving_substate";
}