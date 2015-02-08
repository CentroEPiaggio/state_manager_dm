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
    initialized = false;
    
    lsub = n.subscribe("/ik_control/left_hand/planning_done",1,&ik_planning_substate::callback_l,this);
    rsub = n.subscribe("/ik_control/right_hand/planning_done",1,&ik_planning_substate::callback_r,this);
    bimanualsub = n.subscribe("/ik_control/both_hands/planning_done",1,&ik_planning_substate::callback_bimanual,this);
}

void ik_planning_substate::callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Plan : %s",str->data.c_str());
    plan_executed = true;
}

void ik_planning_substate::callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
    plan_executed = true;
}

void ik_planning_substate::callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Both Hands IK Plan : %s",str->data.c_str());
    plan_executed = true;
}

std::map< ik_transition, bool > ik_planning_substate::getResults()
{
    std::map< ik_transition, bool > results;
    results[ik_transition::move]=plan_executed;
    plan_executed = false;
    initialized = false;
    return results;
}

void ik_planning_substate::run()
{
    if(!initialized)
    {
	initialized = true;
    }

    if(plan_executed) return;

    geometry_msgs::Pose ee_pose;

    srv.request.ee_pose.clear();

    for(auto item:data_.cartesian_plan->at(data_.seq_num))
    {
	ee_pose=item.second;

	srv.request.command = "plan";
	srv.request.ee_name = item.first;
	srv.request.time = 2;
	srv.request.ee_pose.push_back(ee_pose);

	if (client.call(srv))
	{
	    ROS_INFO_STREAM("IK Plan Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.seq_num);
	}
	else
	{
// 	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	}
    }
}

bool ik_planning_substate::isComplete()
{
    return plan_executed;
}

std::string ik_planning_substate::get_type()
{
    return "ik_planning_substate";
}