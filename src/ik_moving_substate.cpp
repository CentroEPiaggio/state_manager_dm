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
    moving_executed = 9999;
    initialized = false;
    move_sent = false;
    
    lsub = n.subscribe("/ik_control/left_hand/action_done",1,&ik_moving_substate::callback_l,this);
    rsub = n.subscribe("/ik_control/right_hand/action_done",1,&ik_moving_substate::callback_r,this);
    bimanualsub = n.subscribe("/ik_control/both_hands/action_done",1,&ik_moving_substate::callback_bimanual,this);
    lgraspsub = n.subscribe("/ik_control/left_hand/grasp_done",1,&ik_moving_substate::callback_l_grasp,this);
    rgraspsub = n.subscribe("/ik_control/right_hand/grasp_done",1,&ik_moving_substate::callback_r_grasp,this);

    command_map[cartesian_commands::MOVE] = "execute";
    command_map[cartesian_commands::GRASP] = "grasp";
    command_map[cartesian_commands::UNGRASP] = "ungrasp";
}

void ik_moving_substate::callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Exec : %s",str->data.c_str());
    moving_executed--;
}

void ik_moving_substate::callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Exec : %s",str->data.c_str());
    moving_executed--;
}

void ik_moving_substate::callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Both Hands IK Exec : %s",str->data.c_str());
    moving_executed--;
}

void ik_moving_substate::callback_r_grasp(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Exec : %s",str->data.c_str());
    moving_executed--;
}

void ik_moving_substate::callback_l_grasp(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Exec : %s",str->data.c_str());
    moving_executed--;
}

std::map< ik_transition, bool > ik_moving_substate::getResults()
{
    std::map< ik_transition, bool > results;
    if(data_.next_plan == data_.cartesian_plan->size()-1)
    {
	results[ik_transition::done]=(moving_executed==0);
    }
    else
    {
	results[ik_transition::plan]=(moving_executed==0);
    }
    initialized = false;
    move_sent = false;
    return results;
}

bool ik_moving_substate::isComplete()
{  
    if(data_.next_plan == data_.cartesian_plan->size()) moving_executed=0;

    return (moving_executed==0);
}

void ik_moving_substate::run()
{
    if(!initialized)
    {
	initialized = true;
    }
    if(move_sent) return;

    moving_executed = 0;

    geometry_msgs::Pose ee_pose;
    
    int i=-1;
    int move_num=0;
    std::string ee_name;

    do
    {
        i++;
        auto item = data_.cartesian_plan->at(data_.next_plan++);
	ee_pose=item.second.cartesian_task;

	if(item.second.command!=cartesian_commands::MOVE)
	{
		srv.request.ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
		srv.request.command = command_map.at(item.second.command);
		srv.request.time = 0;
		srv.request.ee_pose.push_back(ee_pose);

	    	if (client.call(srv))
		{
		    ROS_INFO_STREAM("IK Exec Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
		}
		else
		{
	// 	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
		}

		moving_executed++;
	}
	else
	{
	        move_num++;
		ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
	}
    }
    while(data_.cartesian_plan->at(data_.next_plan).second.seq_num==0);
    
    if(move_num>0)
    {
        srv.request.command = command_map.at(cartesian_commands::MOVE);
	if(move_num>1) srv.request.ee_name="both";
	else srv.request.ee_name = ee_name;

	if(client.call(srv))
	{
	    ROS_INFO_STREAM("IK Exec Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
	}
	else
	{
    // 	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	}
	
	moving_executed++;
    }
    move_sent=true;
}

std::string ik_moving_substate::get_type()
{
    return "ik_moving_substate";
}