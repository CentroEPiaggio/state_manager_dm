#include "ik_planning_substate.h"
#include "../../shared/include/dual_manipulation_shared/databasemapper.h"

ik_planning_substate::ik_planning_substate(ik_shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    initialized = false;
    plan_sent=false;
    plan_executed = 9999;
    
    lsub = n.subscribe("/ik_control/left_hand/planning_done",1,&ik_planning_substate::callback_l,this);
    rsub = n.subscribe("/ik_control/right_hand/planning_done",1,&ik_planning_substate::callback_r,this);
    bimanualsub = n.subscribe("/ik_control/both_hands/planning_done",1,&ik_planning_substate::callback_bimanual,this);
}

void ik_planning_substate::callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Plan : %s",str->data.c_str());
    if(str->data.c_str()=="done") plan_executed--;
}

void ik_planning_substate::callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
    if(str->data.c_str()=="done") plan_executed--;
}

void ik_planning_substate::callback_bimanual(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Both Hands IK Plan : %s",str->data.c_str());
    if(str->data.c_str()=="done") plan_executed--;
}

std::map< ik_transition, bool > ik_planning_substate::getResults()
{
    std::map< ik_transition, bool > results;
    results[ik_transition::move]=(plan_executed==0);
    initialized = false;
    plan_sent = false;
    return results;
}

void ik_planning_substate::run()
{
    if(!initialized)
    {
	initialized = true;
    }

    if(plan_sent) return;

    plan_executed = 0;

    geometry_msgs::Pose ee_pose;

    srv.request.ee_pose.clear();

    int i=-1;
    
    if(data_.cartesian_plan->size()==0)
    {
	ROS_ERROR("Cartesian plan is empty!!");
	return;
    }
    
    do
    {
        i++;
	auto item = data_.cartesian_plan->at(data_.next_plan+i);

	if(item.second.command!=cartesian_commands::MOVE) continue;

	plan_executed++;

	ee_pose=item.second.cartesian_task;

	srv.request.command = "plan";
	if(i>0) srv.request.ee_name="both";
	else srv.request.ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
	srv.request.time = 2;
	srv.request.ee_pose.push_back(ee_pose);

    } while(data_.cartesian_plan->at(data_.next_plan+i).second.seq_num==0);

    if(client.call(srv))
    {
	ROS_INFO_STREAM("IK Plan Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
    }
    else
    {
// 	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
  
    plan_sent=true;
}

bool ik_planning_substate::isComplete()
{
    return (plan_executed==0);
}

std::string ik_planning_substate::get_type()
{
    return "ik_planning_substate";
}