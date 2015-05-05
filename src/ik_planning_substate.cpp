#include "ik_planning_substate.h"
#include <dual_manipulation_shared/ik_response.h>
#include <mutex>

#define CLASS_NAMESPACE "ik_planning_substate::"

ik_planning_substate::ik_planning_substate(ik_shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc=0;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    initialized = false;
    plan_sent=false;
    plan_executed = 9999;
    sequence_counter=0;

    typedef const dual_manipulation_shared::ik_response::ConstPtr& msg_type;
    lsub = n.subscribe<ik_planning_substate,msg_type>("/ik_control/left_hand/planning_done",1,boost::bind(&ik_planning_substate::callback, this, _1, "Left IK Plan"));
    rsub = n.subscribe<ik_planning_substate,msg_type>("/ik_control/right_hand/planning_done",1,boost::bind(&ik_planning_substate::callback, this, _1, "Right IK Plan"));
    bimanualsub = n.subscribe<ik_planning_substate,msg_type>("/ik_control/both_hands/planning_done",1,boost::bind(&ik_planning_substate::callback, this, _1, "Both hands IK Plan"));

    reset();
}

void ik_planning_substate::reset()
{
    plan_executed = 9999;
    initialized = false;
    plan_sent = false;
    failed=false;
    checking_grasp = false;
    pending_sequence_numbers.clear();
}

void ik_planning_substate::callback(const dual_manipulation_shared::ik_response::ConstPtr& str, std::string type)
{
    std::unique_lock<std::mutex> lck(plan_executed_mutex);
    
    // discard refuse messages (from previous sessions)
    if(plan_executed >= 9999)
      return;
    
    ROS_INFO_STREAM(type.c_str()<<" " << str->data << " | plan_executed = " << plan_executed);
    if(str->data=="done")
    {
        if (pending_sequence_numbers.count(str->seq))
        {
            plan_executed--;
        }
        else
            ROS_WARN_STREAM("There was an error, ik_control (seq. #" << str->seq << ") returned msg.data : " << str->data);
    }
    else
    {
        ROS_WARN_STREAM("There was an error, ik_control (seq. #" << str->seq << ") returned msg.data : " << str->data);
        failed=true;
        initialized=false;
    }
}

std::map< ik_transition, bool > ik_planning_substate::getResults()
{
    std::map< ik_transition, bool > results;
    results[ik_transition::move]=(plan_executed==0 && plan_sent);
    results[ik_transition::check_grasp]=checking_grasp;
    results[ik_transition::fail]=failed;
    
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
    
    bool is_first_plan = true;
    cartesian_commands last_plan;
    std::string planning_ee_name = "";

    srv.request.command = "";
    srv.request.ee_name = "";
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

	if(!commands.is_to_be_planned.count(item.second.command))
	{
	    if((i != 0) || (data_.cartesian_plan->at(data_.next_plan+i).second.seq_num == 0))
	    {
	      ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : I found a command (\'" << item.second.command << "\') not to be planned after " << i << " other(s) to be planned, but those will be IGNORED!!!");
	      failed = true;
	      return;
	    }
	    
	    if(commands.is_to_be_checked.count(item.second.command))
	      checking_grasp = true;
	    else
	    {
	      ROS_INFO_STREAM("Command was not MOVE: returning to ik_moving_substate");
	      plan_sent = true;
	    }
	    return;
	}
	
	if(is_first_plan)
	{
	  is_first_plan = false;
	  last_plan = item.second.command;
	}
	else if((commands.can_follow.count(last_plan) == 0) || (commands.can_follow[last_plan].count(item.second.command) == 0))
	{
	  ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : trying to serially plan for \'" << last_plan << "\' and \'" << item.second.command << "\', but this is NOT allowed!!!");
	  failed = true;
	  return;
	}
	
	// flush old targets
	if(commands.flushing.count(item.second.command))
	{
	  // if the service had to be called (!empty), but I couldn't: ERROR
	  if(!srv.request.command.empty() && !client.call(srv))
	  {
	    ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : failed to call service ik_ros_service " << srv.request.command);
            failed = true;
	    return;
	  }
	  
	  // clear the service request
	  srv.request.command = "";
	  srv.request.ee_name = "";
	  srv.request.ee_pose.clear();
	}
	
	// add new target at the request
	srv.request.command = commands.set_target_command.at(item.second.command);
	
	std::string &ee_name(srv.request.ee_name);
	std::string current_ee(std::get<0>(db_mapper.EndEffectors.at(item.first)));
	//TODO: this code is very specific to our setup!!! make it more general...
	if(!ee_name.empty() && ee_name != current_ee)
	{
	  srv.request.ee_pose.resize(2);
	  
	  // if there was a right_hand target, move it to the second position
	  if(ee_name == "right_hand")
	    std::swap(srv.request.ee_pose.at(1),srv.request.ee_pose.at(0));
	  
	  // insert the new pose in the right place
	  if(current_ee == "left_hand")
	    srv.request.ee_pose.at(0) = item.second.cartesian_task;
	  else if(current_ee == "right_hand")
	    srv.request.ee_pose.at(1) = item.second.cartesian_task;
	  else
	  {
	    ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : current end-effector \'" << current_ee << "\' is NOT supported!");
	    failed = true;
	    return;
	  }
	  
	  ee_name = "both_hands";
	}
	else
	{
	  ee_name = current_ee;
	  srv.request.ee_pose.clear();
	  srv.request.ee_pose.push_back(item.second.cartesian_task);
	}
	
	if(planning_ee_name.empty())
	  planning_ee_name = ee_name;
	else if(planning_ee_name != ee_name)
	  planning_ee_name = "both_hands";
	
	// flush newly set target
	if(commands.flushing.count(item.second.command))
	{
	  if(!client.call(srv))
	  {
	    ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : failed to call service ik_ros_service " << srv.request.command);
            failed = true;
	    return;
	  }
	  
	  // clear the service request
	  srv.request.command = "";
	  srv.request.ee_name = "";
	  srv.request.ee_pose.clear();
	}

    } while((data_.cartesian_plan->at(data_.next_plan+i).second.seq_num==0) && (data_.cartesian_plan->size() > data_.next_plan+i+1));

    // if the service had to be called (!empty), but I couldn't: ERROR
    if(!srv.request.command.empty() && !client.call(srv))
    {
      ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : failed to call service ik_ros_service " << srv.request.command);
      failed = true;
      return;
    }
    
    srv.request.command = commands.plan_command[data_.cartesian_plan->at(data_.next_plan+i).second.command];
    srv.request.ee_name = planning_ee_name;
    
    std::unique_lock<std::mutex> lck(plan_executed_mutex);
    plan_executed++;
    sequence_counter++;
    srv.request.seq=sequence_counter;

    std::cout << "data_.next_plan+i = " << data_.next_plan+i << std::endl;
    if(client.call(srv))
    {
        pending_sequence_numbers.insert(sequence_counter);
	ROS_INFO_STREAM("IK Plan Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
	plan_sent = true;
    }
    else
    {
	    ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : failed to call service ik_ros_service " << srv.request.command);
            failed=true;
            initialized=false;
    }
}

bool ik_planning_substate::isComplete()
{
    return (plan_executed==0  || failed || checking_grasp);
}

std::string ik_planning_substate::get_type()
{
    return "ik_planning_substate";
}