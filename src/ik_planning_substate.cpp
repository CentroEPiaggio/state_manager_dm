#include "ik_planning_substate.h"
#include "../../shared/include/dual_manipulation_shared/databasemapper.h"
#include <dual_manipulation_shared/ik_response.h>
#include <mutex>

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
    results[ik_transition::move]=(plan_executed==0);
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

	if(item.second.command!=cartesian_commands::MOVE) 
	{
            int j=i;
            while(data_.cartesian_plan->at(data_.next_plan+j).second.seq_num==0)
            {
                j++;
                if (data_.cartesian_plan->at(data_.next_plan+j).second.command==cartesian_commands::MOVE)
                {
                    ROS_ERROR("I found two MOVE commands with same sequence number, but there was some different command in the middle!!");
                }
            }
	    ROS_INFO_STREAM("Command was not MOVE: returning to ik_moving_substate");
	    plan_sent = true;
	    return;
	}
	
	ee_pose=item.second.cartesian_task;

	srv.request.command = "plan";
	srv.request.ee_pose.push_back(ee_pose);
	if(i>0)
	{
	    srv.request.ee_name="both_hands";
	    if (std::get<0>(db_mapper.EndEffectors.at(item.first)) != "right_hand")
	    {
		//exchange the poses: they have to be: ee_pose[0] -> left; ee_pose[1] -> right
		std::swap(srv.request.ee_pose.front(),srv.request.ee_pose.back());
	    }
	}
	else srv.request.ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
	srv.request.time = 2;

    } while(data_.cartesian_plan->at(data_.next_plan+i).second.seq_num==0);

    std::unique_lock<std::mutex> lck(plan_executed_mutex);
    plan_executed++;
    sequence_counter++;
    srv.request.seq=sequence_counter;

    std::cout << "data_.next_plan+i = " << data_.next_plan+i << std::endl;
    if(client.call(srv))
    {
        pending_sequence_numbers.insert(sequence_counter);
	ROS_INFO_STREAM("IK Plan Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
    }
    else
    {
	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
            failed=true;
            initialized=false;
    }
    plan_sent=true;
}

bool ik_planning_substate::isComplete()
{
    return (plan_executed==0  || failed);
}

std::string ik_planning_substate::get_type()
{
    return "ik_planning_substate";
}