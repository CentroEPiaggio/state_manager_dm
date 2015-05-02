#include "ik_moving_substate.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include <kdl_conversions/kdl_msg.h>

#define OBJ_GRASP_FACTOR 1000

ik_moving_substate::ik_moving_substate(ik_shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");

    typedef const dual_manipulation_shared::ik_response::ConstPtr& msg_type;
    lsub = n.subscribe<ik_moving_substate,msg_type>("/ik_control/left_hand/action_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Left IK Exec"));
    rsub = n.subscribe<ik_moving_substate,msg_type>("/ik_control/right_hand/action_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Right IK Exec"));
    bimanualsub = n.subscribe<ik_moving_substate,msg_type>("/ik_control/both_hands/action_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Both hands IK Exec"));
    lgraspsub = n.subscribe<ik_moving_substate,msg_type>("/ik_control/left_hand/grasp_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Left IK Grasp"));
    rgraspsub = n.subscribe<ik_moving_substate,msg_type>("/ik_control/right_hand/grasp_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Right IK Grasp"));

    command_map[cartesian_commands::MOVE] = "execute";
    command_map[cartesian_commands::MOVE_NO_COLLISION_CHECK] = "execute";
    command_map[cartesian_commands::GRASP] = "grasp";
    command_map[cartesian_commands::UNGRASP] = "ungrasp";
    command_map[cartesian_commands::HOME] = "home";
    reset();
}

void ik_moving_substate::reset()
{
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    moving_executed = 9999;
    initialized = false;
    move_sent = false;
    failed=false;
    pending_sequence_numbers.clear();
}

void ik_moving_substate::callback(const dual_manipulation_shared::ik_response::ConstPtr& str, std::string type)
{
    std::unique_lock<std::mutex> lck(moving_executed_mutex);

    // discard refuse messages (from previous sessions)
    if(moving_executed >= 9999)
      return;
    
    
    ROS_INFO_STREAM(type.c_str()<<" " << str->data << " | moving_executed = " << moving_executed);
    if(str->data=="done")
    {
        if (pending_sequence_numbers.count(str->seq))
        {
            moving_executed--;
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

std::map< ik_transition, bool > ik_moving_substate::getResults()
{
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    std::map< ik_transition, bool > results;
    if(data_.next_plan == data_.cartesian_plan->size())
    {
	results[ik_transition::done]=(moving_executed==0);
    }
    else
    {
	results[ik_transition::plan]=(moving_executed==0);
    }
    results[ik_transition::fail]=failed;
    return results;
}

bool ik_moving_substate::isComplete()
{  
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    if(data_.next_plan == data_.cartesian_plan->size()+1) moving_executed=0;

    return (moving_executed==0 || failed);
}

void ik_moving_substate::run()
{
    if(!initialized)
    {
	initialized = true;
    }
    if(move_sent) return;
    {
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    moving_executed = 0;
    }
//     geometry_msgs::Pose ee_pose;
    
    int i=-1;
    int move_num=0;
    std::string ee_name;
    
    if(data_.cartesian_plan->size()==0)
    {
	ROS_ERROR("Cartesian plan is empty!!");
        failed=true;
	return;
    }

    do
    {
        i++;
        auto item = data_.cartesian_plan->at(data_.next_plan+i);
// 	ee_pose=item.second.cartesian_task;

	if((item.second.command!=cartesian_commands::MOVE) && (item.second.command!=cartesian_commands::MOVE_NO_COLLISION_CHECK))
	{
		if(item.second.command==cartesian_commands::GRASP)
		{
			int grasp = (int)item.second.ee_grasp_id;
			grasp = grasp % OBJ_GRASP_FACTOR;
			if(!deserialize_ik(srv.request,"object" + std::to_string((int)*data_.obj_id) + "/grasp" + std::to_string(grasp)))
			{
			    ROS_ERROR_STREAM("Failed to deserialize object" + std::to_string((int)*data_.obj_id) + "/grasp" + std::to_string(grasp));
			}
			else
			{
				srv.request.attObject.object.id = *data_.object_name;
				srv.request.object_db_id = (int)*data_.obj_id;
				srv.request.ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
				srv.request.command = command_map.at(item.second.command);
			
				// change frame of reference of the grasp trajectory to the current object frame
				change_frame_to_pose_vector(item.second.cartesian_task,srv.request.ee_pose);
                                std::unique_lock<std::mutex> lck(moving_executed_mutex);
                                srv.request.seq=sequence_counter;

				if (client.call(srv))
				{
				    ROS_INFO_STREAM("IK Grasp Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
                                    pending_sequence_numbers.insert(sequence_counter);
                                }
				else
				{
                                    failed=true;
                                    initialized=false;
				    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
				}
                                sequence_counter++;
                                moving_executed++;
			}
		}
		if(item.second.command==cartesian_commands::UNGRASP) //same ad graso, just changing the ee_pose order
		{
			int grasp = (int)item.second.ee_grasp_id;
			grasp = grasp % OBJ_GRASP_FACTOR;
			if(!deserialize_ik(srv.request,"object" + std::to_string((int)*data_.obj_id) + "/grasp" + std::to_string(grasp)))
			{
			    ROS_ERROR_STREAM("Failed to deserialize object" + std::to_string((int)*data_.obj_id) + "/grasp" + std::to_string(grasp));
			}
			else
			{
				srv.request.attObject.object.id = *data_.object_name;
				srv.request.object_db_id = (int)*data_.obj_id;
				srv.request.ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
				srv.request.command = command_map.at(item.second.command);
                                std::unique_lock<std::mutex> lck(moving_executed_mutex);
                                srv.request.seq=sequence_counter;

                                // change frame of reference of the grasp trajectory to the current object frame
				change_frame_to_pose_vector(item.second.cartesian_task,srv.request.ee_pose);
				// invert the order to generate an ungrasp
				std::reverse(srv.request.ee_pose.begin(),srv.request.ee_pose.end());
				std::reverse(srv.request.grasp_trajectory.points.begin(),srv.request.grasp_trajectory.points.end());

				if (client.call(srv))
				{
				    ROS_INFO_STREAM("IK Grasp Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
                                    pending_sequence_numbers.insert(sequence_counter);
				}
				else
				{
                                    failed=true;
                                    initialized=false;
				    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
				}
				moving_executed++;
                                sequence_counter++;
			}
		}
		if(item.second.command==cartesian_commands::HOME)
                {
                    srv.request.ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
                    srv.request.command = command_map.at(item.second.command);
                    std::unique_lock<std::mutex> lck(moving_executed_mutex);
                    srv.request.seq=sequence_counter;

                    if (client.call(srv))
                    {
                        ROS_INFO_STREAM("IK Home Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
                        pending_sequence_numbers.insert(sequence_counter);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
                        initialized=false;
                        failed=true;
                    }
                    moving_executed++;
                    sequence_counter++;
                }
	}
	else
	{
	        move_num++;
		ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
	}
    }
    while(data_.cartesian_plan->at(data_.next_plan+i).second.seq_num==0);
    
    if(move_num>0)
    {
        srv.request.command = command_map.at(cartesian_commands::MOVE);
	if(move_num>1) srv.request.ee_name="both_hands";
	else srv.request.ee_name = ee_name;
        std::unique_lock<std::mutex> lck(moving_executed_mutex);
        srv.request.seq=sequence_counter;
        
	if(client.call(srv))
	{
	    ROS_INFO_STREAM("IK Exec Request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
            pending_sequence_numbers.insert(sequence_counter);
	}
	else
	{
    	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
            initialized=false;
            failed=true;
	}
	moving_executed++;
        sequence_counter++;
    }
    
    // add all the checked phases of the plan to data_.next_plan
    data_.next_plan += i+1;
    
    move_sent=true;
}

std::string ik_moving_substate::get_type()
{
    return "ik_moving_substate";
}

void ik_moving_substate::change_frame_to_pose_vector(geometry_msgs::Pose object_pose_msg, std::vector< geometry_msgs::Pose >& ee_pose)
{
    KDL::Frame object_frame,ee_single_frame;
    tf::poseMsgToKDL(object_pose_msg,object_frame);
    for(int i=0; i<ee_pose.size(); ++i)
    {
	tf::poseMsgToKDL(ee_pose.at(i),ee_single_frame);
	tf::poseKDLToMsg(object_frame*ee_single_frame,ee_pose.at(i));
    }
}