#include "ik_planning_substate.h"

#define CLASS_NAMESPACE "ik_planning_substate::"
#define CLASS_LOGNAME "ik_planning_substate"

#define DEBUG 1 // if 1, print some more information

ik_planning_substate::ik_planning_substate(ik_shared_memory& data):data_(data),db_mapper(data.db_mapper)
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
    
    plan_sub = n.subscribe<ik_planning_substate,msg_type>("ik_control/planning_done",1,boost::bind(&ik_planning_substate::callback, this, _1, "Just to use a boost::bind"));
    
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
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << str->group_name.c_str()<<" " << str->data << " | plan_executed = " << plan_executed);
    if(str->data=="done")
    {
        if (pending_sequence_numbers.count(str->seq))
        {
            plan_executed--;
        }
        else
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : There was an error, ik_control (seq. #" << str->seq << ") returned msg.data : " << str->data);
    }
    else
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : There was an error, ik_control (seq. #" << str->seq << ") returned msg.data : " << str->data);
        failed=true;
        initialized=false;
    }
}

std::map< ik_transition, bool > ik_planning_substate::getResults()
{
    std::map< ik_transition, bool > results;
    if(data_.need_replan.load())
        results[ik_transition::fail] = true;
    else if(failed || data_.move_failed.load())
        results[ik_transition::fail] = true;
    else
    {
        results[ik_transition::move]=(plan_executed==0 && plan_sent);
        results[ik_transition::check_grasp]=checking_grasp;
    }
    
    return results;
}

void ik_planning_substate::run()
{
    if(!initialized)
    {
        initialized = true;
    }
    
    if(plan_sent || checking_grasp)
    {
        // sleep 5ms to allow for other stuff to go on
        usleep(5000);
        return;
    }
    
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
        
        // add new target at the request
        srv.request.command = commands.set_target_command.at(item.second.command);
        
        //NOTE: targets have to be set separately, the sequence of poses in the vector would otherwise be unknown
        srv.request.ee_name = db_mapper.EndEffectors.at(item.first).name;
        srv.request.ee_pose.clear();
        srv.request.ee_pose.push_back(item.second.cartesian_task);
        srv.request.attObject.object.id = *data_.object_name;
        srv.request.object_db_id = (int)*data_.obj_id;
        // sending information on s and t poses, source grasp and if there is a TILT in current plan to ik_control through ik_service
        srv.request.obj_poses.clear();
        srv.request.obj_poses.push_back(sh_data->source_position);
        srv.request.obj_poses.push_back(sh_data->target_position);
        srv.request.current_source_grasp_id = sh_data->source_grasp;
        // Only when current cartesian command is TILT the bool tilt_only_now will be true -> used temporarly in slidingCapability to manage tilting (not added tilting capability yet)
        srv.request.current_transition_tilting = item.second.tilt_only_now;
        
#if DEBUG
        std::cout<<"Starting ee_pose is: "<<std::endl<<srv.request.obj_poses[0].position<<std::endl;
        std::cout<<"Arriving ee_pose is: "<<std::endl<<srv.request.obj_poses[1].position<<std::endl;

        if(srv.request.current_transition_tilting){
            std::cout << "current_transition_tilting is TRUE" << std::endl;
            std::cout << "current cartesian command is: " << srv.request.command << std::endl;
        }
        else{
            std::cout << "current_transition_tilting is FALSE" << std::endl;
            std::cout << "current cartesian command is: " << srv.request.command << std::endl;
        }
#endif

        if(planning_ee_name.empty())
            planning_ee_name = srv.request.ee_name;
        else if(planning_ee_name != srv.request.ee_name)
            // if more than a single group has been considered, always plan for the full robot group
            planning_ee_name = "full_robot";
        
        // flush newly set target - NOTE that all targets need to be set when they are received, cause composition of multiple ones is not allowed
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
        srv.request.attObject.object.id.clear();
        
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
    srv.request.attObject.object.id = *data_.object_name;
    srv.request.object_db_id = (int)*data_.obj_id;
    
    std::unique_lock<std::mutex> lck(plan_executed_mutex);
    plan_executed++;
    sequence_counter++;
    srv.request.seq=sequence_counter;
    
    if(client.call(srv))
    {
        pending_sequence_numbers.insert(sequence_counter);
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << client.getService() << " " << srv.request.command << " request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan << " | data_.next_plan+i = " << data_.next_plan+i);
        plan_sent = true;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : failed to call service " << client.getService() << " " << srv.request.command << " | data_.next_plan+i = " << data_.next_plan+i);
        failed=true;
        initialized=false;
    }
}

bool ik_planning_substate::isComplete()
{
    // I am complete if the robot is not moving AND I've done what I needed
    return (!data_.robot_moving.load() && (plan_executed==0  || failed || checking_grasp));
}

std::string ik_planning_substate::get_type()
{
    return "ik_planning_substate";
}