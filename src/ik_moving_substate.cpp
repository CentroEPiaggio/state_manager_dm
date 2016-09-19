#include "ik_moving_substate.h"

#define PARALLELIZE_PLANNING false

#define CLASS_NAMESPACE "ik_moving_substate::"

ik_moving_substate::ik_moving_substate(ik_shared_memory& data):data_(data),db_mapper(data.db_mapper)
{
    if( !ros::isInitialized() )
    {
        int argc;
        char** argv;
        ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    
    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    
    typedef const dual_manipulation_shared::ik_response::ConstPtr& msg_type;
    
    exe_sub = n.subscribe<ik_moving_substate,msg_type>("ik_control/action_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Execution"));
    grasp_sub = n.subscribe<ik_moving_substate,msg_type>("ik_control/grasp_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Grasp"));
    ungrasp_sub = n.subscribe<ik_moving_substate,msg_type>("ik_control/ungrasp_done",1,boost::bind(&ik_moving_substate::callback, this, _1, "Ungrasp"));
    
    n.param("dual_manipulation_parameters/parallelize_plan_execute",parallelize_planning,PARALLELIZE_PLANNING);
    ROS_INFO_STREAM(CLASS_NAMESPACE << " : initialized and " << (parallelize_planning?"":"NOT ") << "parallelizing planning and execution!");
    
    reset();
}

void ik_moving_substate::reset()
{
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    moving_executed = 9999;
    initialized = false;
    move_sent = false;
    failed=false;
    alone_execution=false;
    pending_sequence_numbers.clear();
}

void ik_moving_substate::callback(const dual_manipulation_shared::ik_response::ConstPtr& str, std::string type)
{
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    
    // discard refuse messages (from previous sessions)
    if(moving_executed >= 9999)
        return;
    
    ROS_INFO_STREAM(type.c_str() << ": " << str->group_name.c_str() << " " << str->data << " | moving_executed = " << moving_executed);
    if(str->data=="done")
    {
        if (pending_sequence_numbers.count(str->seq))
        {
            moving_executed--;
            if(moving_executed == 0)
                data_.robot_moving.store(false);
        }
        else
            ROS_WARN_STREAM("There was an error, ik_control (seq. #" << str->seq << ") returned msg.data : " << str->data);
    }
    else
    {
        ROS_WARN_STREAM("There was an error, ik_control (seq. #" << str->seq << ") returned msg.data : " << str->data);
        failed=true;
        initialized=false;
        data_.robot_moving.store(false);
        data_.move_failed.store(true && parallelize_planning);
    }
}

std::map< ik_transition, bool > ik_moving_substate::getResults()
{
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    std::map< ik_transition, bool > results;
    if(data_.need_replan.load())
    {
        results[ik_transition::need_replan]=true;
    }
    else if(failed)
    {
        results[ik_transition::fail]=failed;
    }
    else if(data_.next_plan == data_.cartesian_plan->size())
    {
        results[ik_transition::done]=(moving_executed==0);
    }
    else
    {
        // I return to planning if I finished moving OR I can parallelize
        results[ik_transition::plan]=(moving_executed==0 || (parallelize_planning && move_sent));
    }
    return results;
}

bool ik_moving_substate::isComplete()
{  
    std::unique_lock<std::mutex> lck(moving_executed_mutex);
    if(data_.next_plan == data_.cartesian_plan->size()+1) moving_executed=0;
    
    // I can return if I executed the movement, I failed, or I sent the movement AND it's not the last one! (this only if I can parallelize, and NOT for grasping WPs..!)
    return (data_.need_replan.load() || moving_executed==0 || failed || (parallelize_planning && move_sent && !alone_execution && data_.next_plan < data_.cartesian_plan->size()));
}

void ik_moving_substate::run()
{
    if(!initialized)
    {
        initialized = true;
    }
    if(move_sent) 
    {
        // sleep 5ms to allow for other stuff to go on
        usleep(5000);
        return;
    }
    {
        std::unique_lock<std::mutex> lck(moving_executed_mutex);
        moving_executed = 0;
    }
    //     geometry_msgs::Pose ee_pose;
    
    int i=-1;
    int move_num=0;
    std::string ee_name("");
    
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
        
        if(commands.is_exec_alone.count(item.second.command))
        {
            if(item.second.seq_num == 0)
            {
                ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : a command to be executed alone (" << item.second.command << ") has seq_num==0! This is NOT allowed!");
                failed = true;
                initialized = false;
            }
            
            srv.request.grasp_trajectory.header.seq = (int)item.second.ee_grasp_id;
            srv.request.attObject.object.id = *data_.object_name;
            srv.request.object_db_id = (int)*data_.obj_id;
            srv.request.ee_pose.clear();
            srv.request.ee_pose.push_back(item.second.cartesian_task);
            ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
            
            alone_execution = true;
            // NOTE: this has to be executed ALONE!
            break;
        }
        else
        {
            move_num++;
            if(!ee_name.empty() && ee_name != std::get<0>(db_mapper.EndEffectors.at(item.first)))
                ee_name = "full_robot";
            else
                ee_name = std::get<0>(db_mapper.EndEffectors.at(item.first));
        }
    }
    while((data_.cartesian_plan->at(data_.next_plan+i).second.seq_num==0) && (data_.cartesian_plan->size() > data_.next_plan+i+1));
    
    if(!failed)
    {
        srv.request.command = commands.command[data_.cartesian_plan->at(data_.next_plan+i).second.command];
        srv.request.ee_name = ee_name;
        std::unique_lock<std::mutex> lck(moving_executed_mutex);
        srv.request.seq=sequence_counter;
        
        if(client.call(srv))
        {
            ROS_INFO_STREAM("IK " << data_.cartesian_plan->at(data_.next_plan+i).second.command << " request accepted: (" << (int)srv.response.ack << ") - seq: "<<data_.next_plan);
            pending_sequence_numbers.insert(sequence_counter);
            data_.robot_moving.store(true && parallelize_planning);
        }
        else
        {
            ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : failed to call service ik_ros_service \'" << srv.request.command << "\'");
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
