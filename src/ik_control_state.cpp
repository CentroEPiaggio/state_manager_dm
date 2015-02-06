#include "../include/ik_control_state.h"
#include "../include/ik_planning_substate.h"
#include "../include/ik_moving_substate.h"
#include "../include/ik_grasping_substate.h"
#include "../include/ik_checking_grasp_substate.h"
#include "tf/tf.h"

ik_control_state::ik_control_state(shared_memory& data):data_(data)
{
    fake_plan();
    subdata.cartesian_plan = &data.cartesian_plan;

    auto ik_planning = new ik_planning_substate(subdata);
    auto ik_moving = new ik_moving_substate(subdata);
    auto ik_grasping = new ik_grasping_substate(subdata);
    auto ik_checking_grasp = new ik_checking_grasp_substate(subdata);
    auto waiting = new ik_steady_substate(subdata);
    exiting = new ik_steady_substate(subdata);
    
    std::vector<std::tuple<abstract_state<ik_transition>*,ik_transition_type,abstract_state<ik_transition>*>> transition_table{
        //------initial state---------------+--------- command ---------------------------------------+-- final state------ +
        std::make_tuple( waiting            , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::move,true)                ,   ik_moving         ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::grasp,true)               ,   ik_grasping       ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::done,true)                ,   exiting           ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_grasping        , std::make_pair(ik_transition::grasp,true)               ,   exiting           ),
        std::make_tuple( ik_grasping        , std::make_pair(ik_transition::checkgrasp,true)          ,   ik_checking_grasp ),
	std::make_tuple( ik_grasping        , std::make_pair(ik_transition::move,true)                ,   ik_planning       ),
	//----------------------------------+---------------------------------------------------------+-------------------- +
	std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::check_done,true)          ,   ik_grasping       )
    };

    sm.insert(transition_table);
    
    result = false;
    complete = false;
    current_state=waiting;
}

std::map< transition, bool > ik_control_state::getResults()
{
    std::map< transition, bool > results;
    results[transition::task_accomplished]=result;
    return results;
}

void ik_control_state::run()
{
    if(current_state==exiting) complete = true;
}

bool ik_control_state::isComplete()
{
    return complete;
}

std::string ik_control_state::get_type()
{
    return "ik_control_state";
}

void ik_control_state::fake_plan()
{
    geometry_msgs::Pose r_ee_pose;
    geometry_msgs::Pose l_ee_pose;

    data_.get_object_pose(r_ee_pose);

    data_.get_object_pose(l_ee_pose);
    double roll = -1.565;
    double pitch = 0.0;
    double yaw = 0.336;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,l_ee_pose.orientation);

    std::map<std::string, geometry_msgs::Pose> poses;

    poses["right_hand"] = r_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    r_ee_pose.position.y-=0.1;
    poses["right_hand"] = r_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    l_ee_pose.position.z+=0.2;
    l_ee_pose.position.y-=0.15;
    poses["left_hand"] = l_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    r_ee_pose.position.y+=0.1;
    poses["right_hand"] = r_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    l_ee_pose.position.y-=0.1;
    poses["left_hand"] = l_ee_pose;
    data_.cartesian_plan.push_back(poses);
}

void ik_control_state::print_plan()
{
    int i=0;
    for(auto item:data_.cartesian_plan)
    {
	for(auto subitem:item)
	{
	    ROS_INFO_STREAM(i<<") "<<subitem.first<<" [ p.x: "<< subitem.second.position.x<<" p.y: "<< subitem.second.position.y<<" p.z: "<< subitem.second.position.z<<
	    " o.x: "<< subitem.second.orientation.x<<" o.y: "<< subitem.second.orientation.y<<" o.z: "<< subitem.second.orientation.z<<" o.w: "<< subitem.second.orientation.w<<" ]"<<std::endl);
	}
	i++;
    }
}