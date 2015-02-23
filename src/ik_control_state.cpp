#include "../include/ik_control_state.h"
#include "../include/ik_planning_substate.h"
#include "../include/ik_moving_substate.h"
#include "../include/ik_grasping_substate.h"
#include "../include/ik_checking_grasp_substate.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

ik_control_state::ik_control_state(shared_memory& data):data_(data)
{
    // fake_plan();
    print_plan();
    subdata.cartesian_plan = &data.cartesian_plan;
    subdata.next_plan=0;
    subdata.obj_id = &data.obj_id;
    subdata.object_name = &data.object_name;

    auto ik_planning = new ik_planning_substate(subdata);
    auto ik_moving = new ik_moving_substate(subdata);
    waiting = new ik_steady_substate(subdata);
    auto exiting = new ik_exiting_substate(subdata);
    
    std::vector<std::tuple<abstract_state<ik_transition>*,ik_transition_type,abstract_state<ik_transition>*>> transition_table{
        //------initial state---------------+--------- command ---------------------------------------+-- final state------ +
        std::make_tuple( waiting            , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::move,true)                ,   ik_moving         ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
//         std::make_tuple( ik_moving          , std::make_pair(ik_transition::grasp,true)               ,   ik_grasping       ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::done,true)                ,   exiting           ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
//         std::make_tuple( ik_grasping        , std::make_pair(ik_transition::done,true)                ,   exiting           ),
//         std::make_tuple( ik_grasping        , std::make_pair(ik_transition::checkgrasp,true)          ,   ik_checking_grasp ),
// 	std::make_tuple( ik_grasping        , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
// 	//----------------------------------+---------------------------------------------------------+-------------------- +
// 	std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::check_done,true)          ,   ik_grasping       )
    };

    sm.insert(transition_table);
    
    result = false;
    complete = false;
    current_state=waiting;
}

std::map< transition, bool > ik_control_state::getResults()
{
    std::map< transition, bool > results;
    result =  (subdata.next_plan == subdata.cartesian_plan->size()-1);
    results[transition::task_accomplished]=result;
    subdata.next_plan=0;
    complete=false;
    current_state=waiting;
    return results;
}

void ik_control_state::run()
{
    if(current_state->get_type()=="ik_exiting_substate") complete = true;

    show_plan_with_tf();
    
    current_state->run();
    
    if (current_state->isComplete())
    {
	ROS_INFO_STREAM("current_state " << current_state->get_type() << " is complete!");
	auto temp_map = current_state->getResults();
	for (auto temp:temp_map)
	    transition_map[temp.first]=temp.second;
    }
    for (auto trigger: transition_map)
    {
	auto temp_state = sm.evolve_state_machine(current_state, trigger);
	if (temp_state!=current_state)
	{
	    current_state=temp_state;
	    std::cout<<"- new substate type: "<<current_state->get_type()<<std::endl;
	    std::cout << "press 'y' key and enter to proceed" << std::endl;
	    char tmp = 'n';
	    while (tmp!='y')
	    {
	      std::cin >> tmp;
	      usleep(200000);
	    }
	    transition_map.clear();
	    break;
	}
    }
}

bool ik_control_state::isComplete()
{
    return complete;
}

std::string ik_control_state::get_type()
{
    return "ik_control_state";
}

void ik_control_state::print_plan()
{
    int i=0;
    for(auto item:data_.cartesian_plan)
    {
	auto subitem=item;
	{
            ROS_INFO_STREAM(i<<") "<<subitem.first<<" [ p.x: "<< subitem.second.cartesian_task.position.x<<" p.y: "<< subitem.second.cartesian_task.position.y<<" p.z: "<< subitem.second.cartesian_task.position.z<<
            " o.x: "<< subitem.second.cartesian_task.orientation.x<<" o.y: "<< subitem.second.cartesian_task.orientation.y<<" o.z: "<< subitem.second.cartesian_task.orientation.z<<" o.w: "<< subitem.second.cartesian_task.orientation.w<<" ]"<<std::endl);
	}
	i++;
    }
}

void poseCallback(const cartesian_command& msg, std::string prefix)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::poseMsgToTF(msg.cartesian_task,transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", prefix + " | command:" + std::to_string((int)msg.command)));
}

void ik_control_state::show_plan_with_tf()
{
    int i=0;
    for (auto item:(*subdata.cartesian_plan))
    {
      poseCallback(item.second, std::to_string(i++) + "ee_id:" + std::to_string((int)item.first));
    }
}
