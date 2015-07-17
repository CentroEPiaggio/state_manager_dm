#include "../include/shared_memory.h"

shared_memory::shared_memory()
{
    reset();
}

void shared_memory::reset()
{
    cartesian_plan.clear();
//     filtered_source_nodes.clear();
//     filtered_target_nodes.clear();
    obj_id=-1;
    object_name="";
    source_grasp=-1;
    target_grasp=-1;
    target_position=geometry_msgs::Pose();
    source_position=geometry_msgs::Pose();
    planner.clear_filtered_arcs();
}

std::ostream& operator<<(std::ostream& output, const cartesian_commands& command)
{
  output << (command==cartesian_commands::HOME?"home":command==cartesian_commands::MOVE?"move":command==cartesian_commands::MOVE_NO_COLLISION_CHECK?"move w/o collision check":command==cartesian_commands::MOVE_BEST_EFFORT?"move best-effort":command==cartesian_commands::MOVE_CLOSE_BEST_EFFORT?"move close best-effort":command==cartesian_commands::GRASP?"grasp":command==cartesian_commands::UNGRASP?"ungrasp":"");
}

std::ostream& operator<<(std::ostream &output, const cartesian_command &o) {
    output<<std::endl;
    output<<o.cartesian_task<<o.command<<(o.command==cartesian_commands::GRASP?(" #" + std::to_string(o.ee_grasp_id)):o.command==cartesian_commands::UNGRASP?(" #" + std::to_string(o.ee_grasp_id)):"")<<"\nseq_num: "<<o.seq_num<<std::endl;
    return output;
}

planning_cmd::planning_cmd()
{
  set_target_command[cartesian_commands::MOVE] = capabilities.name[ik_control_capabilities::SET_TARGET];
  set_target_command[cartesian_commands::MOVE_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::SET_TARGET];
  set_target_command[cartesian_commands::MOVE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::SET_TARGET];
  set_target_command[cartesian_commands::MOVE_CLOSE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::SET_TARGET];
  set_target_command[cartesian_commands::HOME] = capabilities.name[ik_control_capabilities::SET_HOME_TARGET];

  plan_command[cartesian_commands::MOVE] = capabilities.name[ik_control_capabilities::PLAN];
  plan_command[cartesian_commands::MOVE_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::PLAN_NO_COLLISION];
  plan_command[cartesian_commands::MOVE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::PLAN_BEST_EFFORT];
  plan_command[cartesian_commands::MOVE_CLOSE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT];
  plan_command[cartesian_commands::HOME] = capabilities.name[ik_control_capabilities::PLAN];
  
  can_follow[cartesian_commands::MOVE].insert(cartesian_commands::MOVE);
  can_follow[cartesian_commands::MOVE].insert(cartesian_commands::MOVE_BEST_EFFORT);
  can_follow[cartesian_commands::MOVE].insert(cartesian_commands::HOME);
  can_follow[cartesian_commands::MOVE_BEST_EFFORT].insert(cartesian_commands::MOVE);
  can_follow[cartesian_commands::MOVE_BEST_EFFORT].insert(cartesian_commands::MOVE_BEST_EFFORT);
  can_follow[cartesian_commands::MOVE_BEST_EFFORT].insert(cartesian_commands::HOME);
  can_follow[cartesian_commands::HOME].insert(cartesian_commands::MOVE);
  can_follow[cartesian_commands::HOME].insert(cartesian_commands::MOVE_BEST_EFFORT);
  can_follow[cartesian_commands::HOME].insert(cartesian_commands::HOME);
  
  flushing.insert(cartesian_commands::HOME);
  
  is_to_be_checked.insert(cartesian_commands::GRASP);
  
  is_to_be_planned.insert(cartesian_commands::MOVE);
  is_to_be_planned.insert(cartesian_commands::MOVE_NO_COLLISION_CHECK);
  is_to_be_planned.insert(cartesian_commands::MOVE_BEST_EFFORT);
  is_to_be_planned.insert(cartesian_commands::MOVE_CLOSE_BEST_EFFORT);
  is_to_be_planned.insert(cartesian_commands::HOME);
}

moving_cmd::moving_cmd()
{
  command[cartesian_commands::MOVE] = capabilities.name[ik_control_capabilities::MOVE];
  command[cartesian_commands::MOVE_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::MOVE];
  command[cartesian_commands::MOVE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::MOVE];
  command[cartesian_commands::MOVE_CLOSE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::MOVE];
  command[cartesian_commands::GRASP] = capabilities.name[ik_control_capabilities::GRASP];
  command[cartesian_commands::UNGRASP] = capabilities.name[ik_control_capabilities::UNGRASP];
  command[cartesian_commands::HOME] = capabilities.name[ik_control_capabilities::MOVE];
  
  is_exec_alone.insert(cartesian_commands::GRASP);
  is_exec_alone.insert(cartesian_commands::UNGRASP);
}
