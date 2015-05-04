#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "geometry_msgs/Pose.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_item.h>
#include <dual_manipulation_shared/ik_control_capabilities.h>

enum class cartesian_commands
{
    MOVE,
    MOVE_NO_COLLISION_CHECK,
    MOVE_BEST_EFFORT,
    MOVE_BEST_EFFORT_NO_COLLISION_CHECK,
    GRASP,
    UNGRASP,
    HOME
};

class planning_cmd
{
public:
  planning_cmd()
  {
    plan_command[cartesian_commands::MOVE] = capabilities.name[ik_control_capabilities::PLAN];
    plan_command[cartesian_commands::MOVE_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::PLAN_NO_COLLISION];
    plan_command[cartesian_commands::MOVE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::PLAN_BEST_EFFORT];
    plan_command[cartesian_commands::MOVE_BEST_EFFORT_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::PLAN_BEST_EFFORT_NO_COLLISION];
    
    is_to_be_checked.insert(cartesian_commands::GRASP);
    
    is_to_be_planned.insert(cartesian_commands::MOVE);
    is_to_be_planned.insert(cartesian_commands::MOVE_NO_COLLISION_CHECK);
    is_to_be_planned.insert(cartesian_commands::MOVE_BEST_EFFORT);
    is_to_be_planned.insert(cartesian_commands::MOVE_BEST_EFFORT_NO_COLLISION_CHECK);
  };
  ~planning_cmd(){};
  
  ik_control_capability capabilities;
  std::map<cartesian_commands,std::string> plan_command;
  // says whether a command has to be planned
  std::set<cartesian_commands> is_to_be_planned;
  // says whether a command has to be checked
  std::set<cartesian_commands> is_to_be_checked;
};

class moving_cmd
{
public:
  moving_cmd()
  {
    command[cartesian_commands::MOVE] = capabilities.name[ik_control_capabilities::MOVE];
    command[cartesian_commands::MOVE_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::MOVE];
    command[cartesian_commands::MOVE_BEST_EFFORT] = capabilities.name[ik_control_capabilities::MOVE];
    command[cartesian_commands::MOVE_BEST_EFFORT_NO_COLLISION_CHECK] = capabilities.name[ik_control_capabilities::MOVE];
    command[cartesian_commands::GRASP] = capabilities.name[ik_control_capabilities::GRASP];
    command[cartesian_commands::UNGRASP] = capabilities.name[ik_control_capabilities::UNGRASP];
    command[cartesian_commands::HOME] = capabilities.name[ik_control_capabilities::HOME];
    //command[cartesian_commands::HOME] = capabilities.name[ik_control_capabilities::MOVE];
  };
  ~moving_cmd(){};
  
  ik_control_capability capabilities;
  std::map<cartesian_commands,std::string> command;
};

struct cartesian_command
{
    cartesian_command(){};
    cartesian_command(cartesian_commands command,int seq_num,grasp_id ee_grasp_id):command(command),seq_num(seq_num),ee_grasp_id(ee_grasp_id)
    {
        
    };
    geometry_msgs::Pose cartesian_task; //desired position of the HAND in the pregrasp, not the object!!
    cartesian_commands command;
    /**
     * @brief seq_num==0 means that the next cartesian command should be executed in parallel with this one
     * seq_num==1 means that the next cartesian command should be executed after this one
     */
    int seq_num;
    grasp_id ee_grasp_id;
};

std::ostream& operator<<(std::ostream &output, const cartesian_command &o);
class shared_memory
{
public:
shared_memory();
// void set_object_pose(geometry_msgs::Pose object_pose);
// void get_object_pose(geometry_msgs::Pose& object_pose);
geometry_msgs::Pose source_position, target_position;
grasp_id source_grasp, target_grasp;
object_id obj_id;
std::string object_name;
std::vector<std::pair<endeffector_id,cartesian_command>> cartesian_plan;
std::vector<dual_manipulation_shared::planner_item> filtered_source_nodes,filtered_target_nodes;
void reset();
private:
// geometry_msgs::Pose object_pose_;
};

class ik_shared_memory
{
public:
  int next_plan;
  std::vector<std::pair<endeffector_id,cartesian_command>>* cartesian_plan;
  const object_id* obj_id;
  const std::string* object_name;
};

#endif // SHARED_MEMORY_H