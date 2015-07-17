#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "geometry_msgs/Pose.h"
#include <kdl/frames.hpp>
#include <atomic>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_item.h>
#include <dual_manipulation_shared/ik_control_capabilities.h>
#include <dual_manipulation_planner/planner_lib.h>

enum class cartesian_commands
{
    MOVE,
    MOVE_NO_COLLISION_CHECK,
    MOVE_BEST_EFFORT,
    MOVE_CLOSE_BEST_EFFORT,
    GRASP,
    UNGRASP,
    HOME
};

class planning_cmd
{
public:
  planning_cmd();
  ~planning_cmd(){};
  
  ik_control_capability capabilities;
  std::map<cartesian_commands,std::string> set_target_command;
  std::map<cartesian_commands,std::string> plan_command;
  // all possible commands which can follow a given command
  std::map<cartesian_commands,std::set<cartesian_commands>> can_follow;
  // says whether a command should "flush" previously received commands (apply them before continuing)
  std::set<cartesian_commands> flushing;
  // says whether a command has to be planned
  std::set<cartesian_commands> is_to_be_planned;
  // says whether a command has to be checked
  std::set<cartesian_commands> is_to_be_checked;
};

class moving_cmd
{
public:
  moving_cmd();
  ~moving_cmd(){};
  
  ik_control_capability capabilities;
  std::map<cartesian_commands,std::string> command;
  // says whether a command has to be executed alone
  std::set<cartesian_commands> is_exec_alone;
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
std::ostream& operator<<(std::ostream &output, const cartesian_commands &command);

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
dual_manipulation_shared::planner_item filtered_source_nodes,filtered_target_nodes;
dual_manipulation::planner::planner_lib planner;

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
  std::atomic_bool robot_moving;
  std::atomic_bool move_failed;
};

#endif // SHARED_MEMORY_H