#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "geometry_msgs/Pose.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_item.h>

enum class cartesian_commands
{
    MOVE,
    GRASP,
    UNGRASP,
    HOME
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
  const std::vector<std::pair<endeffector_id,cartesian_command>>* cartesian_plan;
  const object_id* obj_id;
  const std::string* object_name;
};

#endif // SHARED_MEMORY_H