#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "geometry_msgs/Pose.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>

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
std::vector<std::map<std::string, geometry_msgs::Pose>> cartesian_plan;
private:
// geometry_msgs::Pose object_pose_;
};

class ik_shared_memory
{
public:
  int seq_num;
  const std::vector<std::map<std::string, geometry_msgs::Pose>>* cartesian_plan;
  std::map<std::string,bool> ees_grasped;
  std::map<std::string,KDL::Frame> objects_ees;
};

#endif // SHARED_MEMORY_H