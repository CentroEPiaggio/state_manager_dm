#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "geometry_msgs/Pose.h"
#include <kdl/frames.hpp>

class shared_memory
{
public:
shared_memory();
void set_object_pose(geometry_msgs::Pose object_pose);
void get_object_pose(geometry_msgs::Pose& object_pose);
std::vector<std::map<std::string, geometry_msgs::Pose>> cartesian_plan;
private:
geometry_msgs::Pose object_pose_;
};

class ik_shared_memory
{
public:
  const std::vector<std::map<std::string, geometry_msgs::Pose>>* cartesian_plan;
  std::map<std::string,bool> ees_grasped;
  std::map<std::string,KDL::Frame> objects_ees;
};

#endif // SHARED_MEMORY_H