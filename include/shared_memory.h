#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "geometry_msgs/Pose.h"

class shared_memory
{
public:
shared_memory();
void set_object_pose(geometry_msgs::Pose object_pose);
void get_object_pose(geometry_msgs::Pose& object_pose);

private:
geometry_msgs::Pose object_pose_;
};

#endif // SHARED_MEMORY_H
