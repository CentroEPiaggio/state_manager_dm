#include "../include/shared_memory.h"

shared_memory::shared_memory()
{
    object_pose_.position.x = 0;
    object_pose_.position.y = 0;
    object_pose_.position.z = 0;
    object_pose_.orientation.w = 1;
    object_pose_.orientation.x = 0;
    object_pose_.orientation.y = 0;
    object_pose_.orientation.z = 0;
}

void shared_memory::get_object_pose(geometry_msgs::Pose& object_pose)
{
    object_pose=object_pose_;
}

void shared_memory::set_object_pose(geometry_msgs::Pose object_pose)
{
    object_pose_=object_pose;
}