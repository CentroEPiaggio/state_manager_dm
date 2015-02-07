#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <ik_control_state.h>

void fake_getting_info_run(visualization_msgs::Marker& marker,    geometry_msgs::Pose& object_pose)
{
    //NOTE: just a test
    
    object_pose.position.x =  -0.41;
    object_pose.position.y = 0.1;
    object_pose.position.z = 0.1;
    
    double roll = 1.565;
    double pitch = 0.0;
    double yaw = 0.336;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,object_pose.orientation);
    
    
    marker.pose = object_pose;
    marker.color.a = 1;
    marker.color.r=0;
    marker.color.g=0;
    marker.color.b=1;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x=0.05;
    marker.scale.y=0.05;
    marker.scale.z=0.05;
    marker.id=1;
    marker.ns="object";
    marker.header.frame_id="world";
    
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
