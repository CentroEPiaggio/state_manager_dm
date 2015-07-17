#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <ik_control_state.h>

int fake_get_grasp_id_from_database()
{
    // only considers a table grasp, just to be sure
    int table_max = 15;
    int table_min = 12;
    return table_min + (rand() % (table_max - table_min + 1));
}

void fake_get_start_position_from_vision(shared_memory& data,visualization_msgs::Marker& source_marker)
{
    //NOTE: just a test
//     data.obj_id=1;
//     data.object_name="cylinder";

//     data.source_position.position.x = -0.35;
//     data.source_position.position.y = 0.3;
//     data.source_position.position.z = 0.06;
//     data.source_grasp=12; // in this position, this is the right grasp I would expect from a function // fake_get_grasp_id_from_database();
//     data.obj_id=1;
//     data.object_name="cylinder";
//     double roll = 1.565;
//     double pitch = 0.0;
//     double yaw = 3.14+0.336;
//     tf::Quaternion q;
//     q.setRPY(roll,pitch,yaw);
//     tf::quaternionTFToMsg(q,data.source_position.orientation);
//     source_marker.pose = data.source_position;
//     source_marker.color.a = 1;
//     source_marker.color.r=0;
//     source_marker.color.g=0;
//     source_marker.color.b=1;
//     source_marker.type = visualization_msgs::Marker::CYLINDER;
//     source_marker.scale.x=0.05;
//     source_marker.scale.y=0.05;
//     source_marker.scale.z=0.05;
//     source_marker.id=1;
//     source_marker.ns="object";
//     source_marker.header.frame_id="world";
}

void fake_getting_info_run(shared_memory& data,visualization_msgs::Marker& source_marker,visualization_msgs::Marker& target_marker)
{
    //NOTE: just a test

    data.source_position.position.x = -0.25;
    data.source_position.position.y = 0.3;
    data.source_position.position.z = 0;
    data.source_grasp=7;
    data.target_position.position.x = 0.3;
    data.target_position.position.y = 0.5;
    data.target_position.position.z = 0;
    data.target_grasp=8;
    data.obj_id=1;
    data.object_name="cylinder";
    double roll = 1.565;
    double pitch = 0.0;
    double yaw = 0.336;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,data.source_position.orientation);
    tf::quaternionTFToMsg(q,data.target_position.orientation);

    source_marker.pose = data.source_position;
    source_marker.color.a = 1;
    source_marker.color.r=0;
    source_marker.color.g=0;
    source_marker.color.b=1;
    source_marker.type = visualization_msgs::Marker::CYLINDER;
    source_marker.scale.x=0.05;
    source_marker.scale.y=0.05;
    source_marker.scale.z=0.05;
    source_marker.id=1;
    source_marker.ns="object";
    source_marker.header.frame_id="world";

    target_marker.pose = data.target_position;
    target_marker.color.a = 1;
    target_marker.color.r=0;
    target_marker.color.g=0;
    target_marker.color.b=1;
    target_marker.type = visualization_msgs::Marker::CYLINDER;
    target_marker.scale.x=0.05;
    target_marker.scale.y=0.05;
    target_marker.scale.z=0.05;
    target_marker.id=1;
    target_marker.ns="target";
    target_marker.header.frame_id="world";
}
