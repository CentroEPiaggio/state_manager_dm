#include "getting_info_state.h"
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>

getting_info_state::getting_info_state(shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    pub = n.advertise<visualization_msgs::Marker>( "/object_marker", 1000 );

    fresh_data = false;
}

std::map< transition, bool > getting_info_state::getResults()
{
    std::map< transition, bool > results;
    results[transition::got_info]=fresh_data;
    return results;
}

void getting_info_state::run()
{
    //NOTE: just a test

    geometry_msgs::Pose object_pose;
    object_pose.position.x =  -0.41;
    object_pose.position.y = 0.1;
    object_pose.position.z = 0.1;

    double roll = 1.565;
    double pitch = 0.0;
    double yaw = 0.336;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,object_pose.orientation);

    data_.set_object_pose(object_pose);

    visualization_msgs::Marker marker;
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

    pub.publish(marker);

    fresh_data = true;
}

bool getting_info_state::isComplete()
{
    return fresh_data;
}

std::string getting_info_state::get_type()
{
    return "getting_info_state";
}