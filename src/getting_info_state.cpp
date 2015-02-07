#include "getting_info_state.h"
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>

extern void fake_getting_info_run(visualization_msgs::Marker& marker,geometry_msgs::Pose& object_pose);

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
    

    fake_getting_info_run(marker,object_pose);
    data_.set_object_pose(object_pose);
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