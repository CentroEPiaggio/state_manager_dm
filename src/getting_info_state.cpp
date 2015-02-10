#include "getting_info_state.h"
#include <dual_manipulation_shared/planner_service.h>
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>

extern void fake_getting_info_run(shared_memory& data,visualization_msgs::Marker& source_marker,visualization_msgs::Marker& target_marker);

getting_info_state::getting_info_state(shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    pub = n.advertise<visualization_msgs::Marker>( "/object_marker", 1000 );
    planner_client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
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
    visualization_msgs::Marker source_marker,target_marker;

    fake_getting_info_run(data_,source_marker,target_marker);
    pub.publish(source_marker);
    pub.publish(target_marker);
    
    dual_manipulation_shared::planner_service srv;
    srv.request.command="set object";
    srv.request.time = 2; //TODO
    srv.request.object_id=data_.obj_id;
    srv.request.object_name=data_.object_name;
    
    if (planner_client.call(srv))
    {
        ROS_INFO("Object id set to %d, planner returned %d", (int)srv.request.object_id, (int)srv.response.ack);
        data_.obj_id=srv.request.object_id;
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::planner_service");
        return;
    }
    
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