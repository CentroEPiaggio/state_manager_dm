#include "getting_info_state.h"
#include <dual_manipulation_shared/planner_service.h>
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>

extern void fake_getting_info_run(shared_memory& data,visualization_msgs::Marker& source_marker,visualization_msgs::Marker& target_marker);
extern void fake_get_start_position_from_vision(shared_memory& data,visualization_msgs::Marker& source_marker);

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
    gui_target_client = n.serviceClient<dual_manipulation_shared::gui_target_service>("gui_target_service");
    fresh_data = false;
}

void getting_info_state::get_start_position_from_vision(visualization_msgs::Marker& source_marker)
{
    fake_get_start_position_from_vision(data_,source_marker);
}

void getting_info_state::get_target_position_from_user(visualization_msgs::Marker& target_marker)
{
    dual_manipulation_shared::gui_target_service srv;
    
    srv.request.info = "waiting for target";
    
    if (gui_target_client.call(srv))
    {
        ROS_INFO_STREAM("Answer: ("<<srv.response.ack<<")");
	if(srv.response.ack) ROS_INFO_STREAM("Target set to "<<srv.response.target_pose.position.x<<' '<<srv.response.target_pose.position.y<<' '<<srv.response.target_pose.position.z
	  <<' '<<srv.response.target_pose.orientation.x<<' '<<srv.response.target_pose.orientation.y<<' '<<srv.response.target_pose.orientation.z<<' '<<srv.response.target_pose.orientation.w);
        
	data_.target_position.position.x = srv.response.target_pose.position.x;
	data_.target_position.position.y = srv.response.target_pose.position.y;
	data_.target_position.position.z = srv.response.target_pose.position.z;
	data_.target_grasp=8;

	double roll = 1.565;
	double pitch = 0.0;
	double yaw = 0.336;
	tf::Quaternion q;
	q.setRPY(roll,pitch,yaw);
	tf::quaternionTFToMsg(q,data_.target_position.orientation);

	target_marker.pose = data_.target_position;
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
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::gui_target_service");
        return;
    }
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

    get_start_position_from_vision(source_marker);
    get_target_position_from_user(target_marker);
    //fake_getting_info_run(data_,source_marker,target_marker);
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