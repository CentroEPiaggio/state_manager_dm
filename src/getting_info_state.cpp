#include "getting_info_state.h"
#include <dual_manipulation_shared/planner_service.h>
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>
#include <kdl_conversions/kdl_msg.h>
#include "dual_manipulation_shared/scene_object_service.h"
#include "../../shared/include/dual_manipulation_shared/databasemapper.h"
#include <dual_manipulation_shared/serialization_utils.h>
#include <dual_manipulation_shared/ik_service.h>

extern void fake_getting_info_run(shared_memory& data,visualization_msgs::Marker& source_marker,visualization_msgs::Marker& target_marker);
extern void fake_get_start_position_from_vision(shared_memory& data,visualization_msgs::Marker& source_marker);
extern int fake_get_grasp_id_from_database();

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
    scene_object_client = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");

    fresh_data = false;
}

void getting_info_state::get_start_position_from_vision(visualization_msgs::Marker& source_marker)
{
    data_.object_name="Cylinder";  
    //fake_get_start_position_from_vision(data_,source_marker);
}

int getting_info_state::get_grasp_id_from_database(int object_id, geometry_msgs::Pose pose, int ee_id)
{
    // make sure we are considering only table grasps
    ee_id = 3;
    ROS_WARN_STREAM("!! getting_info_state::get_grasp_id_from_database : only considering table grasps for now !!");
    
    KDL::Frame obj_frame,grasp_frame;
    tf::poseMsgToKDL(pose,obj_frame);
    double x,y,z,w;
    
    int best_grasp = -1;
    double closeness = -1.0;
    
    for (auto item:db_mapper_.Grasps)
    {
	auto ee_id_tmp = std::get<1>(item.second);
	auto obj_id_tmp = std::get<0>(item.second);
	//auto grasp_name = std::get<2>(item.second);
	//std::cout << "grasp name : " << grasp_name << std::endl;
	
	// for each grasp, if the end-effector is the right one
	if (((int)ee_id_tmp == ee_id) && ((int)obj_id_tmp == object_id))
	{
	    // deserialize grasp
	    dual_manipulation_shared::ik_service srv;
	    // ROS_INFO_STREAM("Deserializing object" + std::to_string(object_id) + "/grasp" + std::to_string((int)item.first));
	    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object_id) + "/grasp" + std::to_string((int)item.first));
	    if (ok)
	      tf::poseMsgToKDL(srv.request.ee_pose.back(),grasp_frame);
	    else
	      ROS_WARN_STREAM("Unable to deserialize grasp entry : object" + std::to_string(object_id) + "/grasp" + std::to_string((int)item.first));
	    
	    // get residual rotation and its quaternion representation
	    KDL::Rotation Rresidual = grasp_frame.M.Inverse()*(obj_frame.M);
	    Rresidual.GetQuaternion(x,y,z,w);
	  
	    // the higher the w (in abs value) the better (smaller rotation angles around any axis)
	    if((closeness < 0) || (std::abs(w) > closeness))
	    {
		closeness = std::abs(w);
		best_grasp = item.first;
	    }
	}
    }
    
    ROS_INFO_STREAM("Best grasp found: " << best_grasp);
    return best_grasp;
}

void getting_info_state::get_target_position_from_user()
{
    dual_manipulation_shared::gui_target_service srv;
    
    srv.request.info = "waiting for target";
    
    if (gui_target_client.call(srv))
    {
        ROS_INFO_STREAM("Answer: ("<<(bool)srv.response.ack<<")");
	if(srv.response.ack) ROS_INFO_STREAM("Target set to "<<srv.response.target_pose.position.x<<' '<<srv.response.target_pose.position.y<<' '<<srv.response.target_pose.position.z
	  <<' '<<srv.response.target_pose.orientation.x<<' '<<srv.response.target_pose.orientation.y<<' '<<srv.response.target_pose.orientation.z<<' '<<srv.response.target_pose.orientation.w);
        
	data_.source_position = srv.response.source_pose;
	data_.target_position = srv.response.target_pose;
	// TODO: take this from vision
	data_.obj_id = srv.response.obj_id;
	std::cout << "data_.obj_id = " << srv.response.obj_id << " | " << data_.obj_id << std::endl;
	// TODO: ask for desired target end-effector; maybe even for desired final grasp?
	data_.source_grasp=get_grasp_id_from_database(data_.obj_id,data_.source_position);
	data_.target_grasp=get_grasp_id_from_database(data_.obj_id,data_.target_position);
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
    visualization_msgs::Marker source_marker;

    get_start_position_from_vision(source_marker);
    get_target_position_from_user();
    //fake_getting_info_run(data_,source_marker,target_marker);
    // pub.publish(source_marker);
    
    dual_manipulation_shared::planner_service srv;
    srv.request.command="set object";
    srv.request.time = 2; //TODO
    srv.request.object_id=data_.obj_id;
    srv.request.object_name=data_.object_name;
    if (!planner_client.exists())
    {
        ROS_ERROR("Service does not exist: dual_manipulation_shared::planner_service");
        failed=true;
        return;
    }
    if (planner_client.call(srv))
    {
        ROS_INFO("Object id set to %d, planner returned %d", (int)srv.request.object_id, (int)srv.response.ack);
        data_.obj_id=srv.request.object_id;
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::planner_service");
        failed=true;
        return;
    }
    
    // send information to the cartesian planner (for collision checking)
    dual_manipulation_shared::scene_object_service srv_obj;
    srv_obj.request.command = "add";
    srv_obj.request.object_db_id = data_.obj_id;
    // NOTE: this should be unique, while we can have more objects with the same "object_db_id"
    srv_obj.request.attObject.object.id = data_.object_name;
    srv_obj.request.attObject.object.mesh_poses.push_back( data_.source_position );
    srv_obj.request.attObject.object.header.frame_id = "world";
    if (scene_object_client.call(srv_obj))
    {
	ROS_INFO("IK_control:test_grasping : %s object %s request accepted: %d", srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str(), (int)srv_obj.response.ack);
    }
    else
    {
	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service: %s %s",srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str());
    }

    fresh_data = true;
}

bool getting_info_state::isComplete()
{
    return fresh_data || failed;
}

std::string getting_info_state::get_type()
{
    return "getting_info_state";
}