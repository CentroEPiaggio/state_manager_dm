#include "getting_info_state.h"
#include <dual_manipulation_shared/planner_service.h>
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>
#include <kdl_conversions/kdl_msg.h>
#include "dual_manipulation_shared/scene_object_service.h"
#include "../../shared/src/lemon/bits/path_dump.h"
#include <dual_manipulation_shared/serialization_utils.h>
#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/parsing_utils.h>
#include <pacman_vision_comm/track_object.h>
#include <pacman_vision_comm/stop_track.h>

#define OBJ_GRASP_FACTOR 1000
#define CLASS_NAMESPACE "getting_info_state::"

getting_info_state::getting_info_state(shared_memory& data):data_(data),db_mapper_(data.db_mapper),target_set(false)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    
    XmlRpc::XmlRpcValue get_info_params;
    if (n.getParam("dual_manipulation_parameters", get_info_params)) parseParameters(get_info_params);

    pub = n.advertise<visualization_msgs::Marker>( "/object_marker", 1000 );
    planner_client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
    gui_target_client = n.serviceClient<dual_manipulation_shared::gui_target_service>("gui_target_service");
    scene_object_client = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
    vision_client = n.serviceClient<pacman_vision_comm::estimate>("/pacman_vision/estimator/estimate");
    target_sub = n.subscribe("/gui_target_response",1,&getting_info_state::gui_target_set_callback,this);
    tracker_start_client = n.serviceClient<pacman_vision_comm::track_object>("/pacman_vision/tracker/track_object");
    tracker_stop_client = n.serviceClient<pacman_vision_comm::stop_track>("/pacman_vision/tracker/stop_track");

    fresh_data = false;
}

void getting_info_state::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,use_vision,"use_vision");
}

void getting_info_state::get_start_position_from_vision(pacman_vision_comm::peArray& source_poses)
{
    pacman_vision_comm::estimate vision_srv;

    dual_manipulation_shared::scene_object_service srv_obj0;
    srv_obj0.request.command = "remove_all";
    if (!scene_object_client.call(srv_obj0))
    {
      ROS_ERROR_STREAM("getting_info_state::get_start_position_from_vision : Failed to call service dual_manipulation_shared::scene_object_service : " << srv_obj0.request.command);
    }
    
    pacman_vision_comm::stop_track track_srv;
    if (tracker_stop_client.call(track_srv))
      ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : unable to stop the tracker, just to inform you...!");
    
    if (vision_client.call(vision_srv))
    {
	ROS_INFO("IK_control: pacman_vision_comm::estimate service response: \n");

	for(auto pose:vision_srv.response.estimated.poses)
	{
	    ROS_INFO_STREAM("name: " << pose.name << " - parent: " << pose.parent_frame << "\n" << pose.pose <<"\n------------\n");
	    source_poses.poses.push_back(pose);
	    
	    // send information to the cartesian planner (for collision checking)
	    dual_manipulation_shared::scene_object_service srv_obj;
	    srv_obj.request.command = "add";
	    if(get_object_id(pose.id) == -1)
	    {
	      ROS_WARN_STREAM("getting_info_state::get_start_position_from_vision : the object " << pose.name << " has no matches in the DB - not publishing as a collision object");
	      continue;
	    }
	    srv_obj.request.object_db_id = get_object_id(pose.id);
	    // NOTE: this should be unique, while we can have more objects with the same "object_db_id"
	    srv_obj.request.attObject.object.id = pose.name;
	    srv_obj.request.attObject.object.mesh_poses.clear();
	    srv_obj.request.attObject.object.mesh_poses.push_back(pose.pose);
	    srv_obj.request.attObject.object.header.frame_id = pose.parent_frame;
	    if (scene_object_client.call(srv_obj))
	    {
		ROS_INFO("getting_info_state::get_start_position_from_vision : %s object %s request accepted: %d", srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str(), (int)srv_obj.response.ack);
	    }
	    else
	    {
		ROS_ERROR("getting_info_state::get_start_position_from_vision : Failed to call service dual_manipulation_shared::scene_object_service: %s %s",srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str());
	    }
	}
	if(vision_srv.response.estimated.poses.empty())
	{
	  failed = true;
	}
    }
    else
    {
	ROS_ERROR("getting_info_state::get_start_position_from_vision : Failed to call service pacman_vision_comm::estimate");
	if(use_vision)
	  failed = true;
    }
    
    source_set = !failed;
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
	    int grasp = (int)item.first;
	    grasp = grasp % OBJ_GRASP_FACTOR;
	    // ROS_INFO_STREAM("Deserializing object" + std::to_string(object_id) + "/grasp" + std::to_string(grasp));
	    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object_id) + "/grasp" + std::to_string(grasp));
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

int getting_info_state::get_object_id(std::string obj_name)
{
  for(auto item:db_mapper_.Objects)
  {
    std::string db_obj_name(std::get<0>(item.second));
    if(obj_name.compare(db_obj_name) == 0)
      return item.first;
  }
  return -1;
}

void getting_info_state::gui_target_set_callback(const dual_manipulation_shared::gui_target_response::ConstPtr& msg)
{
    if (target_set.load()) return;
    ROS_INFO_STREAM("Target set to "<<msg->target_pose.position.x<<' '<<msg->target_pose.position.y<<' '<<msg->target_pose.position.z<<' '<<msg->target_pose.orientation.x<<' '<<msg->target_pose.orientation.y<<' '<<msg->target_pose.orientation.z<<' '<<msg->target_pose.orientation.w);

    data_.source_position = msg->source_pose; //user selects which detected object is the source from the gui
    data_.target_position = msg->target_pose;
    data_.obj_id = msg->obj_id;
    data_.object_name = msg->name;
    // TODO: ask for desired target end-effector; maybe even for desired final grasp?
    data_.source_grasp=get_grasp_id_from_database(data_.obj_id,data_.source_position);
    data_.target_grasp=get_grasp_id_from_database(data_.obj_id,data_.target_position);

    pacman_vision_comm::track_object srv;
    srv.request.name = data_.object_name;

    if(!tracker_start_client.call(srv))
    {
      ROS_ERROR_STREAM("getting_info_state::gui_target_set_callback : unable to call track_object client...");
    }
    target_set.store(true);
}

void getting_info_state::get_target_position_from_user(pacman_vision_comm::peArray source_poses)
{
    dual_manipulation_shared::gui_target_service srv;

    srv.request.info = "waiting for target";
    srv.request.source_poses = source_poses;

    if (gui_target_client.call(srv))
    {
        ROS_INFO_STREAM("Answer: ("<<(bool)srv.response.ack<<")");
	if(srv.response.ack) ROS_INFO_STREAM("Waiting for target from user");
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::gui_target_service");
	// whichever the error is, the source needs to be set again: instead of going through a loop of vision calls, fail and revert to steady
	failed = true;
        return;
    }
    
    target_request=true;
}

std::map< transition, bool > getting_info_state::getResults()
{
    std::map< transition, bool > results;
    if(failed)
      results[transition::failed] = true;
    else
      results[transition::got_info] = fresh_data;
    return results;
}

void getting_info_state::run()
{
    pacman_vision_comm::peArray source_poses;

    if(!source_set) get_start_position_from_vision(source_poses);
    if(source_set && !target_request) get_target_position_from_user(source_poses);
    
    if(target_set.load())
    {

	dual_manipulation_shared::planner_service srv;
	srv.request.command="set object";
	srv.request.time = ros::Time::now().toSec();
	srv.request.object_id=data_.obj_id;
	srv.request.object_name=data_.object_name;
        data_.planner.set_object(data_.obj_id,data_.object_name);
	if (!planner_client.exists())
	{
	    ROS_ERROR("Service does not exist: dual_manipulation_shared::planner_service");
// 	    failed=true;
// 	    return;
	}
	else if (planner_client.call(srv))
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
	
	//NOTE: the following is not necessary when using vision, nonetheless the new collision object will override the one previously set with the same attObject.object.id
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
	source_set = false;
	target_request = false;
	target_set = false;
    }
}

bool getting_info_state::isComplete()
{
    return fresh_data || failed;
}

std::string getting_info_state::get_type()
{
    return "getting_info_state";
}

void getting_info_state::reset()
{
    fresh_data = false;
    source_set = false;
    target_request = false;
    target_set = false;
    failed = false;
}
