#include "getting_info_state.h"
#include <dual_manipulation_shared/planner_service.h>
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>
#include <kdl_conversions/kdl_msg.h>
#include "dual_manipulation_shared/scene_object_service.h"
#include <dual_manipulation_shared/serialization_utils.h>
#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/parsing_utils.h>
#include <pacman_vision_comm/track_object.h>
#include <pacman_vision_comm/stop_track.h>

#define OBJ_GRASP_FACTOR 1000
#define CLASS_NAMESPACE "getting_info_state::"
#define CLASS_LOGNAME "getting_info_state"

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

    planner_client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
    gui_target_client = n.serviceClient<dual_manipulation_shared::gui_target_service>("gui_target_service");
    scene_object_client = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
    vision_client = n.serviceClient<pacman_vision_comm::estimate>("/pacman_vision/estimator/estimate");
    target_sub = n.subscribe("gui_target_response",1,&getting_info_state::gui_target_set_callback,this);
    tracker_start_client = n.serviceClient<pacman_vision_comm::track_object>("/pacman_vision/tracker/track_object");
    tracker_stop_client = n.serviceClient<pacman_vision_comm::stop_track>("/pacman_vision/tracker/stop_track");

    fresh_data = false;
}

void getting_info_state::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,use_vision,"use_vision");
    parseSingleParameter(params,table_ee_id,"table_ee_id");
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
        if(use_vision)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Failed to call service " << vision_client.getService());
            failed = true;
        }
    }
    
    source_set = !failed;
}

int getting_info_state::get_grasp_id_from_database(int object_id, geometry_msgs::Pose pose, int ee_id)
{
    // make sure we are considering only non movable ee grasps for now (as of 10/07/2017)
    
    ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : Only considering non movable end-effector grasps for now with ids " << table_ee_id << ", " << edge_ee_id << ", " << wall_ee_id << "and" << ext_ee_id << ", specified id is " << ee_id << ".");
    // If current grasp is not of movable ee then according to the current ws set to corresponding ee.
    KDL::Frame temporary_frame;
    tf::poseMsgToKDL(pose,temporary_frame);
    // Get current workspaceID (after converting pose msg to KDL frame).
    workspace_id current_ws = db_mapper_.getWorkspaceIDFromPose(temporary_frame);
    switch (current_ws) {
        case 1: ee_id = wall_ee_id; break;
        case 2: ee_id = edge_ee_id; break;
        case 3: ee_id = table_ee_id; break;
        case 4: ee_id = table_ee_id; break;
        case 5: ee_id = table_ee_id; break;
        case 6: ee_id = table_ee_id; break;
        case 7: ee_id = edge_ee_id; break;
        case 8: ee_id = ext_ee_id; break;
        default: ee_id = table_ee_id; break; // Set table grasp by default.
    }
    
    // Two frames: one for the current object pose and the other for the grasp pose (pose of ee for that grasp).
    KDL::Frame obj_frame,grasp_frame;
    tf::poseMsgToKDL(pose,obj_frame);
    
    int best_grasp = -1;
    double closeness = -2.0;
    double other_closeness = -2.0;

    /*
    The following grasp detection works well also if there is only one grasp defined per object side.
    */

    // Recall that a table grasp is defined by the axis which is perpendicular to the table.
    double global_dot_product_x;
    double global_dot_product_y;
    double global_dot_product_z;
    double local_dot_product; // To be used once the perpendicular axis is found.
    double other_dot_product; // To be used to distinguish between grasps with same main axis.

    ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Pose: " << std::endl << pose);
    
    for (auto item:db_mapper_.Grasps)
    {
        auto ee_id_tmp = item.second.ee_id;
        auto obj_id_tmp = item.second.obj_id;

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
	      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unable to deserialize grasp entry : object" + std::to_string(object_id) + "/grasp" + std::to_string((int)item.first));
	    
        // Modification: Get the two rotations from the grasp and object frames.
        KDL::Rotation R_object = obj_frame.M;
        KDL::Rotation R_grasp = grasp_frame.M;

        // Compute global z axis and all axes from each rotation.
        KDL::Vector global_z_axis(0, 0, 1);
        KDL::Vector x_obj = R_object.UnitX(); // x axis of object pose.
        KDL::Vector x_grasp = R_grasp.UnitX(); // x axis of current grasp pose.
        KDL::Vector y_obj = R_object.UnitY(); // y axis of object pose.
        KDL::Vector y_grasp = R_grasp.UnitY(); // y axis of current grasp pose.
        KDL::Vector z_obj = R_object.UnitZ(); // z axis of object pose.
        KDL::Vector z_grasp = R_grasp.UnitZ(); // z axis of current grasp pose.

        // Dot products between global z axis and object x, y and z axes.
        global_dot_product_x = std::abs(dot(global_z_axis, x_obj));
        global_dot_product_y = std::abs(dot(global_z_axis, y_obj));
        global_dot_product_z = std::abs(dot(global_z_axis, z_obj));

        // Looking for the best dot product to choose the case.
        // Case 1) x_obj perpendicular to table
        if(global_dot_product_x > global_dot_product_y && global_dot_product_x > global_dot_product_z)
        {
            std::cout << "I ENTERED x_obj" << std::endl;
            local_dot_product= dot(x_obj, x_grasp);
            if((closeness < -1.0) || (local_dot_product > closeness))
            {
                closeness = local_dot_product;
                other_dot_product = dot(y_obj, y_grasp);
                other_closeness = other_dot_product;
                best_grasp = item.first;
            }
            else if((closeness < -1.0) || (local_dot_product == closeness))
            {
                closeness = local_dot_product;
                other_dot_product = dot(y_obj, y_grasp);
                if((other_closeness < -1.0) || (other_dot_product > other_closeness)){
                    other_closeness = other_dot_product;
                    best_grasp = item.first;
                }
            }
        }
        // Case 2) y_obj perpendicular to table
        else if(global_dot_product_y > global_dot_product_x && global_dot_product_y > global_dot_product_z)
        {
            std::cout << "I ENTERED y_obj" << std::endl;
            local_dot_product= dot(y_obj, y_grasp);
            if((closeness < -1.0) || (local_dot_product > closeness))
            {
                closeness = local_dot_product;
                other_dot_product = dot(z_obj, z_grasp);
                other_closeness = other_dot_product;
                best_grasp = item.first;
            }
            else if((closeness < -1.0) || (local_dot_product == closeness))
            {
                closeness = local_dot_product;
                other_dot_product = dot(z_obj, z_grasp);
                if((other_closeness < -1.0) || (other_dot_product > other_closeness)){
                    other_closeness = other_dot_product;
                    best_grasp = item.first;
                }
            }
        }
        // Case 3) z_obj perpendicular to table
        else if(global_dot_product_z > global_dot_product_x && global_dot_product_z > global_dot_product_y)
        {
            std::cout << "I ENTERED z_obj" << std::endl;
            local_dot_product= dot(z_obj, z_grasp);
            if((closeness < -1.0) || (local_dot_product > closeness))
            {
                closeness = local_dot_product;
                other_dot_product = dot(x_obj, x_grasp);
                other_closeness = other_dot_product;
                best_grasp = item.first;
            }
            else if((closeness < -1.0) || (local_dot_product == closeness))
            {
                closeness = local_dot_product;
                other_dot_product = dot(x_obj, x_grasp);
                if((other_closeness < -1.0) || (other_dot_product > other_closeness)){
                    other_closeness = other_dot_product;
                    best_grasp = item.first;
                }
            }
        }
        else
        {
            ROS_ERROR("The given object pose is ambigous for grasp detection, please change object pose a little bit.");
        }
	}
    }
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Best grasp found: " << best_grasp);
    return best_grasp;
}

int getting_info_state::get_object_id(std::string obj_name)
{
  for(auto item:db_mapper_.Objects)
  {
    std::string db_obj_name(item.second.name);
    if(obj_name.compare(db_obj_name) == 0)
      return item.first;
  }
  return -1;
}

void getting_info_state::gui_target_set_callback(const dual_manipulation_shared::gui_target_response::ConstPtr& msg)
{
    if (target_set.load()) return;
    
    ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : Target set to "<<msg->target_pose.position.x<<' '<<msg->target_pose.position.y<<' '<<msg->target_pose.position.z<<' '<<msg->target_pose.orientation.x<<' '<<msg->target_pose.orientation.y<<' '<<msg->target_pose.orientation.z<<' '<<msg->target_pose.orientation.w);

    data_.source_position = msg->source_pose; //user selects which detected object is the source from the gui
    data_.target_position = msg->target_pose;
    data_.obj_id = msg->obj_id;
    data_.object_name = msg->name;
    // TODO: ask for desired target end-effector; maybe even for desired final grasp?
    // 
    data_.source_grasp=get_grasp_id_from_database(data_.obj_id,data_.source_position);
    data_.target_grasp=get_grasp_id_from_database(data_.obj_id,data_.target_position);
    data_.bad_checksinglegrasp = false;

    pacman_vision_comm::track_object srv;
    srv.request.name = data_.object_name;

    if(!tracker_start_client.call(srv))
    {
        if(use_vision)
        {
            ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : unable to call service " << tracker_start_client.getService());
        }
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
        ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Answer: ("<<(bool)srv.response.ack<<")");
        if(srv.response.ack)
            ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Waiting for target from user");
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Failed to call service " << gui_target_client.getService());
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
        // try setting the object directly in the planner object
        if(!data_.planner.set_object(data_.obj_id,data_.object_name))
        {
            dual_manipulation_shared::planner_service srv;
            srv.request.command="set object";
            srv.request.time = ros::Time::now().toSec();
            srv.request.object_id=data_.obj_id;
            srv.request.object_name=data_.object_name;
                
            if (!planner_client.exists())
            {
                ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Service does not exist: " << planner_client.getService());
                failed = true;
                return;
            }
            else if (planner_client.call(srv))
            {
                ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Object id set to " << srv.request.object_id << ", planner returned " << srv.response.ack);
            }
            else
            {
                ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Failed to call service " << planner_client.getService());
                failed=true;
                return;
            }
        }
        
	//NOTE: the following behavior is needed when more than a single object is in the scene: remove the one we are planning for in order to do sem2cart conversion,
	//      then insert the object back in the scene
	dual_manipulation_shared::scene_object_service srv_obj;
	srv_obj.request.command = "remove";
	srv_obj.request.object_db_id = data_.obj_id;
	// NOTE: this should be unique, while we can have more objects with the same "object_db_id"
	srv_obj.request.attObject.object.id = data_.object_name;
	if (scene_object_client.call(srv_obj))
	{
        ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << srv_obj.request.command << " object " << srv_obj.request.attObject.object.id << " request accepted: " << (int)srv_obj.response.ack);
	}
	else
	{
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Failed to call service " << scene_object_client.getService() << ": " << srv_obj.request.command << " " << srv_obj.request.attObject.object.id);
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
