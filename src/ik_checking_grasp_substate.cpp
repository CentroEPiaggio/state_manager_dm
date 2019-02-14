#include "../include/ik_checking_grasp_substate.h"
#include "../../planning_dm/src/gml2lgf/node.h"
#include <tf_conversions/tf_kdl.h>
#include "dual_manipulation_shared/serialization_utils.h"
#include <dual_manipulation_shared/parsing_utils.h>

#define OBJ_GRASP_FACTOR 1000
#define LONG_TIME_NO_SEE 3.0
#define POSITION_TOLERANCE 0.005
#define ORIENTATION_TOLERANCE 0.005
#define MAX_POSITION_DISTANCE 0.5
#define MAX_ORIENTATION_DISTANCE 0.5

#define CLASS_NAMESPACE "ik_checking_grasp_substate::"
#define TRACKER_TESTING 1

ik_checking_grasp_substate::ik_checking_grasp_substate(ik_shared_memory& data):data_(data),database_(data.db_mapper),converter_(data.db_mapper)
{
    ros::NodeHandle n;
    XmlRpc::XmlRpcValue get_info_params;
    if (n.getParam("dual_manipulation_parameters", get_info_params)) parseParameters(get_info_params);
}

void ik_checking_grasp_substate::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    parseSingleParameter(params,table_ee_id,"table_ee_id");
}

std::map< ik_transition, bool > ik_checking_grasp_substate::getResults()
{
    std::map< ik_transition, bool > results;
#if !TRACKER_TESTING
    if(soft_failed_)
      results[ik_transition::soft_fail] = true;
    else if(need_replanning_)
      results[ik_transition::plan] = true;
    else if(failed_)
      results[ik_transition::fail] = true;
    else if(is_complete_)
#endif
      results[ik_transition::check_done] = true;
    return results;
}

int ik_checking_grasp_substate::get_grasp_id_from_database(int object_id, geometry_msgs::Pose pose, int ee_id)
{
    // make sure we are considering only non movable ee grasps for now (as of 10/07/2017)

    ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : Only considering non movable end-effector grasps for now with ids " << table_ee_id << ", " << edge_ee_id << ", " << wall_ee_id << "and" << ext_ee_id << ", specified id is " << ee_id << ".");
    // If current grasp is not of movable ee then according to the current ws set to corresponding ee.
    KDL::Frame temporary_frame;
    tf::poseMsgToKDL(pose,temporary_frame);
    // Get current workspaceID (after converting pose msg to KDL frame).
    workspace_id current_ws = database_.getWorkspaceIDFromPose(temporary_frame);
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

    for (auto item:database_.Grasps)
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
	      ROS_WARN_STREAM("Unable to deserialize grasp entry : object" + std::to_string(object_id) + "/grasp" + std::to_string((int)item.first));
	    
	    // Modification: Get the two rotations from the grasp and object frames.
        KDL::Rotation R_object = obj_frame.M;
        KDL::Rotation R_grasp = grasp_frame.M;

        // Get Quaternions from Residual Rotation
        KDL::Rotation R_residual = R_object*R_grasp.Inverse();
        double xq, yq, zq, wq;
        R_residual.GetQuaternion(xq, yq, zq, wq);

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
    
    ROS_INFO_STREAM("Best grasp found: " << best_grasp);
    return best_grasp;
}

void ik_checking_grasp_substate::run()
{
  KDL::Frame World_ExpectedObjPose;
  KDL::Frame World_RealObjPose;
  
  tf::poseMsgToKDL(data_.cartesian_plan->at(data_.next_plan).second.cartesian_task,World_ExpectedObjPose);
  
  tf::StampedTransform World_RealObjTf;
  double timeout = 1.0;
  if(!tf_listener_.waitForTransform("world",*data_.object_name + "_tracked",ros::Time(0),ros::Duration(timeout)))
  {
    // TODO: throw some error? continue? ignore everything?
    ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : unable to retrieve TF for \'" << *data_.object_name << "_tracked\' after " << timeout << "s, returning...");
    is_complete_ = true;
    return;
    World_RealObjTf.setIdentity();
  }
  else
    tf_listener_.lookupTransform("world",*data_.object_name + "_tracked", ros::Time(0), World_RealObjTf);
  
  ros::Duration timeDiff = ros::Time::now() - World_RealObjTf.stamp_;
  if(timeDiff.toSec() > LONG_TIME_NO_SEE)
  {
    ROS_ERROR_STREAM("ik_checking_grasp_substate::run : unable to see the object! Returning...");
    soft_failed_ = true;
    return;
  }
  
  // tf::poseTFToKDL(World_RealObjTf,World_RealObjPose);
  KDL::Twist xi;
  xi = KDL::diff(World_RealObjPose,World_ExpectedObjPose);
  if(KDL::Equal(xi.vel,KDL::Vector::Zero(),POSITION_TOLERANCE) && KDL::Equal(xi.rot,KDL::Vector::Zero(),ORIENTATION_TOLERANCE))
  {
    ROS_INFO_STREAM("ik_checking_grasp_substate::run : actual object pose is close enough to the expected one - not changing anything!");
    is_complete_ = true;
    return;
  }
  
  // if (!KDL::Equal(World_RealObjPose.p,World_ExpectedObjPose.p,MAX_POSITION_DISTANCE) || !KDL::Equal(World_RealObjPose.M,World_ExpectedObjPose.M,MAX_ORIENTATION_DISTANCE))
  // {
  //   failed_ = true;
  //   return;
  // }

  ROS_INFO_STREAM("ik_checking_grasp_substate::run : actual object pose is different from the expected one - let's do some more checking...");
  
  // build a partial shared_memory and a full node_info objects
  shared_memory data;
  data.obj_id = *data_.obj_id;
  tf::poseKDLToMsg(World_RealObjPose,data.source_position);
  node_info node;
  node.next_ee_id = data_.cartesian_plan->at(data_.next_plan).first;
  node.next_grasp_id = data_.cartesian_plan->at(data_.next_plan).second.ee_grasp_id;
  
  // are we still inside the workspace?
  workspace_id ws_id = database_.getWorkspaceIDFromPose(World_RealObjPose);
  bool ws_found = (ws_id != -1);
  
  if(!ws_found)
  {
    ROS_ERROR_STREAM("ik_checking_grasp_substate::run : actual object pose is out of the semantic workspace! returning...");
    failed_ = true;
    return;
  }
  node.next_workspace_id = ws_id;
  node.current_workspace_id = ws_id;
  
  // if current end-effector is not movable, it's the table
  if(data_.cartesian_plan->at(data_.next_plan+1).second.command != cartesian_commands::UNGRASP)
  {
    //TODO: make this more general!!!
    node.current_ee_id = 3;
    node.current_grasp_id = get_grasp_id_from_database(data.obj_id,data.source_position,node.current_ee_id);
  }
  else
  {
    node.current_ee_id = data_.cartesian_plan->at(data_.next_plan+1).first;
    node.current_grasp_id = data_.cartesian_plan->at(data_.next_plan+1).second.ee_grasp_id;
  }
  
  dual_manipulation_shared::planner_item filtered_source_nodes,filtered_target_nodes;
  bool first_node = true, last_node = false;
  KDL::Frame unused;
  // try at first to move next end-effector only
  node.type = node_properties::GRASP;
  if(converter_.checkSingleGrasp(unused,node,data,first_node,last_node,filtered_source_nodes,filtered_target_nodes))
  {
    ROS_INFO_STREAM("ik_checking_grasp_substate::run : OK to change only next grasp position!");
    
#if !TRACKER_TESTING
    // also add a waypoint here?
    tf::poseKDLToMsg(World_RealObjPose,data_.cartesian_plan->at(data_.next_plan).second.cartesian_task);
#endif
    is_complete_ = true;
    return;
  }
  
  // if current end-effector is not movable, I failed
  if(data_.cartesian_plan->at(data_.next_plan+1).second.command != cartesian_commands::UNGRASP)
  {
    ROS_ERROR_STREAM("ik_checking_grasp_substate::run : unable to find a good configuration to grasp the object from a non-movable end-effector, returning...");
    failed_ = true;
    return;
  }
  
  // it didn't work: I can still try to move both end-effectors...
  node.type = node_properties::EXCHANGE_GRASP;
  KDL::Frame World_Object;
  if(converter_.compute_intergrasp_orientation(World_Object,node,data.obj_id))
  {
    // move both...
    ROS_INFO_STREAM("ik_checking_grasp_substate::run : OK adding waypoints to both end-effectors to perform next grasp!");
    
    // get grasp matrixes
    Object_GraspMatrixes Object;
    semantic_to_cartesian_converter::getGraspMatrixes(data.obj_id,node,Object);

#if !TRACKER_TESTING
    // change grasping pose
    tf::poseKDLToMsg(World_Object,data_.cartesian_plan->at(data_.next_plan).second.cartesian_task);
    // change ungrasping pose
    tf::poseKDLToMsg(World_Object,data_.cartesian_plan->at(data_.next_plan+1).second.cartesian_task);
    
    // insert before grasp-ungrasp two new parallel move commands
    cartesian_command move_command(cartesian_commands::MOVE,0,-1); // parallelize
    KDL::Frame World_GraspFirstEE(World_Object*Object.PostGraspFirstEE);
    tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
    data_.cartesian_plan->insert(data_.cartesian_plan->begin() + data_.next_plan,std::make_pair(node.current_ee_id,move_command)); //move the first
    cartesian_command second_move_command(cartesian_commands::MOVE,1,-1); // do NOT parallelize
    KDL::Frame World_GraspSecondEE(World_Object*Object.PreGraspSecondEE);
    tf::poseKDLToMsg(World_GraspSecondEE,second_move_command.cartesian_task);
    data_.cartesian_plan->insert(data_.cartesian_plan->begin() + data_.next_plan + 1,std::make_pair(node.next_ee_id,second_move_command)); //move the next
#endif

    need_replanning_ = true;
    return;
  }
  
  // nothing worked...
  ROS_ERROR_STREAM("ik_checking_grasp_substate::run : unable to find a good configuration to perform the object passing, returning...");
  failed_ = true;
  return;
}

bool ik_checking_grasp_substate::isComplete()
{
    return is_complete_ || failed_ || soft_failed_ || need_replanning_;
}

std::string ik_checking_grasp_substate::get_type()
{
    return "ik_checking_grasp_substate";
}

void ik_checking_grasp_substate::reset()
{
  need_replanning_ = false;
  is_complete_ = false;
  failed_ = false;
  soft_failed_ = false;
}
