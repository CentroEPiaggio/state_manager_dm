#include "../include/ik_checking_grasp_substate.h"
#include "../../planning/src/gml2lgf/node.h"
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
    // make sure we are considering only table grasps
    if(ee_id != table_ee_id)
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : only considering table grasps for now with id " << table_ee_id << ", specified id was " << ee_id << " instead");
        ee_id = table_ee_id;
    }
    
    KDL::Frame obj_frame,grasp_frame;
    tf::poseMsgToKDL(pose,obj_frame);
    double x,y,z,w;
    
    int best_grasp = -1;
    double closeness = -1.0;
    
    for (auto item:database_.Grasps)
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
  double xs,ys;
  xs = World_RealObjPose.p.x();
  ys = World_RealObjPose.p.y();
  bool ws_found = false;
  for (auto workspace: database_.WorkspaceGeometry)
  {
      std::vector<Point> temp;
      for (auto point : workspace.second)
	  temp.emplace_back(point.first,point.second);
      if (geom.point_in_ordered_polygon(xs,ys,temp))
      {
	  node.next_workspace_id = workspace.first;
	  node.current_workspace_id = workspace.first;
	  ws_found = true;
	  break;
      }
  }
  if(!ws_found)
  {
    ROS_ERROR_STREAM("ik_checking_grasp_substate::run : actual object pose is out of the semantic workspace! returning...");
    failed_ = true;
    return;
  }
  
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
