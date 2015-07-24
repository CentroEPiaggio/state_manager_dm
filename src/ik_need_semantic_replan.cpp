#include "../include/ik_need_semantic_replan.h"
#include <dual_manipulation_shared/serialization_utils.h>
#include <dual_manipulation_shared/grasp_trajectory.h>
#include <tf_conversions/tf_kdl.h>

#define OBJ_GRASP_FACTOR 1000
#define CLASS_NAMESPACE "ik_need_semantic_replan::"

ik_need_semantic_replan::ik_need_semantic_replan(ik_shared_memory& subdata, shared_memory& data):data_(data),database_(data.db_mapper),subdata_(subdata)
{

}

void ik_need_semantic_replan::ask_semantic_replan()
{
  // failure condition: I've not been able to do even the first step > stop here!
  if(subdata_.next_plan == 0)
  {
    failed_ = false;
    return;
  }
  
  cartesian_command c; c.cartesian_task;
  bool movable_ee = false;
  int grasp_id = data_.source_grasp;
  geometry_msgs::Pose World_Object;
  World_Object.orientation.w = 1;
  
  for(int i=0; i<subdata_.next_plan; i++)
    if(subdata_.cartesian_plan->at(subdata_.next_plan).second.command == cartesian_commands::GRASP)
    {
      movable_ee = true;
      grasp_id = subdata_.cartesian_plan->at(i).second.ee_grasp_id;
    }
    else if(subdata_.cartesian_plan->at(subdata_.next_plan).second.command == cartesian_commands::UNGRASP)
    {
      movable_ee = false;
      World_Object = subdata_.cartesian_plan->at(subdata_.next_plan).second.cartesian_task;
    }
  
  if(!movable_ee)
  {
    grasp_id = get_grasp_id_from_db(data_.obj_id,World_Object,-1);
  }
  
  data_.source_grasp = grasp_id;
  data_.source_position = World_Object;
  
  need_replan_ = true;
}

std::map<ik_transition,bool> ik_need_semantic_replan::getResults()
{
  std::map<ik_transition,bool> results;
  if(failed_)
    results[ik_transition::fail] = true;
  else if(need_replan_)
    results[ik_transition::need_replan] = true;
  return results;
}

int ik_need_semantic_replan::get_grasp_id_from_db(int object_id, const geometry_msgs::Pose& pose, int ee_id)
{
    // make sure we are considering only table grasps
    if(ee_id != 3)
    {
      ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : only considering table grasps for now !! end-effector id changed from " << ee_id << " to " << 3);
      ee_id = 3;
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
	
	// for each grasp, if the end-effector is the right one
	if (((int)ee_id_tmp == ee_id) && ((int)obj_id_tmp == object_id))
	{
	    // deserialize grasp
	    dual_manipulation_shared::grasp_trajectory traj;
	    int grasp = (int)item.first;
	    grasp = grasp % OBJ_GRASP_FACTOR;
	    // ROS_INFO_STREAM("Deserializing object" + std::to_string(object_id) + "/grasp" + std::to_string(grasp));
	    bool ok = deserialize_ik(traj,"object" + std::to_string(object_id) + "/grasp" + std::to_string(grasp));
	    if (ok)
	      tf::poseMsgToKDL(traj.ee_pose.back(),grasp_frame);
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
    
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : more similar grasp found > " << best_grasp);
    return best_grasp;
}
