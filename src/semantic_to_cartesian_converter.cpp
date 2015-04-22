#include "../include/semantic_to_cartesian_converter.h"
#include <shared_memory.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <vector>
#include <kdl_conversions/kdl_msg.h>
#include "dual_manipulation_shared/serialization_utils.h"
#include <math.h>
#include <algorithm>    // std::min_element, std::max_element
#include <std_msgs/String.h>

#define SUPERHACK 0
#define HIGH 0.5
#define LOW 0.06
#define ANGLE_STEPS 4.0 // 6.0
#define BIMANUAL_IK_ATTEMPTS 5
#define BIMANUAL_IK_TIMEOUT 0.002

#define DEBUG 0 // if 1, print some more information

// TODO: write here a good configuration!!!
static std::vector<double> left_arm_pos={0.1,0.1,0.1,0.1,0.1,0.1,0.1};
static std::vector<double> right_arm_pos={0.1,0.1,0.1,0.1,0.1,0.1,0.1};

semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database)
{
  this->database=database;
  double t=(1.0+sqrt(5.0))/2.0;
  for (double angle=-M_PI+M_PI/ANGLE_STEPS;angle<M_PI-0.001;angle=angle+2.0*M_PI/ANGLE_STEPS)
  {
    for (int i=0;i<4;i++)
    {
      sphere_sampling.emplace_back(KDL::Rotation::Rot(KDL::Vector(0, i&2?-1:1, i&1?-t:t),angle));
    }
    for (int i=4;i<8;i++)
    {
      sphere_sampling.emplace_back(KDL::Rotation::Rot(KDL::Vector(i&2?-1:1, i&1?-t:t, 0),angle));
    }
    for (int i=8;i<12;i++)
    {
      sphere_sampling.emplace_back(KDL::Rotation::Rot(KDL::Vector(i&1?-t:t, 0, i&2?-1:1),angle));
    }
  }
  
  ik_check_capability = new dual_manipulation::ik_control::ikCheckCapability();
}

bool semantic_to_cartesian_converter::getPreGraspMatrix(object_id object, grasp_id grasp, KDL::Frame& Object_EE) const
{
    //TODO CACHE VALUES
    dual_manipulation_shared::ik_service srv;
    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
    if (ok)
        tf::poseMsgToKDL(srv.request.ee_pose.front(),Object_EE);
    
#if SUPERHACK
    // get away a little more
    Object_EE.p = Object_EE.p * 1.3;
#endif
    
    return ok;
}

bool semantic_to_cartesian_converter::getGraspMatrix(object_id object, grasp_id grasp, KDL::Frame& Object_EE) const
{
    //TODO CACHE VALUES
    dual_manipulation_shared::ik_service srv;
    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
    if (ok)
        tf::poseMsgToKDL(srv.request.ee_pose.back(),Object_EE);
    
    return ok;
}

bool semantic_to_cartesian_converter::getPostGraspMatrix(object_id object, grasp_id grasp, KDL::Frame& Object_EE) const
{
    //TODO CACHE VALUES
    dual_manipulation_shared::ik_service srv;
    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
    if (ok)
    {
        tf::poseMsgToKDL(srv.request.attObject.object.mesh_poses.front(),Object_EE);
        Object_EE = Object_EE.Inverse();
    }
    return ok;
}


void semantic_to_cartesian_converter::compute_centroid(double& centroid_x,double& centroid_y,double& centroid_z, const node_info& node) const
{
    centroid_x=0;
    centroid_y=0;
    auto w_id=node.next_workspace_id;
    for (auto workspace: database.WorkspaceGeometry.at(w_id))
    {
        centroid_x+=workspace.first;
        centroid_y+=workspace.second;
    }
    centroid_x=centroid_x/database.WorkspaceGeometry.at(w_id).size();
    centroid_y=centroid_y/database.WorkspaceGeometry.at(w_id).size();
    if (node.type==node_properties::MOVABLE_TO_MOVABLE) //both ee are movable: change above ground
    {centroid_z=HIGH;}
    else //one is movable, change on ground
    {centroid_z=LOW;}
    
    return;
}

node_info semantic_to_cartesian_converter::find_node_properties(const std::vector< dual_manipulation_shared::planner_item >& path, const std::vector< dual_manipulation_shared::planner_item >::const_iterator& node, std::vector< dual_manipulation_shared::planner_item >::const_iterator& next_node) const
{
    node_info result;
    auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
    bool movable=std::get<1>(database.EndEffectors.at(ee_id));
    // 3.3) Searching for the next node with a different end effector than the current one
    bool found=false;
    endeffector_id next_ee_id=-1;
    workspace_id next_workspace_id=-1;
    bool next_movable=false;
    while (!found && next_node!=path.end()) //Here we simplify (i.e. remove) all the transitions such that ee_id=next_ee_id and workspace_id != next_workspace_id
    {
        next_node++;
        if (next_node!=path.end())
        {
            next_ee_id = std::get<1>(database.Grasps.at(next_node->grasp_id));
            next_workspace_id = next_node->workspace_id;
            next_movable=std::get<1>(database.EndEffectors.at(next_ee_id));
            if (ee_id==next_ee_id) continue;
            else
            {
                found=true;
                break;
            }
        }
    }
    if (found)
    {
        if (movable && !next_movable) result.type=node_properties::MOVABLE_TO_FIXED;          //found, one is movable, change on ground
        if (!movable && next_movable) result.type=node_properties::FIXED_TO_MOVABLE;          //found, one is movable, change on ground
        if (movable && next_movable) result.type=node_properties::MOVABLE_TO_MOVABLE;        //found, both ee are movable: change above ground
        if (!movable && !next_movable) result.type=node_properties::FIXED_TO_FIXED;            //if (!movable && !next_movable)
    }
    else
    {
        if (!movable) result.type=node_properties::LAST_EE_FIXED;             //3.4.1) if not found, than ee_id is the last end effector in the path //not found not movable
        else result.type=node_properties::LAST_EE_MOVABLE;            //else //not found->last e.e, movable
    }
    result.current_ee_id=ee_id;
    result.next_ee_id=next_ee_id;
    result.current_grasp_id=node->grasp_id;
    result.next_grasp_id=next_node->grasp_id;
    result.current_workspace_id=node->workspace_id;
    result.next_workspace_id=next_workspace_id;
    return result;
}

void semantic_to_cartesian_converter::addNewFilteredArc(const node_info& node, std::vector< dual_manipulation_shared::planner_item >& filtered_source_nodes, std::vector< dual_manipulation_shared::planner_item >& filtered_target_nodes) const
{
        dual_manipulation_shared::planner_item source_node,target_node;
        source_node.grasp_id=node.current_grasp_id;
        source_node.workspace_id=node.next_workspace_id;//THIS IS INTENTIONAL!! We remove the intergrasp transition arc in the target workspace
        target_node.grasp_id=node.next_grasp_id;
        target_node.workspace_id=node.next_workspace_id;//THIS IS INTENTIONAL!! We remove the intergrasp transition arc in the target workspace
        filtered_source_nodes.push_back(source_node);
        filtered_target_nodes.push_back(target_node);
        filtered_source_nodes.push_back(target_node);
        filtered_target_nodes.push_back(source_node);
}

bool semantic_to_cartesian_converter::check_ik(std::string current_ee_name, KDL::Frame World_FirstEE, std::string next_ee_name, KDL::Frame World_SecondEE, std::vector< std::vector< double > >& results) const
{
  std::vector<geometry_msgs::Pose> ee_poses;
  ee_poses.resize(2);
  geometry_msgs::Pose &left_pose = ee_poses.at(0), &right_pose = ee_poses.at(1);
  if(current_ee_name == "left_hand" && next_ee_name == "right_hand")
  {
    tf::poseKDLToMsg(World_FirstEE,left_pose);
    tf::poseKDLToMsg(World_SecondEE,right_pose);
  }
  else if(current_ee_name == "right_hand" && next_ee_name == "left_hand")
  {
    tf::poseKDLToMsg(World_FirstEE,right_pose);
    tf::poseKDLToMsg(World_SecondEE,left_pose);
  }
  else
  {
    ROS_ERROR_STREAM("semantic_to_cartesian_converter::check_ik : unknown end-effector couple <" << current_ee_name << " | " << next_ee_name << ">");
    ROS_ERROR("At now, only \"left_hand\" and \"right_hand\" are supported!");
    return false;
  }
  
#if DEBUG
  std::cout << "check_ik: left_hand in " << ee_poses.at(0) << " and right_hand in " << ee_poses.at(1) << std::endl;
#endif
  
  std::vector <double > initial_guess = std::vector<double>();
  bool check_collisions = true;
  bool return_approximate_solution = false;
  unsigned int attempts = BIMANUAL_IK_ATTEMPTS;
  double timeout = BIMANUAL_IK_TIMEOUT;
  // std::map <std::string, std::string > allowed_collisions = std::map< std::string, std::string >();
  
  ik_check_capability->reset_robot_state();
  bool found_ik = ik_check_capability->find_group_ik("both_hands",ee_poses,results,initial_guess,check_collisions,return_approximate_solution,attempts,timeout/*,allowed_collisions*/);
  
  return found_ik;
}

bool semantic_to_cartesian_converter::check_ik(std::string ee_name, KDL::Frame World_EE) const
{
  std::vector<geometry_msgs::Pose> ee_poses;
  ee_poses.resize(1);
  geometry_msgs::Pose &ee_pose = ee_poses.at(0);
  if(ee_name != "left_hand" && ee_name != "right_hand")
  {
    ROS_ERROR_STREAM("semantic_to_cartesian_converter::check_ik : unknown end-effector <" << ee_name << ">");
    ROS_ERROR("At now, only \"left_hand\" and \"right_hand\" are supported!");
    return false;
  }
  tf::poseKDLToMsg(World_EE,ee_pose);
  
#if DEBUG
  std::cout << "check_ik: " << ee_name << " in " << ee_pose << std::endl;
#endif
  
  std::vector<std::vector<double>> results;
  results.resize(1);
  
//   std::vector <double > initial_guess = std::vector<double>();
//   bool check_collisions = false;
//   bool return_approximate_solution = true;
//   unsigned int attempts = 10;
//   double timeout = 0.005;
//   std::map <std::string, std::string > allowed_collisions = std::map< std::string, std::string >();
  
  ik_check_capability->reset_robot_state();
  bool found_ik = ik_check_capability->find_group_ik(ee_name,ee_poses,results/*,initial_guess,check_collisions,return_approximate_solution,attempts,timeout,allowed_collisions*/);
  
  return found_ik;
}

bool semantic_to_cartesian_converter::compute_intergrasp_orientation(KDL::Vector World_centroid, KDL::Frame& World_Object, const node_info& node, object_id object, int aggiuntivo) const
{
    KDL::Frame World_Centroid_f(World_centroid);
    KDL::Frame Object_PreGraspFirstEE,Object_PreGraspSecondEE;
    KDL::Frame Object_PostGraspFirstEE,Object_PostGraspSecondEE;
    KDL::Frame Object_GraspFirstEE,Object_GraspSecondEE;
    auto current_ee_name=std::get<0>(database.EndEffectors.at(node.current_ee_id));
    auto next_ee_name=std::get<0>(database.EndEffectors.at(node.next_ee_id));
    bool ok=getPostGraspMatrix(object,node.current_grasp_id,Object_PostGraspFirstEE);
    if (!ok) 
    {
	std::cout<<"Error in getting postgrasp matrix for object "<<object<<" and ee "<<node.current_ee_id<<std::endl;
	abort();
    }
    ok=getPostGraspMatrix(object,node.next_grasp_id,Object_PostGraspSecondEE);
    if (!ok) 
    {
	std::cout<<"Error in getting postgrasp matrix for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	abort();
    }
    ok = getPreGraspMatrix(object,node.current_grasp_id,Object_PreGraspFirstEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.current_ee_id<<std::endl;
	abort();
    }
    ok = getPreGraspMatrix(object,node.next_grasp_id,Object_PreGraspSecondEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	abort();
    }
    ok = getGraspMatrix(object,node.current_grasp_id,Object_GraspFirstEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.current_ee_id<<std::endl;
	abort();
    }
    ok = getGraspMatrix(object,node.next_grasp_id,Object_GraspSecondEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	abort();
    }
    
#if DEBUG
    std::cout << current_ee_name << " | " << next_ee_name << std::endl;
    geometry_msgs::Pose ee_pose;
    tf::poseKDLToMsg(World_Centroid_f,ee_pose);
    std::cout << "World_Centroid_f: " << ee_pose << std::endl;
    tf::poseKDLToMsg(Object_FirstEE,ee_pose);
    std::cout << "Object_FirstEE: " << ee_pose << std::endl;
    tf::poseKDLToMsg(Object_SecondEE,ee_pose);
    std::cout << "Object_SecondEE: " << ee_pose << std::endl;
    tf::poseKDLToMsg(Object_GraspFirstEE,ee_pose);
    std::cout << "Object_GraspFirstEE: " << ee_pose << std::endl;
    tf::poseKDLToMsg(Object_GraspSecondEE,ee_pose);
    std::cout << "Object_GraspSecondEE: " << ee_pose << std::endl;
#endif
    
    if (node.type==node_properties::MOVABLE_TO_MOVABLE)
    {
        std::vector<double> joint_pose_norm;
	//TODO: pre-align the grasp to a good configuration, then go through sphere_sampling in a spiral manner, and stop when you first find a feasible one
	for (auto& rot: sphere_sampling)
	{
	    KDL::Frame World_Object(rot,World_centroid);
	    std::vector<std::vector<double>> results;
	    results.resize(2);
	    std::vector<double> &result_first = results.at(0), &result_second = results.at(1);
	    bool ik_ok=check_ik(current_ee_name,World_Object*Object_PostGraspFirstEE,next_ee_name,World_Object*Object_PreGraspSecondEE, results);
    	    bool ik_ok1=check_ik(current_ee_name,World_Object*Object_PostGraspFirstEE,next_ee_name,World_Object*Object_GraspSecondEE, results);
	    if (!(ik_ok && ik_ok1)) 
	    {
	      joint_pose_norm.push_back(1000);
	      continue;
	    }

	    double norm=0;
	    // NOTE: the first result when both hands are used is for left_hand, the second for the right one (lexical order)
	    for (int j=0;j<left_arm_pos.size();j++)
	      norm=norm+pow(result_first[j]-left_arm_pos.at(j),2);
	    for (int j=0;j<right_arm_pos.size();j++)
	      norm=norm+pow(result_second[j]-right_arm_pos.at(j),2);
	    joint_pose_norm.push_back(norm);
	}
	auto it=std::min_element(joint_pose_norm.begin(),joint_pose_norm.end());
	if (*it==1000)
	  return false;
	auto best_rot=sphere_sampling.at(it-joint_pose_norm.begin());
	World_Object=KDL::Frame(best_rot,World_centroid);
#if SUPERHACK
        World_Object.M = fine_tuning[aggiuntivo].M*World_Object.M;
        World_Object.p = World_Object.p + fine_tuning[aggiuntivo].p;
        // TODO: take the above code out
#endif
	return true;
    }
    else if (node.type==node_properties::FIXED_TO_MOVABLE)
    {
	World_Object = World_Centroid_f*(Object_PostGraspFirstEE.Inverse());
	if(check_ik(next_ee_name,World_Object*Object_PreGraspSecondEE))
	  if(check_ik(next_ee_name,World_Object*Object_GraspSecondEE))
	    return true;
	return false;
    }
    else if (node.type==node_properties::MOVABLE_TO_FIXED)
    {
	World_Object = World_Centroid_f*(Object_GraspSecondEE.Inverse());
	  if(check_ik(current_ee_name,World_Object*Object_GraspFirstEE))
	if(check_ik(current_ee_name,World_Object*Object_PostGraspFirstEE))
	    return true;
	return false;
    }
    else 
    {
      std::cout<<"SUPER ERROR"<<std::endl;
      return false;
    }
}
  
  bool semantic_to_cartesian_converter::convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, std::vector< dual_manipulation_shared::planner_item >& filtered_source_nodes, std::vector< dual_manipulation_shared::planner_item >& filtered_target_nodes) const
{
    // 1) Clearing result vector
    result.clear();

    // 3) Start of the main conversion loop
    for (auto node_it=path.begin();node_it!=path.end();)//++node)
    {
        double centroid_x=0, centroid_y=0, centroid_z=0;
        geometry_msgs::Quaternion centroid_orientation;
        KDL::Frame World_Object;
        KDL::Frame Object_PreGraspFirstEE, Object_PreGraspSecondEE;
        KDL::Frame Object_GraspFirstEE, Object_GraspSecondEE;
        KDL::Frame Object_PostGraspFirstEE, Object_PostGraspSecondEE;
        KDL::Frame World_GraspSecondEE;
	
        // 3.1) Getting preliminary info for the current node
        std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it=node_it;
        node_info node = find_node_properties(path,node_it,next_node_it);
        //---------------------------
        //From now on node is not the last in the path

	if (node.type==node_properties::LAST_EE_FIXED)
        {
            break;
        }
        
	bool ok=getPostGraspMatrix(data.obj_id,node.current_grasp_id,Object_PostGraspFirstEE);
	if (!ok) 
	{
	    std::cout<<"Error in getting postgrasp matrix for object "<<data.obj_id<<" and ee "<<node.current_ee_id<<std::endl;
	    abort();
	}
	ok=getPostGraspMatrix(data.obj_id,node.next_grasp_id,Object_PostGraspSecondEE);
	if (!ok) 
	{
	    std::cout<<"Error in getting postgrasp matrix for object "<<data.obj_id<<" and ee "<<node.next_ee_id<<std::endl;
	    abort();
	}
	ok = getPreGraspMatrix(data.obj_id,node.current_grasp_id,Object_PreGraspFirstEE);
	if (!ok) 
	{
	    std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" and ee "<<node.current_ee_id<<std::endl;
	    abort();
	}
	ok = getPreGraspMatrix(data.obj_id,node.next_grasp_id,Object_PreGraspSecondEE);
	if (!ok) 
	{
	    std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" and ee "<<node.next_ee_id<<std::endl;
	    abort();
	}
	ok = getGraspMatrix(data.obj_id,node.current_grasp_id,Object_GraspFirstEE);
	if (!ok) 
	{
	    std::cout<<"Error in getting grasp matrix for object "<<data.obj_id<<" and ee "<<node.current_ee_id<<std::endl;
	    abort();
	}
	ok = getGraspMatrix(data.obj_id,node.next_grasp_id,Object_GraspSecondEE);
	if (!ok) 
	{
	    std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" and ee "<<node.next_ee_id<<std::endl;
	    abort();
	}
        
        // 3.4) Beginning of real actions, depending on the result of 3.1
        if (node.type==node_properties::LAST_EE_MOVABLE)
        {
	    // 3.4.2) We move the last==current end effector in the final workspace centroid, equal to the final desired position
	    cartesian_command move_command;
	    move_command.command=cartesian_commands::MOVE;
	    move_command.seq_num = 1;
	    move_command.ee_grasp_id=node.current_grasp_id;
	    KDL::Frame Object_EE,World_Object;
	    bool ok=getPostGraspMatrix(data.obj_id,node.current_grasp_id,Object_EE);
	    if (!ok) 
	    {
		std::cout<<"Error in getting postgrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<node.current_ee_id<<std::endl;
	    }
	    tf::poseMsgToKDL(data.target_position,World_Object);
	    tf::poseKDLToMsg(World_Object*Object_EE,move_command.cartesian_task);
	    //TODO: what if this is not feasible? test other grasps? future work...
	    result.push_back(std::make_pair(node.current_ee_id,move_command));
	    break; //This break jumps to 4)
        }
        else if (node.type==node_properties::FIXED_TO_FIXED)
        {
            //Error 3
            std::cout<<"ERROR, the planner returned two nodes with not movable different ees!!"<<std::endl;
            return false;
        }
        else if (node.type==node_properties::FIXED_TO_MOVABLE)
        {
	    std::cout << "Semantic to cartesian: node.type==node_properties::FIXED_TO_MOVABLE" << std::endl;
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            bool intergrasp_ok = false;
            if (node_it == path.begin())
            {
                std::cout << "Semantic to cartesian: first ee is not movable, using fixed location to update the path..." << std::endl;
                tf::poseMsgToKDL(data.source_position,World_Object);
		auto next_ee_name=std::get<0>(database.EndEffectors.at(node.next_ee_id));
		if(check_ik(next_ee_name,World_Object*Object_SecondEE))
		if(check_ik(next_ee_name,World_Object*Object_PreGraspSecondEE))
		  if(check_ik(next_ee_name,World_Object*Object_GraspSecondEE))
		    intergrasp_ok = true;
            }
            else
                intergrasp_ok = compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,node,data.obj_id,result.size());
            if (!intergrasp_ok)
            {
                addNewFilteredArc(node,filtered_source_nodes,filtered_target_nodes);
                return false;
            }
	    #if SUPERHACK
            std::cout << "result.size() : " << result.size() << std::endl;
	    #endif
            World_GraspSecondEE = World_Object*Object_PreGraspSecondEE;
            #if SUPERHACK //raise more the grasp to avoid collision : this *should* be fixed
            KDL::Frame World_GraspSecondEE_original(World_GraspSecondEE);
            World_GraspSecondEE.p.z(World_GraspSecondEE.p.z() + 0.05);
            #endif
            cartesian_command move_command(cartesian_commands::MOVE, 1, node.next_grasp_id);
            tf::poseKDLToMsg(World_GraspSecondEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,move_command)); //move the next
            
            //From fixed to movable we will grasp the object
            cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
            #if SUPERHACK
            tf::poseKDLToMsg(World_GraspSecondEE_original,grasp.cartesian_task);
            #else
            tf::poseKDLToMsg(World_Object,grasp.cartesian_task);
            #endif
            result.push_back(std::make_pair(node.next_ee_id,grasp));
            //TODO: this following move_command can be a post-grasp waypoint, plan to it without collision checking (keeping it higher)
            #if SUPERHACK
            World_GraspSecondEE.p.z(World_GraspSecondEE_original.p.z() + 0.05);
            #else
            World_GraspSecondEE.p.z(World_GraspSecondEE.p.z() + 0.05);
            #endif
            //Back to before the grasp (retreat)
            tf::poseKDLToMsg(World_GraspSecondEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,move_command));
            
        }
        else if (node.type==node_properties::MOVABLE_TO_FIXED)
        {
	    std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_FIXED" << std::endl;
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=1;//do not parallelize with the fixed ee :)
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            bool intergrasp_ok =false;
            if ((next_node_it+1) == path.end())
            {
                std::cout << "Semantic to cartesian: last step, using fixed location to update the path..." << std::endl;
                tf::poseMsgToKDL(data.target_position,World_Object);
		auto current_ee_name=std::get<0>(database.EndEffectors.at(node.current_ee_id));
		  if(check_ik(current_ee_name,World_Object*Object_GraspFirstEE))
		if(check_ik(current_ee_name,World_Object*Object_PostGraspFirstEE))
		    intergrasp_ok = true;
            }
            else
                intergrasp_ok = compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,node,data.obj_id,result.size());
            if (!intergrasp_ok)
            {
                addNewFilteredArc(node,filtered_source_nodes,filtered_target_nodes);
                return false;
            }
            KDL::Frame World_GraspFirstEE = World_Object*Object_PostGraspFirstEE;
            //TODO: add a waypoint higher, then plan the last portion without collision checking
            tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
            cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
            #if SUPERHACK
            // consider the ungrasp trajectory as higher if ungrasping on a table
            ungrasp.cartesian_task.position.z = ungrasp.cartesian_task.position.z + 0.07;
            #endif
	    tf::poseKDLToMsg(World_Object,ungrasp.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,ungrasp));
            cartesian_command move_away(cartesian_commands::HOME,1,-1);
            result.push_back(std::make_pair(node.current_ee_id,move_away));
        }
        else if (node.type==node_properties::MOVABLE_TO_MOVABLE)
        {
	    std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_MOVABLE" << std::endl;
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=0;//Care, we are parallelizing here!
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            bool intergrasp_ok = compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,node,data.obj_id,result.size());
            if (!intergrasp_ok)
            {
                addNewFilteredArc(node,filtered_source_nodes,filtered_target_nodes);
                return false;
            }
            KDL::Frame World_GraspFirstEE = World_Object*Object_PostGraspFirstEE;
            tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
            #if SUPERHACK
            //superhack - part 1 - copy
            KDL::Frame Mirko(World_Object);
            //superhack - part 2 - change the world!
            World_Object.M = fine_tuning[result.size()].M*World_Object.M;
            World_Object.p = World_Object.p + fine_tuning[result.size()].p;
            #endif
            ok = getPreGraspMatrix(data.obj_id,node.next_grasp_id,Object_PreGraspSecondEE);
            if (!ok) 
            {
                std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<node.next_ee_id<<std::endl;
            }
            cartesian_command second_move_command;
            second_move_command.command=cartesian_commands::MOVE;
            second_move_command.ee_grasp_id=node.next_ee_id;
            second_move_command.seq_num=1;//Do not parallelize
            World_GraspSecondEE = World_Object*Object_PreGraspSecondEE;
            tf::poseKDLToMsg(World_GraspSecondEE,second_move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,second_move_command)); //move the next
            //From movable to movable we will grasp the object and ungrasp it
            cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
            // make sure that the grasp/ungrasp actions have the object frame
            tf::poseKDLToMsg(World_Object,grasp.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,grasp));
            cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
            #if SUPERHACK
            //superhack - part 3 - go back
            World_Object = Mirko;
            #endif
            tf::poseKDLToMsg(World_Object,ungrasp.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,ungrasp));
            //TODO: make this next seq a 0 once home is implemented as any other location
            cartesian_command move_away(cartesian_commands::HOME,1,-1);
            result.push_back(std::make_pair(node.current_ee_id,move_away));
        }
        else 
        {
            std::cout<<"SUPER ERROR!!"<<std::endl;
        }
        node_it=next_node_it;
    }
    // 4) return
    return true;
}
