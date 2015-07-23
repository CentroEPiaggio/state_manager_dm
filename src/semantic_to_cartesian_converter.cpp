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
#include <kdl_parser/kdl_parser.hpp>
#include <random>
#include <kdl/frames_io.hpp>

#define HIGH 0.6
#define LOW 0.06
#define ANGLE_STEPS 4.0 // 6.0
#define BIMANUAL_IK_ATTEMPTS 3
#define BIMANUAL_IK_TIMEOUT 0.005
#define OBJ_GRASP_FACTOR 1000

#define DEBUG 0 // if 1, print some more information

std::map<std::pair<object_id,grasp_id >,Object_SingleGrasp> semantic_to_cartesian_converter::cache_matrixes;


semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database):distribution(0.0,1.0)
{
  this->database=database;

  ik_check_capability = new dual_manipulation::ik_control::ikCheckCapability();
  ros::NodeHandle nh;
  std::string global_name, relative_name, default_param;
  if (nh.getParam("/robot_description", global_name)) //The checks here are redundant, but they may provide more debug info
  {
      robot_urdf=global_name;
  }
  else if (nh.getParam("robot_description", relative_name))
  {
      robot_urdf=relative_name;
  }
  else 
  {
      ROS_ERROR_STREAM("semantic_to_cartesian_converter::constructor : cannot find robot_description");
      abort();
  }
  if (!urdf_model.initParam("robot_description"))
  {
      ROS_ERROR_STREAM("semantic_to_cartesian_converter::constructor : cannot load robot_description");
      abort();
  }
  
  if (!kdl_parser::treeFromUrdfModel(urdf_model, robot_kdl))
  {
      ROS_ERROR_STREAM("Failed to construct kdl tree");
      abort();
  }
  
  // TODO: generalize to N-arm systems
  robot_kdl.getChain("left_arm_base_link","left_hand_palm_link",LSh_Obj);
  robot_kdl.getChain("right_hand_palm_link","right_arm_base_link",Obj_Rsh);
  robot_kdl.getChain("left_arm_base_link","right_arm_base_link",LSh_Waist_RSh);
  robot_kdl.getChain("vito_anchor","left_arm_base_link",World_LSh_Chain);
//   KDL::ChainFkSolverPos_recursive temp_solver(LSh_Waist_RSh);
//   KDL::JntArray temp_jnts;
//   temp_jnts.resize(robot_kdl.getNrOfJoints());
//   temp_solver.JntToCart(temp_jnts,LSh_RSh);
//   std::cout << "LSh_RSh: " << LSh_RSh << std::endl;
//   KDL::ChainFkSolverPos_recursive temp_solver1(World_LSh_Chain);
//   KDL::JntArray temp_jnts1;
//   temp_jnts1.resize(robot_kdl.getNrOfJoints());
//   temp_solver1.JntToCart(temp_jnts1,World_LSh);
//   std::cout << "World_LSh: " << World_LSh << std::endl;
  
  // TODO: find all World_ArmBase, generate next ones as Inv(A)*B
  World_LSh.M = KDL::Rotation::RPY(1.571, 0.524, -0.524);
  World_LSh.p = KDL::Vector(-0.165, -0.109, 0.390);
// $ rosrun tf tf_echo vito_anchor left_arm_base_link
// At time 1436460598.096
// - Translation: [-0.165, -0.109, 0.390]
// - Rotation: in Quaternion [0.707, -0.000, -0.354, 0.612]
//             in RPY (radian) [1.571, 0.524, -0.524]
//             in RPY (degree) [90.000, 30.000, -30.000]
  
  LSh_RSh.M = KDL::Rotation::RPY(2.428, 0.848, -0.333);
  LSh_RSh.p = KDL::Vector(-0.094, -0.054, -0.188);
// $ rosrun tf tf_echo left_arm_base_link right_arm_base_link
// At time 1436460619.536
// - Translation: [-0.094, -0.054, -0.188]
// - Rotation: in Quaternion [0.866, 0.000, -0.433, 0.250]
//             in RPY (radian) [2.428, 0.848, -0.333]
//             in RPY (degree) [139.107, 48.590, -19.107]

}

void semantic_to_cartesian_converter::initialize_solvers(chain_and_solvers* container) const
{
    delete container->fksolver;
    delete container->iksolver;
    delete container->ikvelsolver;
    container->joint_names.clear();
    for (KDL::Segment& segment: container->chain.segments)
    {
        if (segment.getJoint().getType()==KDL::Joint::None) continue;
        //std::cout<<segment.getJoint().getName()<<std::endl;
        container->joint_names.push_back(segment.getJoint().getName());
    }
    assert(container->joint_names.size()==container->chain.getNrOfJoints());
    container->q_max.resize(container->chain.getNrOfJoints());
    container->q_min.resize(container->chain.getNrOfJoints());
    container->fksolver=new KDL::ChainFkSolverPos_recursive(container->chain);
    container->ikvelsolver = new KDL::ChainIkSolverVel_pinv(container->chain);
    int j=0;
    for (auto joint_name:container->joint_names)
    {
        #if IGNORE_JOINT_LIMITS
        container->q_max(j)=M_PI/3.0;
        container->q_min(j)=-M_PI/3.0;
        #else
        if(urdf_model.joints_.at(joint_name)->safety)
        {
            container->q_max(j)=urdf_model.joints_.at(joint_name)->safety->soft_upper_limit;
            container->q_min(j)=urdf_model.joints_.at(joint_name)->safety->soft_lower_limit;
        }
        else
        {
            container->q_max(j)=urdf_model.joints_.at(joint_name)->limits->upper;
            container->q_min(j)=urdf_model.joints_.at(joint_name)->limits->lower;
        }
        #endif
        j++;
    }
    // impose some new limits on shoulder joints of both arms
    double LSh0_mean = 0.5, LSh1_mean = 1.0;
    double RSh0_mean = -0.5, RSh1_mean = 1.0;
    double allowed_range = 0.75;
    container->q_min(0) = LSh0_mean - allowed_range;
    container->q_min(1) = LSh1_mean - allowed_range;
    container->q_min(12) = RSh1_mean - allowed_range;
    container->q_min(13) = RSh0_mean - allowed_range;
    container->q_max(0) = LSh0_mean + allowed_range;
    container->q_max(1) = LSh1_mean + allowed_range;
    container->q_max(12) = RSh1_mean + allowed_range;
    container->q_max(13) = RSh0_mean + allowed_range;
    container->iksolver= new KDL::ChainIkSolverPos_NR_JL(container->chain,container->q_min,container->q_max,*container->fksolver,*container->ikvelsolver);
}


bool semantic_to_cartesian_converter::getGraspMatrixes(object_id object, grasp_id grasp, Object_SingleGrasp& Matrixes)
{
  dual_manipulation_shared::ik_service srv;
  grasp = grasp%OBJ_GRASP_FACTOR;
  bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
  if (ok)
  {
    tf::poseMsgToKDL(srv.request.ee_pose.front(),Matrixes.PreGrasp);
    tf::poseMsgToKDL(srv.request.ee_pose.back(),Matrixes.Grasp);
    tf::poseMsgToKDL(srv.request.attObject.object.mesh_poses.front(),Matrixes.PostGrasp);
    Matrixes.PostGrasp = Matrixes.PostGrasp.Inverse();
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

void semantic_to_cartesian_converter::addNewFilteredArc(const node_info& node, dual_manipulation_shared::planner_item& filtered_source_node, dual_manipulation_shared::planner_item& filtered_target_node) const
{
    filtered_source_node.grasp_id=node.current_grasp_id;
    filtered_source_node.workspace_id=node.next_workspace_id;//THIS IS INTENTIONAL!! We remove the intergrasp transition arc in the target workspace
    filtered_target_node.grasp_id=node.next_grasp_id;
    filtered_target_node.workspace_id=node.next_workspace_id;//THIS IS INTENTIONAL!! We remove the intergrasp transition arc in the target workspace
}

bool semantic_to_cartesian_converter::check_ik(std::string ee_name, KDL::Frame World_EE) const
{
  std::vector<geometry_msgs::Pose> ee_poses;
  ee_poses.resize(1);
  geometry_msgs::Pose &ee_pose = ee_poses.at(0);
//   if(ee_name != "left_hand" && ee_name != "right_hand")
//   {
//     ROS_ERROR_STREAM("semantic_to_cartesian_converter::check_ik : unknown end-effector <" << ee_name << ">");
//     ROS_ERROR("At now, only \"left_hand\" and \"right_hand\" are supported!");
//     return false;
//   }
  tf::poseKDLToMsg(World_EE,ee_pose);
  
#if DEBUG
  std::cout << "check_ik: " << ee_name << " in " << ee_pose << std::endl;
#endif
  
  std::vector<std::vector<double>> results;
  results.resize(1);
  
  std::vector <double > initial_guess = std::vector<double>();
  bool check_collisions = false;
//   bool return_approximate_solution = true;
//   unsigned int attempts = 10;
//   double timeout = 0.005;
//   std::map <std::string, std::string > allowed_collisions = std::map< std::string, std::string >();
  
  ik_check_capability->reset_robot_state();
  bool found_ik = ik_check_capability->find_group_ik(ee_name,ee_poses,results,initial_guess,check_collisions/*,return_approximate_solution,attempts,timeout,allowed_collisions*/);
  
  return found_ik;
}

bool semantic_to_cartesian_converter::compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object) const
{
//     double centroid_x,centroid_y,centroid_z;
//     compute_centroid(centroid_x,centroid_y,centroid_z,node);
//     KDL::Frame World_centroid(KDL::Vector(centroid_x,centroid_y,centroid_z));

    Object_GraspMatrixes Object;
    auto current_ee_name=std::get<0>(database.EndEffectors.at(node.current_ee_id));
    auto next_ee_name=std::get<0>(database.EndEffectors.at(node.next_ee_id));
    if (!getGraspMatrixes(object, node, Object)) abort();
    if (node.type==node_properties::MOVABLE_TO_MOVABLE)
    {
        //New implementation //TODO: check for PreGrasp
        // TODO: use two maps, direct and inverse chains, indexed by end-effector names
        // TODO: LSh_Obj_RSh.addChain(ChainMap.at(current_ee_name)); ...
        KDL::Chain LSh_Obj_RSh;
        LSh_Obj_RSh.addChain(LSh_Obj);
        if (current_ee_name=="left_hand" && next_ee_name=="right_hand")
        {
            LSh_Obj_RSh.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Object.PostGraspFirstEE.Inverse()));
            LSh_Obj_RSh.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Object.GraspSecondEE));
        }
        else if (current_ee_name=="right_hand" && next_ee_name=="left_hand")
        {
            LSh_Obj_RSh.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Object.GraspSecondEE.Inverse()));
            LSh_Obj_RSh.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Object.PostGraspFirstEE));
        }
        else
        {
            ROS_ERROR_STREAM("semantic_to_cartesian_converter::compute_intergrasp_orientation : unknown end-effector couple <" << current_ee_name << " | " << next_ee_name << ">");
            ROS_ERROR("At now, only \"left_hand\" and \"right_hand\" are supported!");
            return false;
        }
        LSh_Obj_RSh.addChain(Obj_Rsh);
        LSh_Obj_RSh_solvers.chain=LSh_Obj_RSh;
        this->initialize_solvers(&LSh_Obj_RSh_solvers);
        bool done=false;
        bool found=false;
        int counter=0;
        KDL::JntArray random_start;
        KDL::JntArray q_out;
        random_start.resize(LSh_Obj_RSh.getNrOfJoints());
        q_out.resize(LSh_Obj_RSh.getNrOfJoints());
        while(!done && !found)
        {
            for (int i=0;i<LSh_Obj_RSh.getNrOfJoints();i++)
            {
                random_start(i) = distribution(generator)*(LSh_Obj_RSh_solvers.q_max(i)-LSh_Obj_RSh_solvers.q_min(i))+LSh_Obj_RSh_solvers.q_min(i);
            }
            if (LSh_Obj_RSh_solvers.iksolver->CartToJnt(random_start,LSh_RSh,q_out)) 
            {
                // prepare collision checking
                moveit::core::RobotState rs = ik_check_capability->get_robot_state();
                for(int j=0; j<LSh_Obj_RSh.getNrOfJoints();j++)
                  rs.setJointPositions(LSh_Obj_RSh_solvers.joint_names.at(j),&(q_out(j)));
                bool self_collision_only = false;
                // TODO: instead of "both_hands", use the FULL ROBOT group (which has to exist in MoveIt! and doesn't need to change depending on considered ee's)
                found = ik_check_capability->is_state_collision_free(&rs, "both_hands", self_collision_only);
                std::cout << __func__ << "@" << __LINE__ << " : found a configuration which was " << (found?"NOT ":"") << "colliding!" << std::endl;
            }
            if (counter++>10) done=true;
        }
        if (found)
        {
            KDL::ChainFkSolverPos_recursive temp_fk(LSh_Obj);
            KDL::JntArray temp;
            temp.resize(LSh_Obj.getNrOfJoints());
            for (int i=0;i<LSh_Obj.getNrOfJoints();i++)
            {
                temp(i)=q_out(i);
            }
            KDL::Frame LSh_LeftHand;
            temp_fk.JntToCart(temp,LSh_LeftHand);
            // TODO: since we build all possible chains, just use the 1st row
            if (current_ee_name=="left_hand" && next_ee_name=="right_hand") World_Object=World_LSh*LSh_LeftHand*Object.PostGraspFirstEE.Inverse();
            if (current_ee_name=="right_hand" && next_ee_name=="left_hand") World_Object=World_LSh*LSh_LeftHand*Object.GraspSecondEE.Inverse();
        }
        return found;
    }
    else if (node.type==node_properties::FIXED_TO_MOVABLE)
    {
        abort();
    }
    else if (node.type==node_properties::MOVABLE_TO_FIXED)
    {
        abort();
    }
    else 
    {
      std::cout<<"SUPER ERROR"<<std::endl;
      return false;
    }
}

bool semantic_to_cartesian_converter::getGraspMatrixes(object_id object, node_info node, Object_GraspMatrixes& Object)
{
    Object_SingleGrasp temp;
    
    if (cache_matrixes.count(std::make_pair(object,node.current_grasp_id)))
    {
        temp = cache_matrixes[std::make_pair(object,node.current_grasp_id)];
    }
    else
    {
      bool ok = getGraspMatrixes(object,node.current_grasp_id,temp);
      if (!ok)
      {
	  std::cout<<"Error in getting grasp #" << node.current_grasp_id << " matrixes for object "<<object<<" and ee "<<node.current_ee_id<<std::endl;
	  return false;
      }
      cache_matrixes[std::make_pair(object,node.current_grasp_id)]=temp;
    }
    Object.GraspFirstEE = temp.Grasp;
    Object.PostGraspFirstEE = temp.PostGrasp;
    Object.PreGraspFirstEE = temp.PreGrasp;

    if (cache_matrixes.count(std::make_pair(object,node.next_grasp_id)))
    {
        temp = cache_matrixes[std::make_pair(object,node.next_grasp_id)];
    }
    else
    {
      bool ok = getGraspMatrixes(object,node.next_grasp_id,temp);
      if (!ok)
      {
	  std::cout<<"Error in getting grasp #" << node.next_grasp_id << " matrixes for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	  return false;
      }
      cache_matrixes[std::make_pair(object,node.next_grasp_id)]=temp;
    }
    Object.GraspSecondEE = temp.Grasp;
    Object.PostGraspSecondEE = temp.PostGrasp;
    Object.PreGraspSecondEE = temp.PreGrasp;
    
    return true;
}

bool semantic_to_cartesian_converter::checkSingleGrasp(KDL::Frame& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const
{
    double centroid_x=0, centroid_y=0, centroid_z=0;
    Object_GraspMatrixes Object;
    if (!getGraspMatrixes(data.obj_id, node, Object)) abort();
    if (node.type==node_properties::FIXED_TO_MOVABLE)
    {
        std::cout << "Semantic to cartesian: node.type==node_properties::FIXED_TO_MOVABLE" << std::endl;
        // 3.6) compute a rough position of the place where the change of grasp will happen
        compute_centroid(centroid_x,centroid_y,centroid_z,node);
        KDL::Frame World_Centroid_f(KDL::Frame(KDL::Vector(centroid_x,centroid_y,centroid_z)));
        bool intergrasp_ok = false;
        auto next_ee_name=std::get<0>(database.EndEffectors.at(node.next_ee_id));
        
        if (first_node)
        {
            std::cout << "Semantic to cartesian: first ee is not movable, using fixed location to update the path..." << std::endl;
            tf::poseMsgToKDL(data.source_position,World_Object);
        }
        else
        {
             World_Object = World_Centroid_f*(Object.PostGraspFirstEE.Inverse());
        }
        if(check_ik(next_ee_name,World_Object*Object.PreGraspSecondEE))
            if(check_ik(next_ee_name,World_Object*Object.GraspSecondEE))
		intergrasp_ok = true;
        if (!intergrasp_ok)
        {
            addNewFilteredArc(node,filtered_source_nodes,filtered_target_nodes);
            return false;
        }
    }
    else if (node.type==node_properties::MOVABLE_TO_FIXED)
    {
        std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_FIXED" << std::endl;
        // 3.6) compute a rough position of the place where the change of grasp will happen
        compute_centroid(centroid_x,centroid_y,centroid_z,node);
        KDL::Frame World_Centroid_f(KDL::Frame(KDL::Vector(centroid_x,centroid_y,centroid_z)));
        bool intergrasp_ok =false;
        auto current_ee_name=std::get<0>(database.EndEffectors.at(node.current_ee_id));
        if (last_node)
        {
            std::cout << "Semantic to cartesian: last step, using fixed location to update the path..." << std::endl;
            tf::poseMsgToKDL(data.target_position,World_Object);
            // NOTE: here there are two checks only cause ungrasp retreat is already best-effort!!

        }
        else
        {
	    // TODO: make this more general: now the two conditions are equivalent at the last_node only because, for a table, Grasp==PostGrasp!!!
            World_Object = World_Centroid_f*(Object.GraspSecondEE.Inverse());
        }
        if(check_ik(current_ee_name,World_Object*Object.PostGraspFirstEE))
            // NOTE: this checks for World_preGraspSecondEE
            if(check_ik(current_ee_name,World_Object*Object.GraspSecondEE*(Object.PreGraspSecondEE.Inverse())*Object.PostGraspFirstEE))
                // if(check_ik(current_ee_name,World_Object*Object_GraspFirstEE))
                intergrasp_ok = true;
        if (!intergrasp_ok)
        {
            addNewFilteredArc(node,filtered_source_nodes,filtered_target_nodes);
            return false;
        }
    }
    return true;
}

bool semantic_to_cartesian_converter::convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    // 1) Clearing result vector
    result.clear();

    // 3) Start of the main conversion loop
    for (auto node_it=path.begin();node_it!=path.end();)//++node)
    {
        geometry_msgs::Quaternion centroid_orientation;
        KDL::Frame World_Object;
        Object_GraspMatrixes Object;
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

        if (!getGraspMatrixes(data.obj_id, node, Object)) abort();

        // 3.4) Beginning of real actions, depending on the result of 3.1
        if (node.type==node_properties::LAST_EE_MOVABLE)
        {
	    // 3.4.2) We move the last==current end effector in the final workspace centroid, equal to the final desired position
	    cartesian_command move_command;
	    move_command.command=cartesian_commands::MOVE;
	    move_command.seq_num = 1;
	    move_command.ee_grasp_id=node.current_grasp_id;
	    KDL::Frame World_Object;
	    tf::poseMsgToKDL(data.target_position,World_Object);
	    tf::poseKDLToMsg(World_Object*Object.PostGraspFirstEE,move_command.cartesian_task);
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
            if (!checkSingleGrasp(World_Object,node,data,node_it==path.begin(),false,filtered_source_nodes,filtered_target_nodes))
                return false;
            World_GraspSecondEE = World_Object*Object.PreGraspSecondEE;
            cartesian_command move_command(cartesian_commands::MOVE_BEST_EFFORT, 1, node.next_grasp_id);
            tf::poseKDLToMsg(World_GraspSecondEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,move_command)); //move the next

            //From fixed to movable we will grasp the object
            cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
            tf::poseKDLToMsg(World_Object,grasp.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,grasp));
	    // cartesian_command move_no_coll_command(cartesian_commands::MOVE_CLOSE_BEST_EFFORT, 1, node.next_grasp_id);
	    // KDL::Frame World_postGraspSecondEE;
	    // World_postGraspSecondEE = World_Object*Object.GraspFirstEE*(Object.PreGraspFirstEE.Inverse())*Object.PostGraspSecondEE;
	    // tf::poseKDLToMsg(World_postGraspSecondEE,move_no_coll_command.cartesian_task);
	    // result.push_back(std::make_pair(node.next_ee_id,move_no_coll_command));
        }
        else if (node.type==node_properties::MOVABLE_TO_FIXED)
        {
	    std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_FIXED" << std::endl;
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE_BEST_EFFORT;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=1;//do not parallelize with the fixed ee :)
            cartesian_command move_no_coll_command(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, node.current_grasp_id);
            // 3.6) compute a rough position of the place where the change of grasp will happen
            if (!checkSingleGrasp(World_Object,node,data,false,((next_node_it+1) == path.end()),filtered_source_nodes,filtered_target_nodes))
                return false;
            KDL::Frame World_PreGraspSecondEE = World_Object*Object.GraspSecondEE*(Object.PreGraspSecondEE.Inverse())*Object.PostGraspFirstEE;
            tf::poseKDLToMsg(World_PreGraspSecondEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
            KDL::Frame World_GraspSecondEE = World_Object*Object.PostGraspFirstEE;
            tf::poseKDLToMsg(World_GraspSecondEE,move_no_coll_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_no_coll_command)); //move the first
            cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
	    // TODO: check the following transformation, should be more precisely something like 
            // TODO: "World_Object*Object_PostGraspFirstEE*(Object_GraspFirstEE.Inverse())"
	    tf::poseKDLToMsg(World_Object,ungrasp.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,ungrasp));
            cartesian_command move_away(cartesian_commands::HOME,0,-1);
            result.push_back(std::make_pair(node.current_ee_id,move_away));
        }
        else if (node.type==node_properties::MOVABLE_TO_MOVABLE)
        {
	    std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_MOVABLE" << std::endl;
            cartesian_command move_command(cartesian_commands::MOVE,0,-1); // Care, we are parallelizing here!
            // 3.6) compute a rough position of the place where the change of grasp will happen
            bool intergrasp_ok = compute_intergrasp_orientation(World_Object,node,data.obj_id);
            if (!intergrasp_ok)
            {
                addNewFilteredArc(node,filtered_source_nodes,filtered_target_nodes);
                return false;
            }
            KDL::Frame World_GraspFirstEE = World_Object*Object.PostGraspFirstEE;
            tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
            cartesian_command second_move_command(cartesian_commands::MOVE,1,-1); // do NOT parallelize;
            World_GraspSecondEE = World_Object*Object.PreGraspSecondEE;
            tf::poseKDLToMsg(World_GraspSecondEE,second_move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,second_move_command)); //move the next
            //From movable to movable we will grasp the object and ungrasp it
            cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
            // make sure that the grasp/ungrasp actions have the object frame
            tf::poseKDLToMsg(World_Object,grasp.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,grasp));
            cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
            tf::poseKDLToMsg(World_Object,ungrasp.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,ungrasp));
            cartesian_command move_away(cartesian_commands::HOME,0,-1);
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
