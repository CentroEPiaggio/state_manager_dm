#include "../include/semantic_to_cartesian_converter.h"
#include <shared_memory.h>
#include <dual_manipulation_shared/parsing_utils.h>
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
#include <moveit/robot_model/joint_model_group.h>

#define HIGH 0.6
#define LOW 0.06
#define ANGLE_STEPS 4.0 // 6.0
#define BIMANUAL_IK_ATTEMPTS 3
#define BIMANUAL_IK_TIMEOUT 0.005
#define OBJ_GRASP_FACTOR 1000

#define DEBUG 0 // if 1, print some more information
#define SHOW_IK 1
#define MAX_ITER 100
#define EPS 5e-3

std::map<std::pair<object_id,grasp_id >,Object_SingleGrasp> semantic_to_cartesian_converter::cache_matrixes;
bool am_I_Vito = false;

semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database):distribution(0.0,1.0),database(database)
{
//   this->database=database;

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
  
  if (nh.getParam("ik_control_parameters", ik_control_params))
      parseParameters(ik_control_params);

  KDL::Chain temp;
  std::vector<std::string> link_names;
  const moveit::core::JointModelGroup* jmg = ik_check_capability->get_robot_state().getRobotModel()->getJointModelGroup("full_robot");
  jmg->getEndEffectorTips(link_names);
  std::vector<std::string> ee_names = jmg->getAttachedEndEffectorNames();
  std::string root = ik_check_capability->get_robot_state().getRobotModel()->getRootLinkName();
  
#if DEBUG
  std::cout << "root: " << root << std::endl;
#endif
  
  for (int i=0; i<link_names.size(); i++)
  {
    std::string end_effector = link_names.at(i);
    robot_kdl.getChain(root,end_effector,temp);
    std::string seg_fake_name;
    bool fake_next=true;
    for (auto s: temp.segments)
    {
        KDL::Joint j = s.getJoint();

#if DEBUG
        std::cout << "s.getName(): " << s.getName() << std::endl;
        std::cout << "s.getJoint().getName(): " << s.getJoint().getName() << std::endl;
        std::cout << "s.getJoint().JointOrigin(): " << s.getJoint().JointOrigin().x() << " " << s.getJoint().JointOrigin().y() << " " << s.getJoint().JointOrigin().z() << std::endl;
        std::cout << "s.getJoint().JointAxis(): " << s.getJoint().JointAxis().x() << " "<< s.getJoint().JointAxis().y() << " "<< s.getJoint().JointAxis().z() << std::endl;
        std::cout << "s.getJoint().getType(): " << s.getJoint().getType() << std::endl;
        KDL::Frame f = s.getFrameToTip();
        std::cout << f.p.data[0] << " "<< f.p.data[1] << " "<< f.p.data[2] << std::endl;
        double r,p,y; f.M.GetRPY(r,p,y);
        std::cout << r << " " << p << " " << y << std::endl;
#endif

        if (fake_next)
        {
            if (s.getJoint().getType()==KDL::Joint::None)
            {
                seg_fake_name=end_effector;
                fake_next=true;
                j=KDL::Joint(s.getJoint().getName()+end_effector);
            }
            else if (s.getJoint().getType()!=KDL::Joint::None && fake_next)
            {
                seg_fake_name="";
                fake_next=false;
            }
        }
        else
        {
            seg_fake_name="";
            fake_next=false;
        }
        KDL::Segment b(s.getName()+seg_fake_name,j,s.getFrameToTip(),s.getInertia());
        chains[ee_names.at(i)].addSegment(b);
    }
    KDL::Tree t("fake_root");
    bool done = t.addChain(chains[ee_names.at(i)],"fake_root");
    assert(done);
    done = t.getChain(end_effector,"fake_root",chains_reverse[ee_names.at(i)]);
    
#if DEBUG
    temp = chains_reverse.at(ee_names.at(i));
    for (auto s: temp.segments)
    {
        
        std::cout << "s.getName(): " << s.getName() << std::endl;
        KDL::Joint j = s.getJoint();
        
        std::cout << "s.getJoint().getName(): " << s.getJoint().getName() << std::endl;
        KDL::Frame f = s.getFrameToTip();
        std::cout << f.p.data[0] << " "<< f.p.data[1] << " "<< f.p.data[2] << std::endl;
        double r,p,y; f.M.GetRPY(r,p,y);
        std::cout << r << " " << p << " " << y << std::endl;
    }
    std::cout << "t.getNrOfJoints(): " << t.getNrOfJoints() << std::endl;
    std::cout << "chains_reverse[ee_names.at(i)].getNrOfJoints(): " << chains_reverse[ee_names.at(i)].getNrOfJoints() << std::endl;
    std::cout << "chains[" << ee_names.at(i) << "].segments: | ";
    for(auto segs:chains[ee_names.at(i)].segments)
        std::cout << segs.getName() << " | ";
    std::cout << std::endl;
    std::cout << "t.segments: | ";
    for(auto segs:t.getSegments())
        std::cout << segs.first << " | ";
    std::cout << std::endl;
    if(!done)
    {
        std::cout << "semantic_to_cartesian_converter : unable to construct chains_reverse[" << ee_names.at(i) << "] - aborting" << std::endl;
        abort();
    }
#endif
  }

    // check whether I am using Vito
    std::string robot_name = urdf_model.getName();
    am_I_Vito = (robot_name == "vito");
    std::cout << "semantic_to_cartesian_converter : I am using \"" << robot_name << "\" robot!" << std::endl;
}

void semantic_to_cartesian_converter::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    parseSingleParameter(params,chain_names_list,"chain_group_names",1);

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
#if DEBUG
        std::cout<<segment.getJoint().getName()<<std::endl;
#endif
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
    if (am_I_Vito) //Particular case of Vito robot
    {
        //TODO impose some new limits on shoulder joints of both arms
        // double LSh0_mean = 0.5, LSh1_mean = 1.0;
        // double RSh0_mean = -0.5, RSh1_mean = 1.0;
        int start_ind, inc;
        if (container->joint_names.at(0).find("left") == 0)
        {
            start_ind = 10; inc = 1;
        }
        else if (container->joint_names.at(0).find("right") == 0)
        {
            start_ind = 3; inc = -1;
        }
        else
        {
            std::cout << "Vito should start with something else! (container->joint_names.at(0) = " << container->joint_names.at(0) << ")" << std::endl;
            abort();
        }
        double allowed_range = 0.25;
        container->q_min(start_ind) = 1.4 - allowed_range;
        container->q_max(start_ind) = 1.4 + allowed_range;
        container->q_min(start_ind+inc) = 1.8 - allowed_range;
        container->q_max(start_ind+inc) = 1.8 + allowed_range;
        container->q_min(start_ind+2*inc) = 1.0 - allowed_range/2.0;
        container->q_max(start_ind+2*inc) = 1.0 + allowed_range/2.0;
        container->q_min(start_ind+3*inc) = 0.5 - allowed_range/2.0;
        container->q_max(start_ind+3*inc) = 0.5 + allowed_range/2.0;
    }
    uint max_iter = MAX_ITER;
    double eps = EPS;
    container->iksolver= new KDL::ChainIkSolverPos_NR_JL(container->chain,container->q_min,container->q_max,*container->fksolver,*container->ikvelsolver,max_iter,eps);
}


bool semantic_to_cartesian_converter::getGraspMatrixes(object_id object, grasp_id grasp, Object_SingleGrasp& Matrixes)
{
  dual_manipulation_shared::ik_service srv;
  grasp = grasp%OBJ_GRASP_FACTOR;
  bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
  if (ok)
  {
      normalizePose(srv.request.ee_pose.front());
      normalizePose(srv.request.ee_pose.back());
      normalizePose(srv.request.attObject.object.mesh_poses.front());
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
        KDL::Chain First_Obj_Second;
        assert(chains.count(current_ee_name));
        assert(chains_reverse.count(next_ee_name));
        First_Obj_Second.addChain(chains.at(current_ee_name));
        First_Obj_Second.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Object.PostGraspFirstEE.Inverse()));
        First_Obj_Second.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),Object.GraspSecondEE));
        First_Obj_Second.addChain(chains_reverse.at(next_ee_name));
        double_arm_solver.chain=First_Obj_Second;
        this->initialize_solvers(&double_arm_solver);
        bool done=false;
        bool found=false;
        int counter=0;
        KDL::JntArray random_start;
        KDL::JntArray q_out;
        random_start.resize(First_Obj_Second.getNrOfJoints());
        q_out.resize(First_Obj_Second.getNrOfJoints());
        while(!done && !found)
        {
            for (int i=0;i<First_Obj_Second.getNrOfJoints();i++)
            {
                random_start(i) = distribution(generator)*(double_arm_solver.q_max(i)-double_arm_solver.q_min(i))+double_arm_solver.q_min(i);
            }
            if (double_arm_solver.iksolver->CartToJnt(random_start,KDL::Frame::Identity(),q_out) >= 0)
            {
                // prepare collision checking
                moveit::core::RobotState rs = ik_check_capability->get_robot_state();
                for(int j=0; j<First_Obj_Second.getNrOfJoints();j++)
                  rs.setJointPositions(double_arm_solver.joint_names.at(j),&(q_out(j)));
                bool self_collision_only = false;
                found = ik_check_capability->is_state_collision_free(&rs, "full_robot", self_collision_only);
                std::cout << __func__ << "@" << __LINE__ << " : found a configuration which was " << (found?"NOT ":"") << "colliding!" << std::endl;
#if DEBUG
#if SHOW_IK
                publishConfig(double_arm_solver.joint_names,q_out);
                ros::spinOnce();
#endif
                KDL::Frame out_f;
                double_arm_solver.fksolver->JntToCart(q_out,out_f);
                std::cout << "Resulting transformation (should be the Identity)\n: " << out_f << std::endl;
                for(int j=0; j<5; j++){ ros::spinOnce(); usleep(5000); }
                char y;
                std::cin >> y;
#endif
            }
            if (counter++>10) done=true;
        }
        if (found)
        {
            KDL::ChainFkSolverPos_recursive temp_fk(chains.at(current_ee_name));
            KDL::JntArray temp;
            temp.resize(chains.at(current_ee_name).getNrOfJoints());
            for (int i=0;i<chains.at(current_ee_name).getNrOfJoints();i++)
            {
                temp(i)=q_out(i);
            }
            KDL::Frame World_FirstEE;
            temp_fk.JntToCart(temp,World_FirstEE);
            World_Object=World_FirstEE*Object.PostGraspFirstEE.Inverse();
            
#if SHOW_IK
            std::cout << "I found it!" << std::endl;
            publishConfig(double_arm_solver.joint_names,q_out);
#if DEBUG
            std::cout << "Press any key to continue...";
            char y;
            std::cin >> y;
#endif
#endif
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
#if DEBUG
        std::cout << "Semantic to cartesian: node.type==node_properties::FIXED_TO_MOVABLE" << std::endl;
#endif
        // 3.6) compute a rough position of the place where the change of grasp will happen
        compute_centroid(centroid_x,centroid_y,centroid_z,node);
        KDL::Frame World_Centroid_f(KDL::Frame(KDL::Vector(centroid_x,centroid_y,centroid_z)));
        bool intergrasp_ok = false;
        auto next_ee_name=std::get<0>(database.EndEffectors.at(node.next_ee_id));
        
        if (first_node)
        {
            #if DEBUG
            std::cout << "Semantic to cartesian: first ee is not movable, using fixed location to update the path..." << std::endl;
            std::cout << "Ee_name: " << next_ee_name << " | grasp_id: " << node.next_grasp_id << " | ws_id: " << node.next_workspace_id << std::endl;
            #endif
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
#if DEBUG
        std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_FIXED" << std::endl;
#endif
        // 3.6) compute a rough position of the place where the change of grasp will happen
        compute_centroid(centroid_x,centroid_y,centroid_z,node);
        KDL::Frame World_Centroid_f(KDL::Frame(KDL::Vector(centroid_x,centroid_y,centroid_z)));
        bool intergrasp_ok =false;
        auto current_ee_name=std::get<0>(database.EndEffectors.at(node.current_ee_id));
        if (last_node)
        {
#if DEBUG
            std::cout << "Semantic to cartesian: last step, using fixed location to update the path..." << std::endl;
#endif
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
#if DEBUG
	    std::cout << "Semantic to cartesian: node.type==node_properties::FIXED_TO_MOVABLE" << std::endl;
#endif
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
#if DEBUG
	    std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_FIXED" << std::endl;
#endif
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
#if DEBUG
	    std::cout << "Semantic to cartesian: node.type==node_properties::MOVABLE_TO_MOVABLE" << std::endl;
#endif
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

bool semantic_to_cartesian_converter::publishConfig(const std::vector< std::string >& joint_names, const KDL::JntArray& q) const
{
    static ros::Publisher joint_state_pub_;
    static bool pub_initialized(false);
    static ros::NodeHandle node;
    if (!pub_initialized)
    {
        joint_state_pub_ = node.advertise<sensor_msgs::JointState>("/sem2cart/joint_states",10);
        pub_initialized = true;
    }
    sensor_msgs::JointState js_msg;
    js_msg.name = joint_names;
    js_msg.header.stamp = ros::Time::now();
    js_msg.position.clear();
    for(int i=0; i<js_msg.name.size(); i++)
    {
        js_msg.position.push_back(q(i));
    }
    joint_state_pub_.publish(js_msg);
    
    return true;
}

bool semantic_to_cartesian_converter::normalizePose(geometry_msgs::Pose& pose)
{
    geometry_msgs::Quaternion& q(pose.orientation);
    double q_norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    q.x = q.x/q_norm;
    q.y = q.y/q_norm;
    q.z = q.z/q_norm;
    q.w = q.w/q_norm;
    
    bool ok = (q_norm < 1.01 && q_norm > 0.99);
    
    if(!ok)
        std::cout << "Pose not properly normalized, quaternion norm was " << q_norm << std::endl;
    
    return ok;
}

bool semantic_to_cartesian_converter::normalizePoses(std::vector< geometry_msgs::Pose >& poses)
{
    bool ok = true;
    for (auto& p:poses)
        ok = ok & normalizePose(p);
    
    return ok;
}