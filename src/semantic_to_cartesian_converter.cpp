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
#define ANGLE_STEPS 6.0

static std::vector<double> left_arm_pos={0.1,0.1,0.1,0.1,0.1,0.1,0.1};
static std::vector<double> right_arm_pos={0.1,0.1,0.1,0.1,0.1,0.1,0.1};

bool left_ik,right_ik, left_ik_ok, right_ik_ok;

void plan_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Plan : %s",str->data.c_str());
    left_ik=true;
    left_ik_ok = strcmp(str->data.c_str(),"error") != 0;
}

void plan_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
    right_ik=true;
    right_ik_ok = strcmp(str->data.c_str(),"error") != 0;
}


semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database)
{
  this->database=database;
  fine_tuning[4]=KDL::Frame(KDL::Vector(-0.06,0.0,-0.02));
  fine_tuning[5]=KDL::Frame(KDL::Vector(-0.06,0.0,-0.02));
  double t=(1.0+sqrt(5.0))/2.0;
  for (double angle=-M_PI;angle<M_PI-0.001;angle=angle+2.0*M_PI/ANGLE_STEPS)
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
}

bool semantic_to_cartesian_converter::getPreGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE)
{
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

bool semantic_to_cartesian_converter::getGraspMatrix(object_id object, grasp_id grasp, KDL::Frame& Object_EE)
{
    dual_manipulation_shared::ik_service srv;
    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
    if (ok)
        tf::poseMsgToKDL(srv.request.ee_pose.back(),Object_EE);
    
    return ok;
}

bool semantic_to_cartesian_converter::getPostGraspMatrix(object_id object, grasp_id grasp, KDL::Frame& Object_EE)
{
    dual_manipulation_shared::ik_service srv;
    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
    if (ok)
    {
        tf::poseMsgToKDL(srv.request.attObject.object.mesh_poses.front(),Object_EE);
        Object_EE = Object_EE.Inverse();
    }
    return ok;
}


void semantic_to_cartesian_converter::compute_centroid(double& centroid_x,double& centroid_y,double& centroid_z, const node_info& node)
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

node_info semantic_to_cartesian_converter::find_node_properties(const std::vector< dual_manipulation_shared::planner_item >& path, const std::vector< dual_manipulation_shared::planner_item >::const_iterator& node, std::vector< dual_manipulation_shared::planner_item >::const_iterator& next_node)
{
    node_info result;
    auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
    bool movable=std::get<1>(database.EndEffectors.at(ee_id));
    // 3.3) Searching for the next node with a different end effector than the current one
    bool found=false;
    endeffector_id next_ee_id=-1;
    workspace_id next_workspace_id=-1;
    bool next_movable=false;
    while (!found && next_node!=path.end())
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

bool semantic_to_cartesian_converter::check_ik(std::string current_ee_name, KDL::Frame World_FirstEE, std::string next_ee_name, KDL::Frame World_SecondEE, std::vector<double>& result_first, std::vector<double>& result_second)
{
    // assume at first everything went smoothly - TODO: something better
    left_ik = true;
    right_ik = true;
    left_ik_ok = true;
    right_ik_ok = true;

    inverse_kinematics(current_ee_name,World_FirstEE);
    inverse_kinematics(next_ee_name,World_SecondEE);
    bool done=false;
    while (!done)
    {
        ros::spinOnce();
        usleep(200000);
        if (left_ik && right_ik) done=left_ik_ok && right_ik_ok;
    }
    return done;
}

bool semantic_to_cartesian_converter::check_ik(std::string ee_name, KDL::Frame World_EE)
{
    // TODO: implement me!
    return true;
}

bool semantic_to_cartesian_converter::inverse_kinematics(std::string ee_name, KDL::Frame cartesian)
{
    static ros::NodeHandle n;
    static ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    static ros::Subscriber plan_lsub = n.subscribe("/ik_control/left_hand/check_done",0,plan_callback_l);
    static ros::Subscriber plan_rsub = n.subscribe("/ik_control/right_hand/check_done",0,plan_callback_r);
    dual_manipulation_shared::ik_service srv;
    
    geometry_msgs::Pose ee_pose;
    tf::poseKDLToMsg(cartesian,ee_pose);
    srv.request.command = "ik_check";
    srv.request.ee_pose.clear();
    srv.request.ee_pose.push_back(ee_pose);
    srv.request.ee_name = ee_name;
    
    if (client.call(srv))
    {
        ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
        return false;
    }
    return true;
}


bool semantic_to_cartesian_converter::compute_intergrasp_orientation(KDL::Vector World_centroid, KDL::Frame& World_Object, 
                                                             const node_info& node, object_id object,int aggiuntivo)
{
    KDL::Frame World_Centroid_f(World_centroid);
    KDL::Frame Object_FirstEE, Object_SecondEE,Object_GraspFirstEE,Object_GraspSecondEE;
    auto current_ee_name=std::get<0>(database.EndEffectors.at(node.current_ee_id));
    auto next_ee_name=std::get<0>(database.EndEffectors.at(node.next_ee_id));
    bool ok=getPostGraspMatrix(object,node.current_grasp_id,Object_FirstEE);
    if (!ok) 
    {
	std::cout<<"Error in getting postgrasp matrix for object "<<object<<" and ee "<<node.current_ee_id<<std::endl;
	abort();
    }
    ok = getPreGraspMatrix(object,node.next_grasp_id,Object_SecondEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	abort();
    }
    ok = getGraspMatrix(object,node.current_grasp_id,Object_GraspFirstEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	abort();
    }
    ok = getGraspMatrix(object,node.next_grasp_id,Object_GraspSecondEE);
    if (!ok) 
    {
	std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<node.next_ee_id<<std::endl;
	abort();
    }
    
    if (node.type==node_properties::MOVABLE_TO_MOVABLE)
    {
        std::vector<double> joint_pose_norm;
	for (auto& rot: sphere_sampling)
	{
	    KDL::Frame World_Object(rot,World_centroid);
	    std::vector<double> result_first, result_second;
	    bool ik_ok=check_ik(current_ee_name,World_Object*Object_FirstEE,next_ee_name,World_Object*Object_SecondEE, result_first, result_second );
    	    bool ik_ok1=check_ik(current_ee_name,World_Object*Object_FirstEE,next_ee_name,World_Object*Object_GraspSecondEE, result_first, result_second );
	    if (!(ik_ok && ik_ok1)) 
	    {
	      joint_pose_norm.push_back(1000);
	      continue;
	    }

	    double norm=0;
	    if (current_ee_name=="right_hand") result_first.swap(result_second);
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
	World_Object = World_Centroid_f*(Object_FirstEE.Inverse());
	if(check_ik(next_ee_name,World_Object*Object_SecondEE))
	  if(check_ik(next_ee_name,World_Object*Object_GraspSecondEE))
	    return true;
	return false;
    }
    else if (node.type==node_properties::MOVABLE_TO_FIXED)
    {
	World_Object = World_Centroid_f*(Object_SecondEE.Inverse());
	if(check_ik(next_ee_name,World_Object*Object_FirstEE))
	  if(check_ik(next_ee_name,World_Object*Object_GraspFirstEE))
	    return true;
	return false;
    }
    else 
    {
      std::cout<<"SUPER ERROR"<<std::endl;
      return false;
    }
}
  
bool semantic_to_cartesian_converter::convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data)
{
    // 1) Clearing result vector
    result.clear();

    // 3) Start of the main conversion loop
    for (auto node_it=path.begin();node_it!=path.end();)//++node)
    {
        double centroid_x=0, centroid_y=0, centroid_z=0;
        geometry_msgs::Quaternion centroid_orientation;
        KDL::Frame World_Object;
        KDL::Frame Object_FirstEE, Object_SecondEE;
        KDL::Frame World_GraspSecondEE;
        
        // 3.1) Getting preliminary info for the current node
        std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it=node_it;
        node_info node = find_node_properties(path,node_it,next_node_it);
        //---------------------------
        //From now on node is not the last in the path

        // 3.4) Beginning of real actions, depending on the result of 3.1
        if (node.type==node_properties::LAST_EE_FIXED)
        {
            break;
        }
        else if (node.type==node_properties::LAST_EE_MOVABLE)
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
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            if (node_it == path.begin())
            {
                std::cout << "Semantic to cartesian: first ee is not movable, using fixed location to update the path..." << std::endl;
                tf::poseMsgToKDL(data.source_position,World_Object);
            }
            else
                compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,node,data.obj_id,result.size());
            std::cout << "result.size() : " << result.size() << std::endl;
            bool ok = getPreGraspMatrix(data.obj_id,node.next_grasp_id,Object_SecondEE);
            if (!ok) 
            {
                std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<node.next_ee_id<<std::endl;
            }
            World_GraspSecondEE = World_Object*Object_SecondEE;
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
            tf::poseKDLToMsg(World_GraspSecondEE,grasp.cartesian_task);
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
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE;
            move_command.ee_grasp_id=node.current_ee_id;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=1;//do not parallelize with the fixed ee :)
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            if ((next_node_it+1) == path.end())
            {
                std::cout << "Semantic to cartesian: last step, using fixed location to update the path..." << std::endl;
                tf::poseMsgToKDL(data.target_position,World_Object);
            }
            else
                compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,node,data.obj_id,result.size());
            bool ok=getPostGraspMatrix(data.obj_id,node.current_grasp_id,Object_FirstEE);
            if (!ok) 
            {
                std::cout<<"Error in getting postgrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<node.current_ee_id<<std::endl;
            }
            KDL::Frame World_GraspFirstEE = World_Object*Object_FirstEE;
            //TODO: add a waypoint higher, then plan the last portion without collision checking
            tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
            cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
            #if SUPERHACK
            // consider the ungrasp trajectory as higher if ungrasping on a table
            ungrasp.cartesian_task.position.z = ungrasp.cartesian_task.position.z + 0.07;
            #endif
            result.push_back(std::make_pair(node.current_ee_id,ungrasp));
            cartesian_command move_away(cartesian_commands::HOME,1,-1);
            result.push_back(std::make_pair(node.current_ee_id,move_away));
        }
        else if (node.type==node_properties::MOVABLE_TO_MOVABLE)
        {
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=0;//Care, we are parallelizing here!
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,node,data.obj_id,result.size());
            bool ok=getPostGraspMatrix(data.obj_id,node.current_grasp_id,Object_FirstEE);
            if (!ok) 
            {
                std::cout<<"Error in getting postgrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<node.current_ee_id<<std::endl;
            }
            KDL::Frame World_GraspFirstEE = World_Object*Object_FirstEE;
            tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
            #if SUPERHACK
            //superhack - part 1 - copy
            KDL::Frame Mirko(World_Object);
            //superhack - part 2 - change the world!
            World_Object.M = fine_tuning[result.size()].M*World_Object.M;
            World_Object.p = World_Object.p + fine_tuning[result.size()].p;
            #endif
            ok = getPreGraspMatrix(data.obj_id,node.next_grasp_id,Object_SecondEE);
            if (!ok) 
            {
                std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<node.next_ee_id<<std::endl;
            }
            cartesian_command second_move_command;
            second_move_command.command=cartesian_commands::MOVE;
            second_move_command.ee_grasp_id=node.next_ee_id;
            second_move_command.seq_num=1;//Do not parallelize
            World_GraspSecondEE = World_Object*Object_SecondEE;
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
