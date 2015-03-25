#include "../include/semantic_to_cartesian_converter.h"
#include <shared_memory.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <vector>
#include <kdl_conversions/kdl_msg.h>

#define SUPERHACK 0

semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database)
{
this->database=database;
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

node_info semantic_to_cartesian_converter::find_node_properties(const dual_manipulation_shared::planner_serviceResponse::_path_type& path,
                                                                const dual_manipulation_shared::planner_serviceResponse::_path_type::iterator& node,
                                                                dual_manipulation_shared::planner_serviceResponse::_path_type::iterator& next_node
                                                               )
{
    node_info result;
    auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
    bool movable=std::get<1>(database.EndEffectors.at(ee_id));
    if (node+1==path.end()) 
    {
        result.type=node_properties::FINAL_NODE;
        return result;
    }
    // 3.3) Searching for the next node with a different end effector than the current one
    bool found=false;
    endeffector_id next_ee_id;
    workspace_id next_workspace_id;
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
        if (movable && !movable) result.type=node_properties::MOVABLE_TO_FIXED;          //found, one is movable, change on ground
        if (!movable && movable) result.type=node_properties::FIXED_TO_MOVABLE;          //found, one is movable, change on ground
        if (movable && movable) result.type=node_properties::MOVABLE_TO_MOVABLE;        //found, both ee are movable: change above ground
        if (!movable && !movable) result.type=node_properties::FIXED_TO_FIXED;            //if (!movable && !next_movable)
        result.current_ee_id=ee_id;
        result.next_ee_id=next_ee_id;
        result.current_grasp_id=node->grasp_id;
        result.next_grasp_id=next_node->grasp_id;
        result.current_workspace_id=workspace_id;
        result.next_workspace_id=next_workspace_id;
    }
    else
    {
        if (!movable) result.type=node_properties::LAST_EE_FIXED;             //3.4.1) if not found, than ee_id is the last end effector in the path //not found not movable
        else result.type=node_properties::LAST_EE_MOVABLE;            //else //not found->last e.e, movable
    }
    return result;
}

void semantic_to_cartesian_converter::initialize_grasped_map(const dual_manipulation_shared::planner_serviceResponse::_path_type& path)
{
    //This approach is valid only if no more than one e.e. can grasp an object at the same time!! 
    for (auto node=path.begin();node!=path.end();++node)
    {
        auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
        ee_grasped[ee_id]=false;
    }
    auto grasping_ee = std::get<1>(database.Grasps.at(path.front().grasp_id));
    ee_grasped[grasping_ee]=true;
    std::cout<<"Assuming that only "<<std::get<0>(database.EndEffectors.at(grasping_ee))<<" is grasping the object, and no other e.e. is grasping anything!"<<std::endl;
}

void super_compute_intergrasp_orientation()
{
    // treat differently the first and last cases if the associated end-effector is not movable
    if ((node == path.begin()) && (!movable))
    {
        std::cout << "Semantic to cartesian: first ee is not movable, using fixed location to update the path..." << std::endl;
        tf::poseMsgToKDL(data.source_position,World_Object);
    }
    else if ((next_node+1) == path.end())
    {
        std::cout << "Semantic to cartesian: last step, using fixed location to update the path..." << std::endl;
        tf::poseMsgToKDL(data.target_position,World_Object);
    }
    else
    {
        compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,ee_id,next_ee_id,node->grasp_id,next_node->grasp_id,data.obj_id,movable,next_movable,result.size());
    }
}

bool semantic_to_cartesian_converter::convert(std::vector<std::pair<endeffector_id,cartesian_command>>& result,const dual_manipulation_shared::planner_serviceResponse::_path_type& path)
{
    // 1) Clearing result vector
    result.clear();

    // 2) Setting a boolean flag for each end effector in order to keep track if they are grasping something or not
    initialize_grasped_map();
    //-------------------------------------------
    
    // 3) Start of the main conversion loop
    for (auto node_it=path.begin();node_it!=path.end();)//++node)
    {
        double centroid_x=0, centroid_y=0, centroid_z=0;
        geometry_msgs::Quaternion centroid_orientation;
        KDL::Frame World_Object;
        KDL::Frame Object_FirstEE, Object_SecondEE;
        KDL::Frame World_GraspSecondEE;
        
        // 3.1) Getting preliminary info for the current node
        auto next_node_it=node_it;
        node_info node = find_node_properties(path,node_it,next_node_it);
        //---------------------------

        // 3.2) Is this the final_node?
        if (node.type==node_properties::FINAL_NODE) break; //This break jumps to 4)
        //-------------------------
        //From now on node is not the last in the path
        // 3.3) Searching for the next node with a different end effector than the current one
        //Done in find_node_properties

        // 3.4) Beginning of real actions, depending on the result of 3.3
        if (node.type==node_properties::LAST_EE_FIXED)
        {
            //Error1
            std::cout<<"ERROR, ee "<<node.current_ee_id<<" is the last ee but cannot move into the last node of the path!!"<<std::endl;
            return false;
        }
        else if (node.type==node_properties::LAST_EE_MOVABLE)
        {
            if (node.current_workspace_id==node.next_workspace_id)
            {
                //Error2
                std::cout<<"ERROR, the planner returned two nodes with same ee and same workspace!!"<<std::endl;
                return false;
            }
            else  //not found->last e.e, movable, different workspaces
            {
                // 3.4.2) We move the last==current end effector in the final workspace centroid, equal to the final desired position
                compute_centroid(centroid_x,centroid_y,centroid_z,node);
                //TODO: probably we want to write here the final desired position, not the centroid of the workspace
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
        }
        else if (node.type==node_properties::FIXED_TO_FIXED)
        {
            //Error 3
            std::cout<<"ERROR, the planner returned two nodes with not movable different ees!!"<<std::endl;
            return false;
        }
        else if (node.type=node_properties::node_properties::FIXED_TO_MOVABLE)
        {
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            super_compute_intergrasp_orientation();
            #if SUPERHACK
            //superhack - part 1 - copy
            KDL::Frame Mirko(World_Object);
            #endif
            std::cout << "result.size() : " << result.size() << std::endl;
            bool ok = getPreGraspMatrix(data.obj_id,next_node->grasp_id,Object_SecondEE);
            if (!ok) 
            {
                std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<next_ee_id<<std::endl;
            }
            World_GraspSecondEE = World_Object*Object_SecondEE;
            #if SUPERHACK //raise more the grasp
            World_GraspSecondEE.p.z(World_GraspSecondEE.p.z() + 0.05);
            #endif
            cartesian_command move_command(cartesian_commands::MOVE, 1, node.next_grasp_id);
            tf::poseKDLToMsg(World_GraspSecondEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,move_command)); //move the next
            
            //From fixed to movable we will grasp the object
            cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
            result.push_back(std::make_pair(node.next_ee_id,grasp));
            #if SUPERHACK
            World_GraspSecondEE.p.z(World_GraspSecondEE.p.z() + 0.05);
            #endif
            //Back to before the grasp (retreat)
            tf::poseKDLToMsg(World_GraspSecondEE,move_command.cartesian_task);
            result.push_back(std::make_pair(node.next_ee_id,move_command));
            
        }
        else if (node.type=node_properties::node_properties::MOVABLE_TO_FIXED)
        {
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE;
            move_command.ee_grasp_id=node.current_ee_id;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=1;//do not parallelize with the fixed ee :)
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            super_compute_intergrasp_orientation();
            bool ok=getPostGraspMatrix(data.obj_id,node->grasp_id,Object_FirstEE);
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
            #endif
            #if SUPERHACK
            // consider the ungrasp trajectory as higher if ungrasping on a table
            ungrasp.cartesian_task.position.z = ungrasp.cartesian_task.position.z + 0.07;
            #endif
            cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
            result.push_back(std::make_pair(node.current_ee_id,ungrasp));
            cartesian_command move_away(cartesian_commands::HOME,1,-1);
            result.push_back(std::make_pair(node.current_ee_id,move_away));
        }
        else if (node.type=node_properties::node_properties::MOVABLE_TO_MOVABLE)
        {
            cartesian_command move_command;
            move_command.command=cartesian_commands::MOVE;
            move_command.ee_grasp_id=node.current_grasp_id;
            move_command.seq_num=0;//Care, we are parallelizing here!
            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,centroid_z,node);
            super_compute_intergrasp_orientation();
            bool ok=getPostGraspMatrix(data.obj_id,node->grasp_id,Object_FirstEE);
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
            #endif
            std::cout << "result.size() : " << result.size() << std::endl;
            #if SUPERHACK
            //superhack - part 2 - change the world!
            std::cout << "...and movable!" << std::endl;
            World_Object.M = fine_tuning[result.size()].M*World_Object.M;
            World_Object.p = World_Object.p + fine_tuning[result.size()].p;
            #endif
            bool ok = getPreGraspMatrix(data.obj_id,node.next_grasp_id,Object_SecondEE);
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
            cartesian_command move_away(cartesian_commands::HOME,1,-1);
            result.push_back(std::make_pair(node.current_ee_id,move_away));
        }
        else 
        {
            std::cout<<"SUPER ERROR!!"<<std::endl;
        }
        node_it=next_node_it;
    }
    return true;
}
