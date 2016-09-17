#include "semantic_to_cartesian_converter.h"
#include <tf_conversions/tf_kdl.h>

#define DEBUG 0 // if 1, print some more information

#define CLASS_NAMESPACE "semantic_to_cartesian_converter::"

semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database) : database(database)
{
    s2cik.reset(new s2c_ik_converter(this->database));
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
    dual_manipulation::shared::NodeTransitionTypes transit_type = dual_manipulation::shared::NodeTransitionTypes::UNKNOWN;
    bool next_movable=false;
    while (!found && next_node!=path.end())
    {
        next_node++;
        if (next_node!=path.end())
        {
            next_ee_id = std::get<1>(database.Grasps.at(next_node->grasp_id));
            next_workspace_id = next_node->workspace_id;
            next_movable=std::get<1>(database.EndEffectors.at(next_ee_id));
            
            // check for supported cases:
            // - supported type #1: change of workspace with the same grasp, movable end-effector
            bool supported_node = ((node->workspace_id != next_workspace_id) && (node->grasp_id == next_node->grasp_id) && movable);
            // - supported type #2: change of grasp with an allowed transition (not necessarily in the same workspace, as we are moving...)
            supported_node = supported_node || (database.Grasp_transitions.count(node->grasp_id) && database.Grasp_transitions.at(node->grasp_id).count(next_node->grasp_id));
            if(!supported_node)
            {
                std::cout << CLASS_NAMESPACE << __func__ << " : there was a change of grasp which was not present in the database: this is NOT supported!" << std::endl;
                std::cout << CLASS_NAMESPACE << __func__ << " : source(g,w)=(" << node->grasp_id << "," << node->workspace_id << ") > target(g,w):(" << next_node->grasp_id << "," << next_node->workspace_id << ")" << std::endl;
                abort();
            }
            
            std::set<endeffector_id> tmp_ees;
            // NOTE: transitions from database are only inside the same workspace
            if(node->workspace_id==next_workspace_id)
            {
                database.getTransitionInfo(node->grasp_id, next_node->grasp_id, transit_type, tmp_ees);
                result.type = transit_type;
                result.busy_ees.insert(tmp_ees.begin(),tmp_ees.end());
            }
            // NOTE: simplify when a movable end-effector is changing workspace but keeping the same grasp
            if ((ee_id==next_ee_id) && (node->workspace_id!=next_workspace_id) && movable && next_movable && (node->grasp_id == next_node->grasp_id))
                continue;
            else
            {
                found=true;
                break;
            }
        }
    }
    if (found)
    {
        if (movable && !next_movable) result.type=node_properties::UNGRASP;          //found, one is movable, change on ground
        if (!movable && next_movable) result.type=node_properties::GRASP;          //found, one is movable, change on ground
        if (movable && next_movable) result.type=node_properties::EXCHANGE_GRASP;        //found, both ee are movable: change above ground
        if (!movable && !next_movable) result.type=node_properties::UNKNOWN;            //if (!movable && !next_movable)
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

bool semantic_to_cartesian_converter::convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    // 1) Clearing result vector
    result.clear();
    
    // 2) Clear any previously found intergrasp configuration
    s2cik->clearCachedIkSolutions();
    
    // 3) Start of the main conversion loop
    for (auto node_it=path.begin();node_it!=path.end();)//++node)
    {
        // 3.1) Getting preliminary info for the current node
        std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it=node_it;
        node_info node = find_node_properties(path,node_it,next_node_it);
        //---------------------------
        
        if (node.type==node_properties::LAST_EE_FIXED)
        {
            if(manage_transition_last_ee_fixed(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
            {
                // do nothing
            }
            else
                return false;
        }
        else if (node.type==node_properties::LAST_EE_MOVABLE)
        {
            if(manage_transition_last_ee_movable(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
            {
                // do nothing
            }
            else
                return false;
        }
        else if (node.type==node_properties::UNKNOWN)
        {
            if(manage_transition_unknown(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
            {
                // do nothing
            }
            else
                return false;
        }
        else if (node.type==node_properties::GRASP)
        {
#if DEBUG
            std::cout << "Semantic to cartesian: node.type==node_properties::GRASP" << std::endl;
#endif
            if(manage_transition_grasp(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
            {
                // do nothing
            }
            else
                return false;
        }
        else if (node.type==node_properties::UNGRASP)
        {
#if DEBUG
            std::cout << "Semantic to cartesian: node.type==node_properties::UNGRASP" << std::endl;
#endif
            if(manage_transition_ungrasp(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
            {
                // do nothing
            }
            else
                return false;
        }
        else if (node.type==node_properties::EXCHANGE_GRASP)
        {
#if DEBUG
            std::cout << "Semantic to cartesian: node.type==node_properties::EXCHANGE_GRASP" << std::endl;
#endif
            if(manage_transition_exchange_grasp(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
            {
                // do nothing
            }
            else
                return false;
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

bool semantic_to_cartesian_converter::checkSingleGrasp(KDL::Frame& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    return s2cik->checkSingleGrasp(World_Object,node,data,first_node,last_node,filtered_source_nodes,filtered_target_nodes);
}

bool semantic_to_cartesian_converter::compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object) const
{
    return s2cik->compute_intergrasp_orientation(World_Object,node,object);
}

bool semantic_to_cartesian_converter::getGraspMatrixes(object_id object, node_info node, Object_GraspMatrixes& Object)
{
    return s2c_ik_converter::getGraspMatrixes(object,node,Object);
}

void semantic_to_cartesian_converter::getGraspMatrixesFatal(const shared_memory& data, node_info node, Object_GraspMatrixes& Object)
{
    if (!s2c_ik_converter::getGraspMatrixes(data.obj_id, node, Object))
    {
        std::cout << CLASS_NAMESPACE << __func__ << " : unable to get grasp matrixes for object " << data.object_name << " | grasps #" << node.current_grasp_id << "," << node.next_grasp_id << std::endl;
        abort();
    }
}

bool semantic_to_cartesian_converter::manage_transition_unknown(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    std::cout << CLASS_NAMESPACE << " : ERROR, the planner returned two nodes for which no known transition is implemented!!"<<std::endl;
    return false;
}

bool semantic_to_cartesian_converter::manage_transition_last_ee_movable(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
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
    
    return true;
}

bool semantic_to_cartesian_converter::manage_transition_last_ee_fixed(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    return true;
}

bool semantic_to_cartesian_converter::manage_transition_grasp(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    KDL::Frame World_Object,World_GraspSecondEE;
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    // 3.6) compute a rough position of the place where the change of grasp will happen
    if (!s2cik->checkSingleGrasp(World_Object,node,data,node_it==path.begin(),false,filtered_source_nodes,filtered_target_nodes))
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
    
    return true;
}

bool semantic_to_cartesian_converter::manage_transition_ungrasp(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    KDL::Frame World_Object;
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    cartesian_command move_command;
    move_command.command=cartesian_commands::MOVE_BEST_EFFORT;
    move_command.ee_grasp_id=node.current_grasp_id;
    move_command.seq_num=1;//do not parallelize with the fixed ee :)
    cartesian_command move_no_coll_command(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, node.current_grasp_id);
    // 3.6) compute a rough position of the place where the change of grasp will happen
    if (!s2cik->checkSingleGrasp(World_Object,node,data,false,((next_node_it+1) == path.end()),filtered_source_nodes,filtered_target_nodes))
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
    
    return true;
}

bool semantic_to_cartesian_converter::manage_transition_exchange_grasp(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    KDL::Frame World_Object,World_GraspSecondEE;
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    cartesian_command move_command(cartesian_commands::MOVE,0,-1); // Care, we are parallelizing here!
    // 3.6) compute a rough position of the place where the change of grasp will happen
    if(!s2cik->compute_intergrasp_orientation(World_Object,node,data.obj_id,filtered_source_nodes,filtered_target_nodes))
        return false;
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
    
    return true;
}
