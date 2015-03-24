#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>

enum class node_properties
{
    FINAL_NODE,                //  if (final_node==path.end())
    MOVABLE_TO_FIXED,          //found, one is movable, change on ground
    FIXED_TO_MOVABLE,          //found, one is movable, change on ground
    MOVABLE_TO_MOVABLE,        //found, both ee are movable: change above ground
    FIXED_TO_FIXED,            //if (!movable && !next_movable)
    LAST_EE_FIXED,             //3.4.1) if not found, than ee_id is the last end effector in the path //not found not movable
    LAST_EE_MOVABLE            //else //not found->last e.e, movable
};

struct node_info
{
    node_properties type=node_properties::FIXED_TO_FIXED;
    endeffector_id current_ee_id=-1, next_ee_id=-1;
    workspace_id next_workspace_id=-1, workspace_id=-1;
    grasp_id grasp_id=-1, next_grasp_id=-1;
};

class semantic_to_cartesian_converter
{
public:
     semantic_to_cartesian_converter(const databaseMapper& database);
    
private:
    node_info find_node_properties(const dual_manipulation_shared::planner_serviceResponse::_path_type& path, const std::vector::iterator& node);
    
    void initialize_grasped_map(const dual_manipulation_shared::planner_serviceResponse::_path_type& path);
    
private:
     databaseMapper database;
     std::map<endeffector_id,bool> ee_grasped;
     
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
