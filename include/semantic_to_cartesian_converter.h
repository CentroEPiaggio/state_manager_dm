#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>
#include "shared_memory.h"
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <kdl/frames.hpp>

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
    workspace_id next_workspace_id=-1, current_workspace_id=-1;
    grasp_id current_grasp_id=-1, next_grasp_id=-1;
};

class semantic_to_cartesian_converter
{
public:
     semantic_to_cartesian_converter(const databaseMapper& database);
     bool convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const dual_manipulation_shared::planner_serviceResponse_< std::allocator >::_path_type& path, const shared_memory& data);
private:
    node_info find_node_properties(const dual_manipulation_shared::planner_serviceResponse::_path_type& path, const dual_manipulation_shared::planner_serviceResponse::_path_type::iterator& node, dual_manipulation_shared::planner_serviceResponse::_path_type::iterator& next_node);
    void compute_centroid(double& centroid_x,double& centroid_y,double& centroid_z, const node_info& node);
    bool getPreGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE);
    bool getPostGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE);
    bool inverse_kinematics(std::string ee_name, KDL::Frame cartesian);
    bool check_ik(endeffector_id ee_id, KDL::Frame World_FirstEE, endeffector_id next_ee_id, KDL::Frame World_SecondEE);
    bool compute_intergrasp_orientation(KDL::Vector World_centroid, KDL::Frame& World_Object, 
                                                                         endeffector_id ee_id, endeffector_id next_ee_id, grasp_id grasp, 
                                                                         grasp_id next_grasp, object_id object,
                                                                         bool movable,bool next_movable,int aggiuntivo);
    
private:
     databaseMapper database;
     int counter=0;
     std::map<int,KDL::Frame> fine_tuning;
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
