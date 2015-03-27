#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>
#include "shared_memory.h"
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <kdl/frames.hpp>

enum class node_properties
{
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
     bool convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector<dual_manipulation_shared::planner_item>& path, const shared_memory& data);
private:
    node_info find_node_properties(const std::vector<dual_manipulation_shared::planner_item>& path, const std::vector<dual_manipulation_shared::planner_item>::const_iterator& node, std::vector<dual_manipulation_shared::planner_item>::const_iterator& next_node);
    void compute_centroid(double& centroid_x,double& centroid_y,double& centroid_z, const node_info& node);
    bool getPreGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE);
    bool getGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE);
    bool getPostGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE);
    bool inverse_kinematics(std::string ee_name, KDL::Frame cartesian);
    bool check_ik(std::string current_ee_name, KDL::Frame World_FirstEE, std::string next_ee_name, KDL::Frame World_SecondEE, std::vector< double >& result_first, std::vector< double >& result_second);
    bool check_ik(std::string ee_name, KDL::Frame World_EE);
    bool compute_intergrasp_orientation(KDL::Vector World_centroid, KDL::Frame& World_Object, 
                                                                         endeffector_id ee_id, endeffector_id next_ee_id, grasp_id grasp, 
                                                                         grasp_id next_grasp, object_id object,
                                                                         bool movable,bool next_movable,int aggiuntivo);
    
private:
     databaseMapper database;
     int counter=0;
     std::map<int,KDL::Frame> fine_tuning;
     std::vector<KDL::Rotation> sphere_sampling;
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
