#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>
#include "shared_memory.h"
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <ik_check_capability/ik_check_capability.h>

#define BIG_NR 10000

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
    bool operator<(const node_info& a) const
    {
        uint64_t this_node_val = BIG_NR*BIG_NR*current_ee_id + BIG_NR*current_workspace_id + current_grasp_id;
        uint64_t other_node_val = BIG_NR*BIG_NR*a.current_ee_id + BIG_NR*a.current_workspace_id + a.current_grasp_id;
        return (this_node_val < other_node_val);
    }
};

/**
 * @brief All the matrixes in this structure are expressed in the notation Object_Matrix
 * 
 */

struct Object_SingleGrasp
{
    KDL::Frame PreGrasp;
    KDL::Frame PostGrasp;
    KDL::Frame Grasp;
};

struct Object_GraspMatrixes
{
    KDL::Frame PreGraspFirstEE,PreGraspSecondEE;
    KDL::Frame PostGraspFirstEE,PostGraspSecondEE;
    KDL::Frame GraspFirstEE,GraspSecondEE;
};

class chain_and_solvers
{
public:
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive* fksolver=0;
    KDL::ChainIkSolverPos_NR_JL* iksolver=0;
    KDL::ChainIkSolverVel_pinv* ikvelsolver=0;
    std::vector<std::string> joint_names;
    int index;
    KDL::JntArray q_min, q_max;
};


class semantic_to_cartesian_converter
{
public:
     semantic_to_cartesian_converter(const databaseMapper& database);
     bool convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const;
     bool checkSingleGrasp(KDL::Frame& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const;
     bool compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object) const;
     static bool getGraspMatrixes(object_id object, node_info node, Object_GraspMatrixes& Object);
private:
    node_info find_node_properties(const std::vector< dual_manipulation_shared::planner_item >& path, const std::vector< dual_manipulation_shared::planner_item >::const_iterator& node, std::vector< dual_manipulation_shared::planner_item >::const_iterator& next_node) const;
    void compute_centroid(double& centroid_x,double& centroid_y,double& centroid_z, const node_info& node) const;
    static bool getGraspMatrixes(object_id object, grasp_id grasp, Object_SingleGrasp& Matrixes);
    bool check_ik(std::string ee_name, KDL::Frame World_EE) const;
    void addNewFilteredArc(const node_info& node, dual_manipulation_shared::planner_item& filtered_source_node, dual_manipulation_shared::planner_item& filtered_target_node) const;
    void initialize_solvers(chain_and_solvers* container) const;
    void parseParameters(XmlRpc::XmlRpcValue& params);
    bool publishConfig(const std::vector<std::string>& joint_names, const KDL::JntArray& q) const;
    bool normalizePoses(std::vector< geometry_msgs::Pose >& poses);
    static bool normalizePose(geometry_msgs::Pose& pose);
    static bool getCachedIKSolution(const node_info& node, KDL::JntArray& q_out);
    static void setCachedIKSolution(const node_info& node, const KDL::JntArray& q_out);
    static void eraseCachedIKSolution(const node_info& node);
private:
     const databaseMapper& database;
     std::map<int,KDL::Frame> fine_tuning;
     std::vector<KDL::Rotation> sphere_sampling;
     static std::map<std::pair<object_id,grasp_id>, Object_SingleGrasp> cache_matrixes;
     static std::map<node_info, KDL::JntArray> cache_ik_solutions;
     mutable dual_manipulation::ik_control::ikCheckCapability *ik_check_capability;
     mutable chain_and_solvers double_arm_solver;
     std::string robot_urdf;
     urdf::Model urdf_model;
     KDL::Tree robot_kdl;
     mutable std::default_random_engine generator;
     mutable std::uniform_real_distribution<double> distribution;
     // managing external parameters
     XmlRpc::XmlRpcValue ik_control_params;
     std::vector<std::string> chain_names_list;
     std::map<std::string,KDL::Chain> chains;
     std::map<std::string,KDL::Chain> chains_reverse;
     
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
