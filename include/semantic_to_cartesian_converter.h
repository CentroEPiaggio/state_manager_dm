#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>
#include "shared_memory.h"
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <dual_manipulation_shared/node_transitions.h>
#include "s2c_ik_converter.h"

class semantic_to_cartesian_converter
{
public:
    semantic_to_cartesian_converter(const databaseMapper& database);
    
    /**
     * @brief Convert a vector of planner_item @p path into a sequence of cartesian commands @p result
     * 
     * @param data shared memory containing various information about the plan
     * @param filtered_source_nodes vector which will contain a number of source nodes to filter for next planning, if any
     * @param filtered_target_nodes vector which will contain a number of target nodes to filter for next planning, if any
     * 
     * @return false if the conversion failed, true otherwise
     */
    bool convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const;
    
    /**
     * @brief Check a single grasp, knowing the object pose, information about the grasp, whether it is the first or last in the sequence, and sets filter nodes for the next planning, if any
     * 
     * @param World_Object pose of the object in world frame
     * @param node information about the current node to be converted
     * @param data shared memory containing various information about the plan
     * @param first_node is this the first node?
     * @param last_node is this the last node?
     * @param filtered_source_nodes vector which will contain a number of source nodes to filter for next planning, if any
     * @param filtered_target_nodes vector which will contain a number of target nodes to filter for next planning, if any
     * 
     * @return false if the conversion failed, true otherwise
     */
    bool checkSingleGrasp(KDL::Frame& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const;
    
    /**
     * @brief Compute a candidate location for an intergrasp (grasp exchange between two movable end-effectors) to happen
     * 
     * @param World_Object pose of the object in world frame to be computed via this function
     * @param node information about the current node to be converted
     * @param object the id of the object
     * 
     * @return false if no integrasp configuration could be found, true otherwise
     */
    bool compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object) const;
    
    /**
     * @brief Static function to get grasp matrixes (pre / grasp / post) associated to a given object and pair of grasp_id
     * 
     * @param object the id of the object
     * @param node information about the current node to be converted
     * @param Object grasp matrixes returned
     * 
     * @return true on success (matrix have been found)
     */
    static bool getGraspMatrixes(object_id object, node_info node, Object_GraspMatrixes& Object);
    
private:
    
    /**
     * @brief Find properties associated to the next node
     * 
     * @param path a vector of planner_item to be converted
     * @param node an iterator to the current node
     * @param next_node an iterator to the next node (which will be updated - can "jump" due to non-considered cases)
     * 
     * @return the node_info object containing the found properties
     */
    node_info find_node_properties(const std::vector< dual_manipulation_shared::planner_item >& path, const std::vector< dual_manipulation_shared::planner_item >::const_iterator& node, std::vector< dual_manipulation_shared::planner_item >::const_iterator& next_node) const;
    
private:
    
    /// manage information from the database
    const databaseMapper& database;
    
    /// perform inverse kinematics and other "heavy" duties
    std::unique_ptr<s2c_ik_converter> s2cik;
    
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
