/*********************************************************************
*
* Software License Agreement (BSD License)
*
*
*  Copyright (c) 2016, Hamal Marino <hamal dot marino at gmail dot com>
*  Copyright (c) 2017, George Jose Pollayil <gpollayil at gmail dot com>
*  Copyright (c) 2017, Mathew Jose Pollayil <mathewjosepollayil at gmail dot com>
*  Copyright (c) 2017, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>
#include "shared_memory.h"
#include <dual_manipulation_shared/planner_serviceResponse.h>
#include <dual_manipulation_shared/node_transitions.h>
#include "s2c_ik_converter.h"
#include <functional>

// A function macro to make sure all functions to handle transitions have the same signature
#define MANAGE_TRANSITION_FUN_MACRO(function_name) bool function_name(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const node_info& node, const std::vector< dual_manipulation_shared::planner_item >::const_iterator node_it, const std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes)

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
    bool convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes);
    
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

    /// adding information about s and t positions for correct hand positioning during sliding
    /// Note: the following are correctly assigned only in semantic_planning_state::run before converter is called
    geometry_msgs::Pose source_position = geometry_msgs::Pose();
    geometry_msgs::Pose target_position = geometry_msgs::Pose();

    // current source grasp id
    int current_source_grasp = -1;

    // adding info about the two main table and edge grasp ids for sliding
    int sCbt = 401641; // side C bottom table
    int sCtt = 401644; // side C top table
    int sCbe = 401841; // side C bottom edge
    int sCte = 401844; // side C top edge

    // Info on if current transition is TILT
    bool current_transition_tilting = false;
    
private:

    /**
     * @brief Sets Object_PreSlide and Object_Slide of the class according to best hand position for achieving good sliding
     *        using a dot product test
     * 
     * @param source_position a geometry_msg Pose with info on source position and orientation
     * @param target_position a geometry_msg Pose with info on target position and orientation
     * 
     * @return true if the two KDL Frames were set correctly, false otherwise
     */
    bool set_hand_pose_sliding(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position);

    /**
     * @brief Sets Object_PreTilt and Object_Tilt of the class according to best hand position for achieving good tilting
     *        using a dot product test (same as sliding)
     * 
     * @param source_position a geometry_msg Pose with info on source position and orientation
     * @param target_position a geometry_msg Pose with info on target position and orientation
     * 
     * @return true if the two KDL Frames were set correctly, false otherwise
     */
    bool set_hand_pose_tilting(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position);
        
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
    
    /**
     * @brief A utility function to get grasp matrixes associated to an object, aborting the program in case errors are found
     */
    inline static void getGraspMatrixesFatal(const shared_memory& data, node_info node, Object_GraspMatrixes& Object);
    
    /// from now on, a set of functions to manage the various transitions is laid out; must all have the same signature
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_unknown);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_last_ee_fixed);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_last_ee_movable);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_grasp);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_ungrasp);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_exchange_grasp);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_move_nonblocking);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_slide);
    MANAGE_TRANSITION_FUN_MACRO(manage_transition_tilt);
    
private:
    
    /// manage information from the database
    const databaseMapper& database;
    
    /// perform inverse kinematics and other "heavy" duties
    std::unique_ptr<s2c_ik_converter> s2cik;
    
    /// type of function which manage a single transition
    typedef decltype(&semantic_to_cartesian_converter::manage_transition_unknown) my_fun_t;
    /// map containing information about correspondences between functions and transition types
    std::map<dual_manipulation::shared::NodeTransitionTypes,my_fun_t> manage_transition_by_type;
    
    bool use_slide =  true;
    KDL::Frame Object_PreSlide, Object_Slide;

    bool use_tilt =  true;
    KDL::Frame Object_PreTilt, Object_Tilt;
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
