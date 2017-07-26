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

#ifndef S2C_IK_CONVERTER_H
#define S2C_IK_CONVERTER_H

#include <vector>
#include <XmlRpcValue.h>

#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

#include "shared_memory.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_item.h>
#include <ik_check_capability/ik_check_capability.h>
#include <dual_manipulation_ik_control/shared_ik_memory.h>

/// keep @p node_info out of any namespace for compatibility...
#define BIG_NR 10000
typedef dual_manipulation::shared::NodeTransitionTypes node_properties;
struct node_info
{
    node_properties type=node_properties::UNKNOWN;
    endeffector_id current_ee_id=-1, next_ee_id=-1;
    workspace_id next_workspace_id=-1, current_workspace_id=-1;
    grasp_id current_grasp_id=-1, next_grasp_id=-1;
    std::vector<endeffector_id> busy_ees;
    bool operator<(const node_info& a) const
    {
        uint64_t this_node_val = BIG_NR*BIG_NR*current_ee_id + BIG_NR*current_workspace_id + current_grasp_id;
        uint64_t other_node_val = BIG_NR*BIG_NR*a.current_ee_id + BIG_NR*a.current_workspace_id + a.current_grasp_id;
        return (this_node_val < other_node_val);
    }
};

/**
 * @brief All the matrixes in these structures are expressed in the notation Object_Matrix
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

/**
 * @brief A class to perform inverse kinematics queries used for converting a semantic path to a cartesian one
 */
class s2c_ik_converter
{
public:
    s2c_ik_converter(const databaseMapper& database);
    bool checkSingleGrasp(KDL::Frame& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const;
    bool checkSlidePoses(std::vector<KDL::Frame>& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes, const std::vector<KDL::Frame>& Object_ee_poses, const std::string& ee_name) const;
    bool compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object) const;
    bool compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object, dual_manipulation_shared::planner_item & filtered_source_nodes, dual_manipulation_shared::planner_item & filtered_target_nodes) const;
    static bool getGraspMatrixes(object_id object, node_info node, Object_GraspMatrixes& Object);
    static void clearCachedIkSolutions();
private:
    void compute_centroid(KDL::Frame& ws_centroid, const node_info& node) const;
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
    static std::map<std::pair<object_id,grasp_id>, Object_SingleGrasp> cache_matrixes;
    static std::map<node_info, KDL::JntArray> cache_ik_solutions;
    std::shared_ptr<dual_manipulation::ik_control::ikCheckCapability> ik_check_capability;
    mutable chain_and_solvers double_arm_solver;
    std::string robot_urdf;
    urdf::Model urdf_model;
    KDL::Tree robot_kdl;
    mutable std::default_random_engine generator;
    mutable std::uniform_real_distribution<double> distribution;
    // managing external parameters
    ros::NodeHandle nh;
    std::shared_ptr<XmlRpc::XmlRpcValue> ik_control_params;
    std::map<std::string,KDL::Chain> chains;
    std::map<std::string,KDL::Chain> chains_reverse;
    mutable KDL::Frame last_computed_target_pose;
    
};

#endif // S2C_IK_CONVERTER_H
