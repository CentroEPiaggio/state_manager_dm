#ifndef IK_CHECKING_GRASP_SUBSTATE_H
#define IK_CHECKING_GRASP_SUBSTATE_H

#include "abstract_state.h"
#include "transitions.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dual_manipulation_shared/databasemapper.h>
#include "semantic_to_cartesian_converter.h"
#include <XmlRpcValue.h>

class ik_checking_grasp_substate : public abstract_state<ik_transition>
{
public:
    ik_checking_grasp_substate(ik_shared_memory& data);
    bool isComplete();
    void run();
    std::map<ik_transition,bool> getResults();
    virtual std::string get_type();
    void reset();
    void parseParameters(XmlRpc::XmlRpcValue& params);
private:
    int get_grasp_id_from_database(int object_id, geometry_msgs::Pose pose, int ee_id);
  
    ik_shared_memory& data_;
    bool need_replanning_ = false;
    bool is_complete_ = false;
    bool failed_ = false;
    bool soft_failed_ = false;
    tf::TransformListener tf_listener_;
    const databaseMapper& database_;
    semantic_to_cartesian_converter converter_;
    int table_ee_id=3;
    int edge_ee_id=4;
    int ext_ee_id=5;
    int wall_ee_id=6;
};

#endif // IK_CHECKING_GRASP_SUBSTATE_H
