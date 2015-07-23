#ifndef ik_moving_substate_H
#define ik_moving_substate_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/ik_response.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <std_msgs/String.h>
#include <mutex>

class ik_moving_substate : public abstract_state<ik_transition>
{
public:
    ik_moving_substate(ik_shared_memory& data);
    bool isComplete();
    void run();
    std::map<ik_transition,bool> getResults();
    virtual std::string get_type();
private:
    ik_shared_memory& data_;
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::ik_service srv;
    ros::Subscriber exe_sub,grasp_sub,ungrasp_sub;
    bool initialized;
    void callback(const dual_manipulation_shared::ik_response::ConstPtr& str, std::string type);
    bool parallelize_planning;
    
    void reset();
    
    /**
     * @brief change frame of reference of the grasp trajectory to the current object frame
     * 
     * @param object_pose
     *        the current pose of the object (to be used for the local grasp trajectory @ee_pose
     * @param ee_pose
     *        a grasp trajectory expressed in object_frame, which will be returned expressed in world frame
     *        (i.e., each frame will be "pre-multiplied" by object_pose)
     */
    void change_frame_to_pose_vector(geometry_msgs::Pose object_pose_msg, std::vector<geometry_msgs::Pose>& ee_pose);

    databaseMapper db_mapper;
    bool move_sent;
    bool grasping;
    int moving_executed;
    bool failed;
    int sequence_counter;
    std::set<int> pending_sequence_numbers;
    std::mutex moving_executed_mutex;
    moving_cmd commands;
};

#endif // ik_moving_substate_H
