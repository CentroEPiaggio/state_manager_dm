#ifndef ik_moving_substate_H
#define ik_moving_substate_H

#include "abstract_state.h"
#include "transitions.h"
#include "ros_server.h"
#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <std_msgs/String.h>

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
    ros::Subscriber lsub;
    ros::Subscriber rsub;
    ros::Subscriber bimanualsub;
    ros::Subscriber lgraspsub;
    ros::Subscriber rgraspsub;
    bool initialized;
    void callback_l(const std_msgs::String::ConstPtr& str);
    void callback_r(const std_msgs::String::ConstPtr& str);
    void callback_bimanual(const std_msgs::String::ConstPtr& str);
    void callback_l_grasp(const std_msgs::String::ConstPtr& str);
    void callback_r_grasp(const std_msgs::String::ConstPtr& str);
    
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

    std::map<cartesian_commands,std::string> command_map;
    databaseMapper db_mapper;
    bool move_sent;
    int moving_executed;
};

#endif // ik_moving_substate_H
