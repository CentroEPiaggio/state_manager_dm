#ifndef GETTING_INFO_STATE_H
#define GETTING_INFO_STATE_H

#include <abstract_state.h>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <dual_manipulation_shared/databasemapper.h>

class getting_info_state : public abstract_state<transition>
{
public:
    getting_info_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
private:
    shared_memory& data_;
    bool fresh_data;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::ServiceClient planner_client, gui_target_client, scene_object_client;
    
    databaseMapper db_mapper_;

    void get_start_position_from_vision(visualization_msgs::Marker& source_marker);
    
    /**
     * @brief given and object id and its pose, return the associated grasp id (checked from the database)
     * 
     * @param object_id
     *   the object_id of the object in the scene
     * @param pose
     *   the pose for which we want to know the grasp
     * @param ee_id
     *   the id of the end-effector, decided from the user (default = table)
     *   (at now table is the only implemented one)
     * @return the index of the associated grasp id
     */
    int get_grasp_id_from_database(int object_id, geometry_msgs::Pose pose, int ee_id = 3);
    
    void get_target_position_from_user(visualization_msgs::Marker& target_marker);
};

#endif // GETTING_INFO_STATE_H
