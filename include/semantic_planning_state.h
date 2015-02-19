#ifndef SEMANTIC_PLANNING_STATE_H
#define SEMANTIC_PLANNING_STATE_H

#include "abstract_state.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/geometry_tools.h>
#include <dual_manipulation_shared/planner_service.h>
#include <ros/node_handle.h>

class semantic_planning_state : public abstract_state<transition>
{
public:
    semantic_planning_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();

    /**
     * @brief Expects a cartesian source position, an object ID and a cartesian target position from the shared_memory
     * 
     * @return void
     */
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    bool semantic_to_cartesian(std::vector<std::pair<endeffector_id,cartesian_command>>& result,const dual_manipulation_shared::planner_serviceResponse::_path_type& path);//TODO make private
    
private:
    void compute_centroid(double& centroid_x,double& centroid_y, workspace_id w_id);
    bool compute_intergrasp_orientation(double centroid_x, double centroid_y, double centroid_z, KDL::Frame& World_Object, endeffector_id ee_id, endeffector_id next_ee_id, grasp_id grasp, grasp_id next_grasp, object_id object);
    bool inverse_kinematics(std::string ee_name, KDL::Frame cartesian);
    bool check_ik(endeffector_id ee_id, KDL::Frame World_FirstEE, endeffector_id next_ee_id, KDL::Frame World_SecondEE);
    bool getPreGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE);
    ros::NodeHandle n;
    ros::ServiceClient client;
    dual_manipulation_shared::planner_service srv;
    databaseMapper database;
    geometry_tools geom;
    shared_memory& data;
    bool completed;
    std::map< transition, bool > internal_state;
};

#endif // SEMANTIC_PLANNING_STATE_H
