#ifndef SEMANTIC_PLANNING_STATE_H
#define SEMANTIC_PLANNING_STATE_H

#include "abstract_state.h"
#include "semantic_to_cartesian_converter.h"
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
    virtual void reset();
private:

    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Publisher good_grasps_pub;
    dual_manipulation_shared::planner_service srv;
    databaseMapper database;
    geometry_tools geom;
    shared_memory& data;
    bool completed;
    std::map< transition, bool > internal_state;
    semantic_to_cartesian_converter converter;
};

#endif // SEMANTIC_PLANNING_STATE_H
