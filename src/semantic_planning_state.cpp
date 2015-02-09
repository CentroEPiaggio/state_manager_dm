#include "../include/semantic_planning_state.h"
#include <dual_manipulation_shared/geometry_tools.h>
#include <ros/init.h>


semantic_planning_state::semantic_planning_state(shared_memory& data):data(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
        char** argv;
        ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
}

std::map< transition, bool > semantic_planning_state::getResults()
{

}

void semantic_planning_state::run()
{
    //convert cartesian into semantic
    bool source_found=false, target_found=false;
    workspace_id source,target;
    for (auto workspace: database.WorkspaceGeometry)
    {
        double xs=data.source_position.position.x;
        double ys=data.source_position.position.y;
        double xt=data.target_position.position.x;
        double yt=data.target_position.position.y;
        std::vector<Point> temp;
        for (auto point : workspace.second)
            temp.emplace_back(point.first,point.second);
        geom.order_points(temp);
        if (geom.point_in_ordered_polygon(x,y,temp))
        {
            std::cout<<"source position is in workspace "<<workspace.first<<std::endl;
            source_found=true;
            source=workspace.first;
        }
        if (geom.point_in_ordered_polygon(x,y,temp))
        {
            std::cout<<"source position is in workspace "<<workspace.first<<std::endl;
            target_found=true;
            target=workspace.first;
        }
    }
    if (!source_found)
        std::cout<<"source position is outside the workspaces!!!"<<std::endl;
    if (!target_found)
        std::cout<<"target position is outside the workspaces!!!"<<std::endl;
    
    
    //Call planner with the right parameters
    srv.request.command="plan";
    srv.request.source.grasp_id=data.source_grasp;
    srv.request.source.workspace_id=source;
    srv.request.destination.grasp_id=data.target_grasp;
    srv.request.destination.workspace_id=target;
    
    if (client.call(srv))
    {
        ROS_INFO("Planning Request accepted: %d", (int)srv.response.ack);
        for (auto node:srv.response.path)
            std::cout<<node.grasp_id<<" "<<node.workspace_id<<std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::planner_service");
        return;
    }
    
    ros::spin();
    
    
    //TODO convert semantic plan into cartesian vector
            //for each grasp understand the e.e
            //for each workspace find the centroid of the polygon
            //create a cartesian path from (grasp/workspace) to (e.e./centroid)
    
    
    
    //TODO parallelize movements between arms?!?
}

bool semantic_planning_state::isComplete()
{
    return false;
}

std::string semantic_planning_state::get_type()
{
    return "semantic_planning_state";
}