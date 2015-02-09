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
        if (geom.point_in_ordered_polygon(xs,ys,temp))
        {
            std::cout<<"source position is in workspace "<<workspace.first<<std::endl;
            source_found=true;
            source=workspace.first;
        }
        if (geom.point_in_ordered_polygon(xt,yt,temp))
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
    /*
     * grasp1 W1 (table) -> grasp2 W1 (table/ee) -> grasp2 W2 (ee)      -> grasp3 W2 (ee)      -> grasp3 W3 (ee/table) -> grasp4 W3(table)
     * not used          -> z=0 x,y=centroid W1  -> z=1 x,y=centroid W2 -> z=1 x,y=centroid W2 -> z=0 x,y=centroid W3  -> not used
     */
    
    grasp_id previous_grasp, next_grasp;
    workspace_id previous_workspace, next_workspace;
    for (auto node=srv.response.path.begin();node!=srv.response.path.end();++node)
    {
        auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
        auto ee_name=std::get<0>(database.EndEffectors.at(ee_id));
        double centroid_x=0, centroid_y=0;
        for (auto workspace: database.WorkspaceGeometry.at(node->workspace_id))
        {
            centroid_x+=workspace.first;
            centroid_y+=workspace.second;
        }
        centroid_x=centroid_x/database.WorkspaceGeometry.at(node->workspace_id).size();
        centroid_y=centroid_y/database.WorkspaceGeometry.at(node->workspace_id).size();
        bool movable=std::get<1>(database.EndEffectors.at(ee_id));
        
    }
    
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