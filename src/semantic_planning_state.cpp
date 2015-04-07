#include "../include/semantic_planning_state.h"
#include <dual_manipulation_shared/geometry_tools.h>
#include <ros/init.h>
#include <dual_manipulation_shared/stream_utils.h>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/String.h>


semantic_planning_state::semantic_planning_state(shared_memory& data):data(data),converter(database)
{
    if( !ros::isInitialized() )
    {
        int argc;
        char** argv;
        ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
    completed=false;
}

std::map< transition, bool > semantic_planning_state::getResults()
{
    return internal_state;
}


void semantic_planning_state::run()
{
    completed=false;
    internal_state.clear();
    //convert cartesian into semantic
    bool source_found=false, target_found=false;
    workspace_id source,target;
    double xs=data.source_position.position.x;
    double ys=data.source_position.position.y;
    double xt=data.target_position.position.x;
    double yt=data.target_position.position.y;
    
    for (auto workspace: database.WorkspaceGeometry)
    {
        std::vector<Point> temp;
        for (auto point : workspace.second)
            temp.emplace_back(point.first,point.second);
        if (geom.point_in_ordered_polygon(xs,ys,temp))
        {
            std::cout<<"source position is in workspace "<<workspace.first<<std::endl;
            source_found=true;
            source=workspace.first;
        }
        if (geom.point_in_ordered_polygon(xt,yt,temp))
        {
            std::cout<<"target position is in workspace "<<workspace.first<<std::endl;
            target_found=true;
            target=workspace.first;
        }
    }
    if (!source_found)
        std::cout<<"source position is outside the workspaces!!!"<<std::endl;
    if (!target_found)
        std::cout<<"target position is outside the workspaces!!!"<<std::endl;
    if (!source_found || !target_found)
    {
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
    }
    int max_counter=25;
    while(max_counter>0)
    {
        max_counter--;
        //Call planner with the right parameters
        srv.request.command="plan";
        srv.request.source.grasp_id=data.source_grasp;
        srv.request.source.workspace_id=source;
        srv.request.destination.grasp_id=data.target_grasp;
        srv.request.destination.workspace_id=target;
        srv.request.filtered_source_nodes=data.filtered_source_nodes;
        srv.request.filtered_target_nodes=data.filtered_target_nodes;

        if (client.call(srv))
        {
            ROS_INFO("Planning Request accepted: %d", (int)srv.response.ack);
            for (auto node:srv.response.path)
                std::cout<<node.grasp_id<<" "<<node.workspace_id<<std::endl;
        }
        else
        {
            ROS_ERROR("Failed to call service dual_manipulation_shared::planner_service");
            internal_state.insert(std::make_pair(transition::failed_plan,true));
            completed=true;
            return;
        }

        ros::spinOnce();

        if (srv.response.path.size()<2)
        {
            ROS_ERROR("The planner returned a path with less than 2 nodes");
            internal_state.insert(std::make_pair(transition::failed_plan,true));
            completed=true;
            return;
        }
        if (srv.response.path.size()<3)
        {
            ROS_INFO("The planner returned a path with less than 3 nodes, should we handle this in a different way??");
            internal_state.insert(std::make_pair(transition::failed_plan,true));
            completed=true;
            return;
        }
        std::vector<std::pair<endeffector_id,cartesian_command>> result;
        bool converted=converter.convert(result,srv.response.path,data,data.filtered_source_nodes,data.filtered_target_nodes);
        if (!converted)
        {
            std::cout<<"Error converting semantic to cartesian!, I will try again for "<<max_counter<<std::endl;
            continue;
        }
        std::cout << "=== Cartesian plan print-out ===" << std::endl;
        std::cout << "( Note that grasp/ungrasp poses are the object poses, not the end-effector ones )" << std::endl;
        data.cartesian_plan = result;
        for (auto i:result)
            std::cout<<i<<std::endl;
        std::cout << "=== end of cartesian plan print-out ===" << std::endl;
    }
    //TODO parallelize movements between arms?!?
    internal_state.insert(std::make_pair(transition::good_plan,true));
    completed=true;
    return;
    
}

bool semantic_planning_state::isComplete()
{
    return completed;
}

std::string semantic_planning_state::get_type()
{
    return "semantic_planning_state";
}