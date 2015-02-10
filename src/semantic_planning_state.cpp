#include "../include/semantic_planning_state.h"
#include <dual_manipulation_shared/geometry_tools.h>
#include <ros/init.h>

#define HIGH 0.5

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

void semantic_planning_state::compute_centroid(double& centroid_x,double& centroid_y, workspace_id w_id)
{
    centroid_x=0;
    centroid_y=0;
    for (auto workspace: database.WorkspaceGeometry.at(w_id))
    {
        centroid_x+=workspace.first;
        centroid_y+=workspace.second;
    }
    centroid_x=centroid_x/database.WorkspaceGeometry.at(w_id).size();
    centroid_y=centroid_y/database.WorkspaceGeometry.at(w_id).size();
    return;
}

bool semantic_planning_state::semantic_to_cartesian(std::vector<std::pair<endeffector_id,cartesian_command>>& result,const dual_manipulation_shared::planner_serviceResponse::_path_type& path)
{
    result.clear();
    for (auto node=path.begin();node!=path.end();)//++node)
    {
        //Getting info for the current node
        auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
        auto ee_name=std::get<0>(database.EndEffectors.at(ee_id));
        double centroid_x=0, centroid_y=0, centroid_z=0;
        bool movable=std::get<1>(database.EndEffectors.at(ee_id));
        
        auto final_node=node;
        final_node++;
        if (final_node==path.end())
        {
            break;
        }
        //From now on node is not the last in the path
        //searching for the next node with a different end effector than the current one
        bool found=false;
        endeffector_id next_ee_id;
        workspace_id next_workspace_id;
        bool next_movable=false;
        auto next_node=node;
        while (!found && next_node!=path.end())
        {
            next_node++;
            if (next_node!=path.end())
            {
                next_ee_id = std::get<1>(database.Grasps.at(next_node->grasp_id));
                next_workspace_id = next_node->workspace_id;
                next_movable=std::get<1>(database.EndEffectors.at(next_ee_id));
                if (ee_id==next_ee_id) continue;
                else
                {
                    found=true;
                    break;
                }
            }
        }
        if (!found)
        {
            //ee_id is the last end effector in the path
            if (!movable)
            {
                std::cout<<"ERROR, ee "<<ee_id<<" is the last ee but cannot move into the last node of the path!!"<<std::endl;
//                 break;
                return false;
            }
            else //not found, movable
            {
                if (node->workspace_id==next_workspace_id)
                {
                    std::cout<<"ERROR, the planner returned two nodes with same ee and same workspace!!"<<std::endl;
//                     break;
                    return false;
                }
                else
                {
                    //not found, movable, different workspaces
                    compute_centroid(centroid_x,centroid_y,next_workspace_id);
                    centroid_z=HIGH;
                    std::cout<<"centroid: "<<centroid_x<<" "<<centroid_y<<" "<<centroid_z<<std::endl;
                    cartesian_command temp;
                    temp.seq_num = 1;
                    temp.cartesian_task.position.x=centroid_x;
                    temp.cartesian_task.position.y=centroid_y;
                    temp.cartesian_task.position.z=centroid_z;
                    temp.command=cartesian_commands::MOVE;
                    result.push_back(std::make_pair(ee_id,temp));
                    break;
                }
            }
        }
        else //found, ee_id is not the last ee in the path
        {
            if (!movable && !next_movable)
            {
                std::cout<<"ERROR, the planner returned two nodes with not movable different ees!!"<<std::endl;
//                 break;
                return false;
            }
            compute_centroid(centroid_x,centroid_y,next_workspace_id);
            if (movable && next_movable) //both ee are movable: change above ground
                centroid_z=HIGH;
            else //one is movable, change on ground
                centroid_z=0;
            std::cout<<"centroid: "<<centroid_x<<" "<<centroid_y<<" "<<centroid_z<<std::endl;
            
            cartesian_command temp;
            temp.cartesian_task.position.x=centroid_x;
            temp.cartesian_task.position.y=centroid_y;
            temp.cartesian_task.position.z=centroid_z;
            temp.seq_num = 1;
            temp.command=cartesian_commands::MOVE;
            if (movable) result.push_back(std::make_pair(ee_id,temp)); //move the first
            temp.seq_num = 0;
            if (next_movable) result.push_back(std::make_pair(next_ee_id,temp)); //move the next
            node=next_node;
            continue;
        }
        
    }
    //move the second e.e (if the first is not movable) in the source position (got from getting_info)
    auto initial_node=path.front();
    auto ee_id = std::get<1>(database.Grasps.at(initial_node.grasp_id));
    bool movable=std::get<1>(database.EndEffectors.at(ee_id));
    if (!movable)
    {
        result.front().second.cartesian_task=data.source_position;
    }
    
    //move the last e.e in the final position (got from getting_info)
    result.back().second.cartesian_task=data.target_position;
    return true;
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
        abort();//TODO: getResults should return a failed planning, and go back into steady
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
    if (srv.response.path.size()<2)
    {
        ROS_ERROR("The planner returned a path with less than 2 nodes");
        abort();//TODO: getResults should return a failed planning, and go back into steady
    }
    if (srv.response.path.size()<3)
    {
        ROS_INFO("The planner returned a path with less than 3 nodes, should we handle this in a different way??");
        abort();//TODO: getResults should return a failed planning, and go back into steady
    }
    std::vector<std::pair<endeffector_id,cartesian_command>> result;
    bool converted=semantic_to_cartesian(result,srv.response.path);
    if (!converted)
    {
        std::cout<<"Error converting semantic to cartesian!"<<std::endl;
    }
    //TODO go through the results and insert open/close grasps
    
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