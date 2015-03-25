#include "../include/semantic_planning_state.h"
#include <dual_manipulation_shared/geometry_tools.h>
#include <ros/init.h>
#include <dual_manipulation_shared/stream_utils.h>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/String.h>

bool left_ik,right_ik, left_ik_ok, right_ik_ok;

void plan_callback_l(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Left IK Plan : %s",str->data.c_str());
    left_ik=true;
    left_ik_ok = strcmp(str->data.c_str(),"error") != 0;
}

void plan_callback_r(const std_msgs::String::ConstPtr& str)
{
    ROS_INFO("Right IK Plan : %s",str->data.c_str());
    right_ik=true;
    right_ik_ok = strcmp(str->data.c_str(),"error") != 0;
}

semantic_planning_state::semantic_planning_state(shared_memory& data):data(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
        char** argv;
        ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
    completed=false;
    
//     fine_tuning[3]=KDL::Frame(KDL::Rotation::RotZ(M_PI/2.0));
//     fine_tuning[6]=KDL::Frame(KDL::Vector(0,0,0.0));
//     fine_tuning[7]=KDL::Frame(KDL::Rotation::RotY(M_PI/18.0),KDL::Vector(0,0,-0.17));
}

std::map< transition, bool > semantic_planning_state::getResults()
{
    return internal_state;
}


bool semantic_planning_state::check_ik(endeffector_id ee_id, KDL::Frame World_FirstEE, endeffector_id next_ee_id, KDL::Frame World_SecondEE)
{
    // assume at first everything went smoothly - TODO: something better
    left_ik = true;
    right_ik = true;
    left_ik_ok = true;
    right_ik_ok = true;
    
    if(std::get<1>(database.EndEffectors.at(ee_id)))
      inverse_kinematics(std::get<0>(database.EndEffectors[ee_id]),World_FirstEE);
    if(std::get<1>(database.EndEffectors.at(next_ee_id)))
      inverse_kinematics(std::get<0>(database.EndEffectors[next_ee_id]),World_SecondEE);
    bool done=false;
    while (!done)
    {
        ros::spinOnce();
        usleep(200000);
        if (left_ik && right_ik) done=left_ik_ok && right_ik_ok;
    }
    return done;
}

bool semantic_planning_state::inverse_kinematics(std::string ee_name, KDL::Frame cartesian)
{
    static ros::NodeHandle n;
    static ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    static ros::Subscriber plan_lsub = n.subscribe("/ik_control/left_hand/check_done",0,plan_callback_l);
    static ros::Subscriber plan_rsub = n.subscribe("/ik_control/right_hand/check_done",0,plan_callback_r);
    dual_manipulation_shared::ik_service srv;
    
    geometry_msgs::Pose ee_pose;
    tf::poseKDLToMsg(cartesian,ee_pose);
    srv.request.command = "ik_check";
    srv.request.ee_pose.clear();
    srv.request.ee_pose.push_back(ee_pose);
    srv.request.ee_name = ee_name;
    
    if (client.call(srv))
    {
        ROS_INFO("IK Request accepted: %d", (int)srv.response.ack);
    }
    else
    {
        ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
        return false;
    }
    return true;
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
//         geom.order_points(temp);
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
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
        
        abort();//TODO: getResults should return a failed planning, and go back into steady
    }

    ros::spinOnce();

    // convert semantic plan into cartesian vector
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
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
        
        abort();//TODO: getResults should return a failed planning, and go back into steady
    }
    if (srv.response.path.size()<3)
    {
        ROS_INFO("The planner returned a path with less than 3 nodes, should we handle this in a different way??");
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
        
        abort();//TODO: getResults should return a failed planning, and go back into steady
    }
    std::vector<std::pair<endeffector_id,cartesian_command>> result;
    bool converted=semantic_to_cartesian(result,srv.response.path);
    if (!converted)
    {
        std::cout<<"Error converting semantic to cartesian!"<<std::endl;
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
        
    }
    std::cout << "=== Cartesian plan print-out ===" << std::endl;
    std::cout << "( Note that grasp/ungrasp poses are the object poses, not the end-effector ones )" << std::endl;
    data.cartesian_plan = result;
    for (auto i:result)
        std::cout<<i<<std::endl;
    std::cout << "=== end of cartesian plan print-out ===" << std::endl;
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