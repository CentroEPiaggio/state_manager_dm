#include "../include/semantic_planning_state.h"
#include "dual_manipulation_planner/planner_lib.h"
#include <ros/init.h>
#include <dual_manipulation_shared/stream_utils.h>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include "dual_manipulation_shared/good_grasp_msg.h"
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <dual_manipulation_shared/grasp_trajectory.h>

#define OBJ_GRASP_FACTOR 1000
#define GRASP_TESTING 0
#define DEBUG 0
#define CLASS_NAMESPACE "semantic_planning_state::"
#define CLASS_LOGNAME "semantic_planning_state"

semantic_planning_state::semantic_planning_state(shared_memory& data):data(data),database(data.db_mapper),converter(data.db_mapper)
{
    if( !ros::isInitialized() )
    {
        int argc;
        char** argv;
        ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    client = n.serviceClient<dual_manipulation_shared::planner_service>("planner_ros_service");
    good_grasps_pub = n.advertise<dual_manipulation_shared::good_grasp_msg>( "/good_grasp_topic", 1000 );
    completed=false;
    planned_path_publisher_ = n.advertise<visualization_msgs::MarkerArray>("cartesian_converted_semantic_path", 1000, true );
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
    
    KDL::Frame tmp;
    tf::poseMsgToKDL(data.source_position,tmp);
    source = database.getWorkspaceIDFromPose(tmp);
    tf::poseMsgToKDL(data.target_position,tmp);
    target = database.getWorkspaceIDFromPose(tmp);
    source_found = source != -1;
    target_found = target != -1;
    
    if (!source_found || !target_found)
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : " << (source_found?"":"source ") << (source_found | target_found ?"":"AND ") << (target_found?"":"target ") << "position" << (source_found | target_found?" is":"s are") << " outside the workspaces!!!");
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
    }
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : source position is in workspace " << source);
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : target position is in workspace " << target);
    //Filtering source and target grasps in order to reduce the graph size during planning
    KDL::Frame fake;
    dual_manipulation_shared::good_grasp_msg msg;
    
    node_info temp;
    temp.current_ee_id = std::get<1>(database.Grasps.at(data.source_grasp));
    temp.current_grasp_id = data.source_grasp;
    temp.current_workspace_id = source;
    temp.next_workspace_id = source;
    temp.type=node_properties::GRASP;
    if(database.Grasp_transitions.empty())
    {
        std::cerr << "There are no transitions in the loaded database, plan failed" << std::endl;
        internal_state.insert(std::make_pair(transition::failed_plan,true));
        completed=true;
        return;
    }
    
    //TODO: check grasps only if source is a table grasp
    bool source_ee_movable = std::get<1>(database.EndEffectors.at(temp.current_ee_id));
    if(source_ee_movable)
    {
        //DO NOT CHECK GRASPS!!!
        //NOTE: this could be used if going to planning directly from execution, when we have a grasp but we cannot perform the passing, to replan!
        //how to test? having ik_control state wait for us, and changing the transition grasp with an unfeasible one!
    }
    else
    {
        std::cout << "source: #database.Grasp_transitions.at(" << data.source_grasp << ")=" << database.Grasp_transitions.at(data.source_grasp).size() << " :" << std::endl;
        for (auto next_grasp_id :database.Grasp_transitions.at(data.source_grasp))
        {
#if DEBUG>0
            std::cout << " | " << next_grasp_id;
#endif
            temp.next_grasp_id = next_grasp_id;
            temp.next_ee_id = std::get<1>(database.Grasps.at(next_grasp_id));
            if (database.Reachability.at(temp.next_ee_id).count(source))
            {
                if(converter.checkSingleGrasp(fake, temp, data, true, false, data.filtered_source_nodes, data.filtered_target_nodes))
                    msg.good_source_grasps.push_back(temp.next_grasp_id);
                else
                {
                    msg.bad_source_grasps.push_back(temp.next_grasp_id);
                    data.planner.add_filtered_arc(data.source_grasp,source,next_grasp_id,source);
                }
            }
        }
#if DEBUG>0
        std::cout << " | " << std::endl;
#endif
    }
    
    temp.next_ee_id = std::get<1>(database.Grasps.at(data.target_grasp));
    temp.next_grasp_id = data.target_grasp;
    temp.current_workspace_id = target;
    temp.next_workspace_id = target;
    temp.type=node_properties::UNGRASP;
    
    //TODO: check grasps only if target is a table grasp
    bool target_ee_movable = std::get<1>(database.EndEffectors.at(temp.next_ee_id));
    if(target_ee_movable)
    {
        //DO NOT CHECK GRASPS!!!
    }
    else
    {
        std::cout << "target: #database.Grasp_transitions.at(" << data.target_grasp << ")=" << database.Grasp_transitions.at(data.target_grasp).size() << " :" << std::endl;
        for (auto current_grasp_id :database.Grasp_transitions.at(data.target_grasp))
        {
#if DEBUG>0
            std::cout << " | " << current_grasp_id;
#endif
            temp.current_grasp_id = current_grasp_id;
            temp.current_ee_id = std::get<1>(database.Grasps.at(current_grasp_id));
            if (database.Reachability.at(temp.current_ee_id).count(target))
            {
                if(converter.checkSingleGrasp(fake, temp, data, false, true, data.filtered_source_nodes, data.filtered_target_nodes))
                    msg.good_target_grasps.push_back(temp.current_grasp_id);
                else
                {
                    msg.bad_target_grasps.push_back(temp.current_grasp_id);
                    data.planner.add_filtered_arc(current_grasp_id,target,data.target_grasp,target);
                }
            }
        }
#if DEBUG>0
        std::cout << " | " << std::endl;
#endif
    }
    
    good_grasps_pub.publish(msg);
    
    int max_counter=1000;
    while(max_counter>0)
    {
        max_counter--;
        //Call planner with the right parameters
        srv.request.command="plan";
        srv.request.source.grasp_id=data.source_grasp;
        srv.request.source.workspace_id=source;
        srv.request.destination.grasp_id=data.target_grasp;
        srv.request.destination.workspace_id=target;
        srv.response.path.clear();
        #if GRASP_TESTING
            if (client.call(srv))
            {
                ROS_INFO("Planning Request accepted, response: %d", (int)srv.response.ack);
                if (srv.response.ack)
                {
                    //for (auto node:srv.response.path)
                       //std::cout<<node.grasp_id<<" "<<node.workspace_id<<std::endl;
                }
                else
                {
                    ROS_ERROR("Failed to plan, reason: %s",srv.response.status.c_str());
                }
            }
        #else
        if (data.planner.plan(data.source_grasp,source,data.target_grasp,target,srv.response.path))
        {
            {
                data.planner.draw_path();
            }
        }
        #endif
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
        std::vector<std::pair<endeffector_id,cartesian_command>> result;
        bool converted=converter.convert(result,srv.response.path,data,data.filtered_source_nodes,data.filtered_target_nodes);
        #if GRASP_TESTING
        if(result.empty())
        {
            ROS_ERROR_STREAM("The conversion resulted in an empty path!");
            max_counter = -1;
        }
        #else
        if (!converted)
        {
            data.planner.add_filtered_arc(data.filtered_source_nodes.grasp_id,data.filtered_source_nodes.workspace_id,data.filtered_target_nodes.grasp_id, data.filtered_target_nodes.workspace_id);
            if(max_counter > 0)
                ROS_WARN_STREAM("Error converting semantic to cartesian! I will try again for " << max_counter << " times");
            else
            {
                max_counter = -1;
                ROS_ERROR_STREAM("Error converting semantic to cartesian! Maximum number of attempts reached!");
            }
            continue;
        }
        #endif
        std::cout << "=== Semantic and Cartesian plans print-out ===" << std::endl;
        std::cout << "( Note that grasp/ungrasp poses are the object poses, not the end-effector ones )" << std::endl;
        data.cartesian_plan = result;
        for (auto node:srv.response.path)
            std::cout<<node.grasp_id<<" "<<node.workspace_id<<std::endl;
        for (auto i:result)
            std::cout<<i<<std::endl;
        std::cout << "=== end of plans print-out ===" << std::endl;
        
        break;
    }
    if(max_counter >= 0)
    {
        internal_state.insert(std::make_pair(transition::good_plan,true));
        show_plan_with_markers();
    }
    else
        internal_state.insert(std::make_pair(transition::failed_plan,true));
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

void semantic_planning_state::reset()
{
    //   data.filtered_source_nodes.clear();
    //   data.filtered_target_nodes.clear();
    data.cartesian_plan.clear();
}

void semantic_planning_state::show_plan_with_markers()
{
    std::string file_name;
    int obj_id = (int)(data.obj_id);
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markers;
    std::string path_r = "package://soft_hand_description/meshes/palm_right.stl";
    std::string path_l = "package://soft_hand_description/meshes/palm_left.stl";
    std::string path_obj = std::get<1>(database.Objects.at( obj_id ));
    
    marker.action=3; //delete all
    marker.header.frame_id = "world";
    markers.markers.push_back(marker);
    
    marker.action=visualization_msgs::Marker::ADD;
    marker.lifetime=ros::DURATION_MAX;
    marker.type=visualization_msgs::Marker::MESH_RESOURCE;
    
    int marker_id = 0;
    for(auto item:(data.cartesian_plan))
    {
        if(item.second.command != cartesian_commands::GRASP && item.second.command != cartesian_commands::UNGRASP)
            continue;
        
        int grasp_id = item.second.ee_grasp_id;
        int ee_id = item.first;
        dual_manipulation_shared::grasp_trajectory grasp_msg;
        file_name = "object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id % OBJ_GRASP_FACTOR);
        if(!deserialize_ik(grasp_msg,file_name))
        {
            ROS_WARN_STREAM("Error in deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "! . . . Retry!");
            continue;
        }
        // object
        marker.scale.x=1;
        marker.scale.y=1;
        marker.scale.z=1;
        // I'll use the same ID in different namespaces
        marker.id = marker_id++;
        
        marker.color.a = 1;
        marker.color.b = 1;
        marker.color.g = 0;
        marker.color.r = 1;
        marker.pose = item.second.cartesian_task;
        marker.mesh_resource = path_obj.c_str();
        marker.ns = "objects";
        markers.markers.push_back(marker);
        
        // hand marker should be scaled
        marker.scale.x=0.001;
        marker.scale.y=0.001;
        marker.scale.z=0.001;
        KDL::Frame Obj_EEGrasp,Obj_EEPostGrasp,World_Object;
        tf::poseMsgToKDL(item.second.cartesian_task,World_Object);
        tf::poseMsgToKDL(grasp_msg.attObject.object.mesh_poses.front(),Obj_EEPostGrasp);
        Obj_EEPostGrasp = Obj_EEPostGrasp.Inverse();
        tf::poseMsgToKDL(grasp_msg.ee_pose.back(),Obj_EEGrasp);
        
        if(item.second.command == cartesian_commands::GRASP)
        {
            marker.color.a = 1;
            marker.color.b = (ee_id==1)?0:1;
            marker.color.g = (ee_id==1)?1:0;
            marker.color.r = 0;
            tf::poseKDLToMsg(World_Object*Obj_EEGrasp,marker.pose);
            marker.mesh_resource = (ee_id==1)?(path_l.c_str()):(path_r.c_str());
            marker.ns = "grasping_ee";
        }
        else if(item.second.command == cartesian_commands::UNGRASP)
        {
            marker.color.a = 1;
            marker.color.b = (ee_id==1)?0:1;
            marker.color.g = (ee_id==1)?1:0;
            marker.color.r = 0;
            tf::poseKDLToMsg(World_Object*Obj_EEPostGrasp,marker.pose);
            marker.mesh_resource = (ee_id==1)?(path_l.c_str()):(path_r.c_str());
            marker.ns = "ungrasping_ee";
        }
        markers.markers.push_back(marker);
    }
    
    planned_path_publisher_.publish(markers);
}
