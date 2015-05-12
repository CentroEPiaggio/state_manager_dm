#include "../include/ik_control_state.h"
#include "../include/ik_planning_substate.h"
#include "../include/ik_moving_substate.h"
#include "../include/ik_grasping_substate.h"
#include "../include/ik_checking_grasp_substate.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <visualization_msgs/MarkerArray.h>
#include <dual_manipulation_shared/serialization_utils.h>
#include <dual_manipulation_shared/grasp_trajectory.h>
#include <tf_conversions/tf_kdl.h>

#define OBJ_GRASP_FACTOR 1000
#define ALLOW_REPLANNING 0

#if ALLOW_REPLANNING
#include "../include/ik_need_semantic_replan.h"
#endif

ik_control_state::ik_control_state(shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc=0;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    // fake_plan();
    print_plan();
    subdata.cartesian_plan = &data.cartesian_plan;
    subdata.next_plan=0;
    subdata.obj_id = &data.obj_id;
    subdata.object_name = &data.object_name;
    subdata.robot_moving.store(false);
    subdata.move_failed.store(false);

    auto ik_planning = new ik_planning_substate(subdata);
    auto ik_checking_grasp = new ik_checking_grasp_substate(subdata);
    auto ik_moving = new ik_moving_substate(subdata);
    waiting = new ik_steady_substate(subdata);
    auto exiting = new ik_exiting_substate(subdata);
    auto failing = new ik_failing_substate(subdata);
#if ALLOW_REPLANNING
    auto ik_need_replan = new ik_need_semantic_replan(subdata,data);
#endif
    
    std::vector<std::tuple<abstract_state<ik_transition>*,ik_transition_type,abstract_state<ik_transition>*>> transition_table{
        //------initial state---------------+--------- command ---------------------------------------+-- final state------ +
        std::make_tuple( waiting            , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::move,true)                ,   ik_moving         ),
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::check_grasp,true)         ,   ik_checking_grasp ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::done,true)                ,   exiting           ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
//         std::make_tuple( ik_grasping        , std::make_pair(ik_transition::done,true)                ,   exiting           ),
//         std::make_tuple( ik_grasping        , std::make_pair(ik_transition::checkgrasp,true)          ,   ik_checking_grasp ),
// 	std::make_tuple( ik_grasping        , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
// 	//----------------------------------+---------------------------------------------------------+-------------------- +
	std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::check_done,true)          ,   ik_moving         ),
	std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::soft_fail,true)           ,   ik_moving         ),
	std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
	//----------------------------------+---------------------------------------------------------+-------------------- +
#if ALLOW_REPLANNING
	std::make_tuple( ik_need_replan     , std::make_pair(ik_transition::need_replan,true)         ,   exiting           ),
#endif
	//----------------------------------+---------------------------------------------------------+-------------------- +
	std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::fail,true)                ,   failing           ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::fail,true)                ,   failing           ),
#if ALLOW_REPLANNING
	std::make_tuple( ik_planning        , std::make_pair(ik_transition::fail,true)                ,   ik_need_replan    ),
	std::make_tuple( ik_need_replan     , std::make_pair(ik_transition::fail,true)                ,   failing           ),
#else
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::fail,true)                ,   failing           ),
#endif
    };

    sm.insert(transition_table);
    
    result = false;
    complete = false;
    new_plan = true;
    show_plan = true;
    need_replan = false;
    current_state=waiting;

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    planned_path_publisher_ = n.advertise<visualization_msgs::MarkerArray>("cartesian_converted_semantic_path", 1000, true );
}

std::map< transition, bool > ik_control_state::getResults()
{
    std::map< transition, bool > results;
    result =  (subdata.next_plan == subdata.cartesian_plan->size());
    if (current_state->get_type()=="ik_failing_substate") results[transition::abort_move]=true;
    else results[transition::task_accomplished]=result;
    return results;
}

void ik_control_state::reset()
{
    srv.request.command = "free_all";
    if(client.call(srv))
    {
	ROS_INFO_STREAM("IK FREE_ALL Request accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }

    subdata.next_plan=0;
    subdata.robot_moving.store(false);
    subdata.move_failed.store(false);
    complete=false;
    new_plan = true;
    need_replan = false;
    current_state=waiting;
    current_state->reset();
}

void ik_control_state::run()
{
    if(current_state->get_type()=="ik_exiting_substate" || current_state->get_type()=="ik_failing_substate")
    {
	complete = true;
	return;
    }
    else if(current_state->get_type()=="ik_need_semantic_replan")
    {
	need_replan = true;
    }

    if(new_plan || show_plan)
    {
      new_plan = false;
      show_plan = false;
      show_plan_with_markers();
    }
    
    current_state->run();
    
    if (current_state->isComplete())
    {
	ROS_INFO_STREAM("current_state " << current_state->get_type() << " is complete!");
	auto temp_map = current_state->getResults();
	for (auto temp:temp_map)
	    transition_map[temp.first]=temp.second;
    }
    for (auto trigger: transition_map)
    {
	auto temp_state = sm.evolve_state_machine(current_state, trigger);
	if (temp_state!=current_state)
	{
	    current_state=temp_state;
            current_state->reset();
	    std::cout<<"- new substate type: "<<current_state->get_type()<<std::endl;
	    // std::cout << "press 'y' key and enter to proceed" << std::endl;
	    // char tmp = 'n';
	    // while (tmp!='y')
	    // {
	    //   std::cin >> tmp;
	    //   usleep(200000);
	    // }
	    usleep(5000);
	    transition_map.clear();
	    break;
	}
    }
}

bool ik_control_state::isComplete()
{
    return complete;
}

std::string ik_control_state::get_type()
{
    return "ik_control_state";
}

void ik_control_state::print_plan()
{
    int i=0;
    for(auto item:data_.cartesian_plan)
    {
	auto subitem=item;
	{
            ROS_INFO_STREAM(i<<") "<<subitem.first<<" [ p.x: "<< subitem.second.cartesian_task.position.x<<" p.y: "<< subitem.second.cartesian_task.position.y<<" p.z: "<< subitem.second.cartesian_task.position.z<<
            " o.x: "<< subitem.second.cartesian_task.orientation.x<<" o.y: "<< subitem.second.cartesian_task.orientation.y<<" o.z: "<< subitem.second.cartesian_task.orientation.z<<" o.w: "<< subitem.second.cartesian_task.orientation.w<<" ]"<<std::endl);
	}
	i++;
    }
}

void poseCallback(const cartesian_command& msg, std::string prefix)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setIdentity();
  const geometry_msgs::Quaternion& q = msg.cartesian_task.orientation;

  // check for malformed poses
  if(!(std::norm(q.x)+std::norm(q.y)+std::norm(q.z)+std::norm(q.w) < 0.001))
    tf::poseMsgToTF(msg.cartesian_task,transform);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", prefix + " | command:" + std::to_string((int)msg.command)));
}

void ik_control_state::show_plan_with_tf()
{
    int i=0;
    for (auto item:(*subdata.cartesian_plan))
    {
      poseCallback(item.second, std::to_string(i++) + "ee_id:" + std::to_string((int)item.first));
    }
}

void ik_control_state::show_plan_with_markers()
{
  std::string file_name;
  int obj_id = (int)(*subdata.obj_id);
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markers;
  std::string path_r = "package://soft_hand_description/meshes/palm_right.stl";
  std::string path_l = "package://soft_hand_description/meshes/palm_left.stl";
  std::string path_obj = "package://asus_scanner_models/" + std::get<1>(db_mapper.Objects.at( obj_id ));

  marker.action=3; //delete all
  marker.header.frame_id = "world";
  markers.markers.push_back(marker);

  marker.action=visualization_msgs::Marker::ADD;
  marker.lifetime=ros::DURATION_MAX;
  marker.type=visualization_msgs::Marker::MESH_RESOURCE;
  
  int marker_id = 0;
  for(auto item:(*subdata.cartesian_plan))
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
