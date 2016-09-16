#include <ros/service_client.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <atomic>

#include <dual_manipulation_shared/state_manager_service.h>
#include <dual_manipulation_shared/gui_target_service.h>
#include <dual_manipulation_shared/gui_target_response.h>
#include <dual_manipulation_shared/ik_response.h>

#define CLASS_NAMESPACE "test_full_state_manager::"

std::atomic_bool busy;
bool received_planned_path = false;

bool gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request& req, dual_manipulation_shared::gui_target_service::Response& res)
{
    res.ack = true;
    std::cout << CLASS_NAMESPACE << __func__ << " : received gui_target_service request!" << std::endl;
    busy.store(false);
    return res.ack;
}

void planned_path_sub_fcn(const visualization_msgs::MarkerArrayConstPtr&)
{
    if(received_planned_path) return;
    
    received_planned_path = true;
    std::cout << CLASS_NAMESPACE << __func__ << " : received cartesian_converted_semantic_path!" << std::endl;
    busy.store(false);
}

void ungrasp_callback(const dual_manipulation_shared::ik_response::ConstPtr& msg)
{
    std::cout << CLASS_NAMESPACE << __func__ << " : received Ungrasp (group " << msg->group_name << " | seq: " << msg->seq << ") : " << msg->data << "!" << std::endl;
    busy.store(false);
}

void service_ret_print(const std::string& prefix, const std::string& cmd, bool res)
{
    if(!res)
    {
        ROS_ERROR_STREAM(prefix << "Service \'" << cmd << "\' rejected!");
        abort();
    }
    else
    {
        ROS_INFO_STREAM(prefix << "Service \'" << cmd << "\' accepted!");
    }
    
    while(busy.load() && ros::ok())
        usleep(5000);
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_full_state_manager "<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Test the major functionalities in dual_manipulation_state_manager. In order to do so, you have to run:"<<std::endl;
    std::cout<<"1: roslaunch vito_moveit_configuration demo.launch"<<std::endl;
    std::cout<<"2: rosrun dual_manipulation_state_manager dual_manipulation_state_manager"<<std::endl;
    std::cout<<"3: rosrun dual_manipulation_ik_control dual_manipulation_ik_control"<<std::endl;
    std::cout<<"4: rosrun dual_manipulation_state_manager test_full_state_manager"<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "test_full_state_manager");
    ros::NodeHandle node;
    ros::AsyncSpinner aspin(1);
    ros::ServiceClient client = node.serviceClient<dual_manipulation_shared::state_manager_service>("state_manager_ros_service");
    ros::ServiceServer gui_target_server = node.advertiseService<dual_manipulation_shared::gui_target_serviceRequest,dual_manipulation_shared::gui_target_serviceResponse>("gui_target_service",&gui_target_service_callback);
    ros::Publisher target_pub = node.advertise<dual_manipulation_shared::gui_target_response>("gui_target_response",1,true);
    ros::Subscriber planned_path_sub = node.subscribe<visualization_msgs::MarkerArray>("cartesian_converted_semantic_path", 1, &planned_path_sub_fcn );
    ros::Subscriber ungrasp_sub = node.subscribe<const dual_manipulation_shared::ik_response::ConstPtr&>("ik_control/ungrasp_done",1,&ungrasp_callback);
    
    busy.store(false);
    
    aspin.start();
    
    dual_manipulation_shared::state_manager_service srv;
    
    // get_info
    srv.request.command = "get_info";
    srv.request.time = ros::Time::now().toSec();
    busy.store(true);
    service_ret_print("State_manager ",srv.request.command, client.call(srv));
    
    // fill source and target poses
    geometry_msgs::Pose p_source,p_target;
    p_source.orientation.w = 1.0;
    p_source.position.x = -0.4;
    p_source.position.y = +0.4;
    p_target = p_source;
    p_target.position.y *= -1;
    
    dual_manipulation_shared::gui_target_response msg;
    msg.target_pose = p_target;
    msg.source_pose = p_source;
    msg.obj_id = 2;
    msg.name = "ContainerB";
    // busy.store(true);
    target_pub.publish(msg);
    
    usleep(500000);
    
    // "plan"
    srv.request.command = "plan";
    srv.request.time = ros::Time::now().toSec();
    busy.store(true);
    service_ret_print("State_manager ",srv.request.command, client.call(srv));
    
    usleep(500000);
    
    // "start_moving"
    srv.request.command = "start_moving";
    srv.request.time = ros::Time::now().toSec();
    busy.store(true);
    service_ret_print("State_manager ",srv.request.command, client.call(srv));
    
    // "abort_move"
    srv.request.command = "abort_move";
    srv.request.time = ros::Time::now().toSec();
    service_ret_print("State_manager ",srv.request.command, client.call(srv));
    
    usleep(500000);
    
    // "exit"
    srv.request.command = "exit";
    srv.request.time = ros::Time::now().toSec();
    service_ret_print("State_manager ",srv.request.command, client.call(srv));
    
    for(int i=0; i<3; i++)
    {
        ROS_INFO_STREAM("Finishing #" << i << "...");
        ros::spinOnce();
        usleep(500000);
    }
    
    return 0;
}
