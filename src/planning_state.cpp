#include "planning_state.h"
#include "tf/tf.h"

planning_state::planning_state(shared_memory& data):data_(data)
{
    if( !ros::isInitialized() )
    {
        int argc;
	char** argv;
	ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }

    client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    plan_executed = false;
    seq=0;
    initialize = false;
}

std::map< transition, bool > planning_state::getResults()
{
    std::map< transition, bool > results;
    results[transition::planning_done]=plan_executed;
    plan_executed = false;
    return results;
}

void planning_state::fake_plan()
{
    geometry_msgs::Pose r_ee_pose;
    geometry_msgs::Pose l_ee_pose;

    data_.get_object_pose(r_ee_pose);

    data_.get_object_pose(l_ee_pose);
    double roll = -1.565;
    double pitch = 0.0;
    double yaw = 0.336;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,l_ee_pose.orientation);

    std::map<std::string, geometry_msgs::Pose> poses;

    poses["right_hand"] = r_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    r_ee_pose.position.y-=0.1;
    poses["right_hand"] = r_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    l_ee_pose.position.z+=0.2;
    l_ee_pose.position.y-=0.15;
    poses["left_hand"] = l_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    r_ee_pose.position.y+=0.1;
    poses["right_hand"] = r_ee_pose;
    data_.cartesian_plan.push_back(poses);

    poses.clear();
    l_ee_pose.position.y-=0.1;
    poses["left_hand"] = l_ee_pose;
    data_.cartesian_plan.push_back(poses);
}

void planning_state::print_plan()
{
    int i=0;
    for(auto item:data_.cartesian_plan)
    {
	for(auto subitem:item)
	{
	    ROS_INFO_STREAM(i<<") "<<subitem.first<<" [ p.x: "<< subitem.second.position.x<<" p.y: "<< subitem.second.position.y<<" p.z: "<< subitem.second.position.z<<
	    " o.x: "<< subitem.second.orientation.x<<" o.y: "<< subitem.second.orientation.y<<" o.z: "<< subitem.second.orientation.z<<" o.w: "<< subitem.second.orientation.w<<" ]"<<std::endl);
	}
	i++;
    }
}

void planning_state::run()
{
    //TODO: call the planner, not the ik_control

    if(plan_executed) return;

    if(!initialize)
    {
	fake_plan();
	print_plan();
	seq=0;
	initialize = true;
    }

    if(seq>=data_.cartesian_plan.size())
    {
	initialize = false;
        return;
    }

    geometry_msgs::Pose ee_pose;

    srv.request.ee_pose.clear();

    for(auto item:data_.cartesian_plan.at(seq))
    {
	ee_pose=item.second;

	srv.request.command = "plan";
	srv.request.ee_name = item.first;
	srv.request.time = 2;
	srv.request.ee_pose.push_back(ee_pose);

	if (client.call(srv))
	{
	    ROS_INFO_STREAM("IK Request accepted: (" << (int)srv.response.ack << ") - seq: "<<seq);
	    plan_executed = true;
	}
	else
	{
	    ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
	}
    }

    seq++;
}

bool planning_state::isComplete()
{
    return plan_executed;
}

std::string planning_state::get_type()
{
    return "planning_state";
}