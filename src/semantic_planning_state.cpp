#include "../include/semantic_planning_state.h"
#include <dual_manipulation_shared/geometry_tools.h>
#include <ros/init.h>
#include <dual_manipulation_shared/stream_utils.h>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/String.h>
#define HIGH 0.5

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
}

std::map< transition, bool > semantic_planning_state::getResults()
{
    return internal_state;
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

bool semantic_planning_state::getPreGraspMatrix(object_id object,grasp_id grasp, KDL::Frame & Object_EE)
{
    dual_manipulation_shared::ik_service srv;
    bool ok = deserialize_ik(srv.request,"object" + std::to_string(object) + "/grasp" + std::to_string(grasp));
    if (ok)
        tf::poseMsgToKDL(srv.request.ee_pose.front(),Object_EE);
    return ok;
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

bool semantic_planning_state::compute_intergrasp_orientation(KDL::Vector World_centroid, KDL::Frame& World_Object, 
                                                             endeffector_id ee_id, endeffector_id next_ee_id, grasp_id grasp, 
                                                             grasp_id next_grasp, object_id object,bool movable,bool next_movable)
{
    bool found=false;
    if (movable && next_movable)
    {
        //Case both movable (centroid_z should be HIGH)
        KDL::Frame World_Centroid(World_centroid);
        KDL::Frame Object_FirstEE, Object_SecondEE;
        bool ok=getPreGraspMatrix(object,grasp,Object_FirstEE);
        if (!ok) 
        {
            std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<ee_id<<std::endl;
        }
        ok = getPreGraspMatrix(object,next_grasp,Object_SecondEE);
        if (!ok) 
        {
            std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<next_ee_id<<std::endl;
        }
        
    //     Object_FirstEE=database.Grasps_sequence[grasp][0].first;
    //     Object_SecondEE=database.Grasps_sequence[next_grasp][0].first.Inverse();
        KDL::Frame Object_MiddlePosition;
        Object_MiddlePosition.p=(Object_FirstEE.p+Object_SecondEE.p)/2;
        KDL::Vector Object_Xmiddle=Object_SecondEE.p-Object_FirstEE.p;
        Object_Xmiddle.Normalize();
        double a=Object_Xmiddle.x();
        double b=Object_Xmiddle.y();
        double c=Object_Xmiddle.z();
        KDL::Vector Object_Zmiddle(a*c,b*c,1-c*c); //(I - X^T * X ) * (0 0 1)^T
        /*
        a b c   a b c
        1-aa  ab  ac  0    ac
        ba  1-bb  bc  0 =  bc
        ca  cb  1-cc  1    1-cc
        */
        Object_Zmiddle.Normalize();
        KDL::Vector Object_Ymiddle=Object_Zmiddle*Object_Xmiddle;
        Object_Ymiddle.Normalize();//Just in case
        Object_MiddlePosition.M=KDL::Rotation(Object_Xmiddle,Object_Ymiddle,Object_Zmiddle);
        for (double anglez=0;anglez<M_PI;anglez=anglez+0.1)
        {
            KDL::Frame Object_MiddlePositionRotatedz1=Object_MiddlePosition*KDL::Frame(KDL::Rotation::Rot(Object_Zmiddle,anglez));
            //iterate along rotation until a solution if found
            for (double anglex=0;anglex<M_PI;anglex=anglex+0.1)
            {
                //find back the grasps
                KDL::Frame Object_MiddlePositionRotatedx1=Object_MiddlePositionRotatedz1*KDL::Frame(KDL::Rotation::Rot(Object_Xmiddle,anglex));
                World_Object = World_Centroid*Object_MiddlePositionRotatedx1.Inverse();
                found = check_ik(ee_id,World_Object*Object_FirstEE,next_ee_id,World_Object*Object_SecondEE);
                if (found) break;
                KDL::Frame Object_MiddlePositionRotatedx2=Object_MiddlePositionRotatedz1*KDL::Frame(KDL::Rotation::Rot(Object_Xmiddle,-anglex));
                World_Object = World_Centroid*Object_MiddlePositionRotatedx2.Inverse();
                found = check_ik(ee_id,World_Object*Object_FirstEE,next_ee_id,World_Object*Object_SecondEE);
                if (found) break;
            }
            if (found) break;
            if (anglez==0) continue;
            KDL::Frame Object_MiddlePositionRotatedz2=Object_MiddlePosition*KDL::Frame(KDL::Rotation::Rot(Object_Zmiddle,-anglez));
            //iterate along rotation until a solution if found
            for (double anglex=0;anglex<M_PI;anglex=anglex+0.1)
            {
                //find back the grasps
                KDL::Frame Object_MiddlePositionRotatedx1=Object_MiddlePositionRotatedz2*KDL::Frame(KDL::Rotation::Rot(Object_Xmiddle,anglex));
                World_Object = World_Centroid*Object_MiddlePositionRotatedx1.Inverse();
                found = check_ik(ee_id,World_Object*Object_FirstEE,next_ee_id,World_Object*Object_SecondEE);
                if (found) break;
                KDL::Frame Object_MiddlePositionRotatedx2=Object_MiddlePositionRotatedz2*KDL::Frame(KDL::Rotation::Rot(Object_Xmiddle,-anglex));
                World_Object = World_Centroid*Object_MiddlePositionRotatedx2.Inverse();
                found = check_ik(ee_id,World_Object*Object_FirstEE,next_ee_id,World_Object*Object_SecondEE);
                if (found) break;
            }
            if (found) break;
        }
    }
    else if ((movable && !next_movable) || (!movable && next_movable))
    {
        //Case one ee movable
        KDL::Frame World_Centroid(World_centroid);
        KDL::Frame Object_FirstEE, Object_SecondEE;
	bool ok=getPreGraspMatrix(object,grasp,Object_FirstEE);
        if (!ok) 
        {
            std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<ee_id<<std::endl;
        }
        ok = getPreGraspMatrix(object,next_grasp,Object_SecondEE);
        if (!ok) 
        {
            std::cout<<"Error in getting pregrasp matrix for object "<<object<<" and ee "<<next_ee_id<<std::endl;
        }
        // from now on, first is movable and second is not movable: I don't need to keep ordering in here
	if (!movable && next_movable)
	{
	  KDL::Frame switch_frame(Object_FirstEE);
	  Object_FirstEE = Object_SecondEE;
	  Object_SecondEE = switch_frame;
	  endeffector_id switch_id = ee_id;
	  ee_id = next_ee_id;
	  next_ee_id = switch_id;
	  // grasps are not used later, but just in case
	  grasp_id switch_grasp = grasp;
	  grasp = next_grasp;
	  next_grasp = switch_grasp;
	}
        
        // This will place the secondEE (not movable) exactly on the centroid (a table is on the centroid of the workspace)
        KDL::Frame World_Object = World_Centroid*Object_SecondEE.Inverse();
	// to use for rotations, axis aligned with world z computed in object frame
	KDL::Vector Object_worldZ(World_Object.Inverse().M*KDL::Vector(0,0,1));
        
	for (double anglez=0;anglez<M_PI;anglez=anglez+0.1)
        {
            KDL::Frame World_Object_Rotatedz1=World_Object*KDL::Frame(KDL::Rotation::Rot(Object_worldZ,anglez));
            //iterate along rotation until a solution if found
	    found = check_ik(ee_id,World_Object_Rotatedz1*Object_FirstEE,next_ee_id,World_Object_Rotatedz1*Object_SecondEE);
	    if (found)
	    {
	      World_Object = World_Object_Rotatedz1;
	      break;
	    }
            if (anglez==0) continue;
            KDL::Frame World_Object_Rotatedz2=World_Object*KDL::Frame(KDL::Rotation::Rot(Object_worldZ,-anglez));
	    found = check_ik(ee_id,World_Object_Rotatedz2*Object_FirstEE,next_ee_id,World_Object_Rotatedz2*Object_SecondEE);
            if (found)
	    {
	      World_Object = World_Object_Rotatedz2;
	      break;
	    }
        }

        // at the end, switch frames if necessary
	if (!movable && next_movable)
	{
	  KDL::Frame switch_frame(Object_FirstEE);
	  Object_FirstEE = Object_SecondEE;
	  Object_SecondEE = switch_frame;
	}
    }
    else if (!movable && !next_movable)
    {
        //error 4!!
        std::cout<<"error! both E.E. are not movable"<<std::endl;
        return false;
    }
    return found;
}


bool semantic_planning_state::semantic_to_cartesian(std::vector<std::pair<endeffector_id,cartesian_command>>& result,const dual_manipulation_shared::planner_serviceResponse::_path_type& path)
{
// 1) Getting preliminary information
    std::map<endeffector_id,bool> ee_grasped;
    result.clear();
    auto ee_id = std::get<1>(database.Grasps.at(path.front().grasp_id));
    auto ee_name=std::get<0>(database.EndEffectors.at(ee_id));
    bool movable=std::get<1>(database.EndEffectors.at(ee_id));
//--------------------------------------

// 2) Setting a boolean flag for each end effector in order to keep track if they are grasping something or not

    //This approach is valid only if no more than one e.e. can grasp an object at the same time!! 
    ee_grasped[ee_id]=true;
    std::cout<<"Assuming that only "<<ee_name<<" is grasping the object, and no other e.e. is grasping anything!"<<std::endl;
    for (auto node=path.begin();node!=path.end();++node)
    {
        auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
        if (!ee_grasped.count(ee_id))
            ee_grasped[ee_id]=false;
    }
//-------------------------------------------

// 3) Start of the main conversion loop
    for (auto node=path.begin();node!=path.end();)//++node)
    {
        // 3.1) Getting preliminary info for the current node
        auto ee_id = std::get<1>(database.Grasps.at(node->grasp_id));
        auto ee_name=std::get<0>(database.EndEffectors.at(ee_id));
        double centroid_x=0, centroid_y=0, centroid_z=0;
        geometry_msgs::Quaternion centroid_orientation;
        bool movable=std::get<1>(database.EndEffectors.at(ee_id));
        //---------------------------
        
        // 3.2) Is this the final_node?
        auto final_node=node;
        final_node++;
        if (final_node==path.end())
        {
            break; //This break jumps to 4)
        }
        //-------------------------

        //From now on node is not the last in the path

        // 3.3) Searching for the next node with a different end effector than the current one
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
                    break; //this break jumps to 3.4)
                }
            }
        }
        //-----------------------
        
        // 3.4) Beginning of real actions, depending on the result of 3.3
        if (!found)
        {
            //3.4.1) if not found, than ee_id is the last end effector in the path
            if (!movable) //not found not movable
            {
                //Error1
                std::cout<<"ERROR, ee "<<ee_id<<" is the last ee but cannot move into the last node of the path!!"<<std::endl;
                return false;
            }
            else //not found, movable
            {
                if (node->workspace_id==next_workspace_id)
                {
                    //Error2
                    std::cout<<"ERROR, the planner returned two nodes with same ee and same workspace!!"<<std::endl;
                    return false;
                }
                else  //not found->last e.e, movable, different workspaces
                {
                    // 3.4.2) We move the last==current end effector in the final workspace centroid, later on we will change this to the final desired position
                    compute_centroid(centroid_x,centroid_y,next_workspace_id);
                    centroid_z=HIGH;
                    cartesian_command temp;
                    temp.seq_num = 1;
                    temp.ee_grasp_id=node->grasp_id;
                    temp.cartesian_task.position.x=centroid_x;
                    temp.cartesian_task.position.y=centroid_y;
                    temp.cartesian_task.position.z=centroid_z;
                    temp.command=cartesian_commands::MOVE;
                    result.push_back(std::make_pair(ee_id,temp));
                    break; //This break jumps to 4)
                }
            }
        }
        else //found -> ee_id is not the last ee in the path
        {
            //3.5) There is gonna be a change of grasps
            KDL::Frame World_Object;
            if (!movable && !next_movable)
            {
                //Error 3
                std::cout<<"ERROR, the planner returned two nodes with not movable different ees!!"<<std::endl;
                return false;
            }
            //--------------

            // 3.6) compute a rough position of the place where the change of grasp will happen
            compute_centroid(centroid_x,centroid_y,next_workspace_id);
            if (movable && next_movable) //both ee are movable: change above ground
            {centroid_z=HIGH;}
            else //one is movable, change on ground
            {centroid_z=0;}
            compute_intergrasp_orientation(KDL::Vector(centroid_x,centroid_y,centroid_z),World_Object,ee_id,next_ee_id,node->grasp_id,next_node->grasp_id,data.obj_id,movable,next_movable);

            //--------------

            //3.7) Inizialize some temporary commands to be pushed back into the plan 
            cartesian_command temp, grasp, ungrasp;
            KDL::Frame Object_FirstEE, Object_SecondEE;
            temp.ee_grasp_id=node->grasp_id;
            temp.seq_num = !next_movable; //Do not parallelize movements if only the current ee is moving
            //--------------
            grasp.command=cartesian_commands::GRASP;
            grasp.seq_num = 1;
            ungrasp.command=cartesian_commands::UNGRASP;
            ungrasp.seq_num = 1;
            temp.command=cartesian_commands::MOVE;
            
            //3.8) get the pose of the first end effector with respect to the object position
            if (movable) 
            {
                bool ok=getPreGraspMatrix(data.obj_id,node->grasp_id,Object_FirstEE);
                if (!ok) 
                {
                    std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<ee_id<<std::endl;
                }
                KDL::Frame World_GraspFirstEE = World_Object*Object_FirstEE;
                tf::poseKDLToMsg(World_GraspFirstEE,temp.cartesian_task);
                result.push_back(std::make_pair(ee_id,temp)); //move the first
            }
            
            //3.9) get the pose of the second end effector with respect to the object position
            temp.seq_num = 1;
            temp.ee_grasp_id=next_node->grasp_id;
            if (next_movable)
            {
                bool ok = getPreGraspMatrix(data.obj_id,next_node->grasp_id,Object_SecondEE);
                if (!ok) 
                {
                    std::cout<<"Error in getting pregrasp matrix for object "<<data.obj_id<<" "<<data.object_name<<" and ee "<<next_ee_id<<std::endl;
                }
                KDL::Frame World_GraspSecondEE = World_Object*Object_SecondEE;
                tf::poseKDLToMsg(World_GraspSecondEE,temp.cartesian_task);
                result.push_back(std::make_pair(next_ee_id,temp)); //move the next
            }
            
            //3.10) after moving one or two end effectors, we can grasp/ungrasp depending on the state of the ee and of movable/not movable
            if (next_movable) {result.push_back(std::make_pair(next_ee_id,ee_grasped[next_ee_id]?ungrasp:grasp));ee_grasped[next_ee_id]=!ee_grasped[next_ee_id];}
            if (movable) {result.push_back(std::make_pair(ee_id,ee_grasped[ee_id]?ungrasp:grasp));ee_grasped[ee_id]=!ee_grasped[ee_id];}
            node=next_node;
            continue;
        }
        
    }
    //4) move the second e.e (if the first is not movable) in the source position (got from getting_info)
    auto initial_node=path.front();
    ee_id = std::get<1>(database.Grasps.at(initial_node.grasp_id));
    movable=std::get<1>(database.EndEffectors.at(ee_id));
    if (!movable)
    {
        result.front().second.cartesian_task=data.source_position;
    }

    //5) move the last e.e in the final position (got from getting_info)
    result.back().second.cartesian_task=data.target_position;
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
    data.cartesian_plan = result;
    for (auto i:result)
        std::cout<<i<<std::endl;
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