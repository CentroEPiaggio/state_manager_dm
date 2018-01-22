#include "semantic_to_cartesian_converter.h"
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>

#define DEBUG 0 // if 1, print some more information

#define CLASS_NAMESPACE "semantic_to_cartesian_converter::"
#define CLASS_LOGNAME "semantic_to_cartesian_converter"

#define PI 3.14159

semantic_to_cartesian_converter::semantic_to_cartesian_converter(const databaseMapper& database) : database(database)
{
    s2cik.reset(new s2c_ik_converter(this->database));
    
    /// instantiate function pointers with associated transition type
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::LAST_EE_FIXED] = &semantic_to_cartesian_converter::manage_transition_last_ee_fixed;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::LAST_EE_MOVABLE] = &semantic_to_cartesian_converter::manage_transition_last_ee_movable;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::UNKNOWN] = &semantic_to_cartesian_converter::manage_transition_unknown;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::GRASP] = &semantic_to_cartesian_converter::manage_transition_grasp;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::UNGRASP] = &semantic_to_cartesian_converter::manage_transition_ungrasp;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::EXCHANGE_GRASP] = &semantic_to_cartesian_converter::manage_transition_exchange_grasp;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::MOVE_NONBLOCKING] = &semantic_to_cartesian_converter::manage_transition_move_nonblocking;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::SLIDE] = &semantic_to_cartesian_converter::manage_transition_slide;
    manage_transition_by_type[dual_manipulation::shared::NodeTransitionTypes::TILT] = &semantic_to_cartesian_converter::manage_transition_tilt;
    
    ros::NodeHandle node;
    std::vector<double> pOP(6,0), pOS(6,0); //Parameters for Object_Preslide and Object_Slide
    use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_L", pOP);
    use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_L", pOS);
    use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
    if( use_slide )
    {
        Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
        Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
    }
    else    
        std::cout << CLASS_NAMESPACE << __func__ << " Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
    
    
}

bool semantic_to_cartesian_converter::set_hand_pose_sliding(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position)
{
    ros::NodeHandle node;
    std::vector<double> pOP(6,0), pOS(6,0); //Parameters for Object_Preslide and Object_Slide

    // Here we check if the yaw angles of the two objects differ more than 45° -> if so, send warning
    KDL::Frame source_frame;
    tf::poseMsgToKDL(source_position, source_frame);
    KDL::Frame target_frame;
    tf::poseMsgToKDL(target_position, target_frame);
    double roll_s, pitch_s, yaw_s;
    double roll_t, pitch_t, yaw_t;
    source_frame.M.GetRPY(roll_s, pitch_s, yaw_s);
    target_frame.M.GetRPY(roll_t, pitch_t, yaw_t);
#if DEBUG
    if(std::abs(yaw_t - yaw_s)>= PI/4){
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " Source and Target differ in yaw more than 45°, sliding might not function correctly because of current IK limitations.");
    }
#endif
    // extracting x_s, y_s, -x_s, -y_s vectors of source object frame and the diff_v vector (difference between two object positions)
    KDL::Vector x_s = source_frame.M.UnitX();
    KDL::Vector _x_s = x_s;
    _x_s.ReverseSign();
    KDL::Vector y_s = source_frame.M.UnitY();
    KDL::Vector _y_s = y_s;
    _y_s.ReverseSign();
    KDL::Vector diff_v = target_frame.p - source_frame.p;

    // Defining the 4 dot products between diff_v and x_s, y_s, -x_s, -y_s vectors
    double xdot = dot(diff_v, x_s);
    double _xdot = dot(diff_v, _x_s);
    double ydot = dot(diff_v, y_s);
    double _ydot = dot(diff_v, _y_s);

    // choose the most negative dot product and set the hand pose but first check the correct side (sCbt or sCtt)
    // case 1: current source grasp is sCbt
    if(current_source_grasp == sCbt || current_source_grasp == sCbe)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_L", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_L", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 1: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_R", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_R", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 2: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_D", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_D", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 3: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_U", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_U", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 4: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for sliding. There might be some problems." << std::endl;
        return false;
    }
    }
    // case 2: current source grasp is sCtt
    else if(current_source_grasp == sCtt || current_source_grasp == sCte)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_L_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_L_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 5: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_R_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_R_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 6: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_D_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_D_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 7: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_slide &= node.getParam("ik_control_parameters/slide/Object_PreSlide_U_2", pOP);
        use_slide &= node.getParam("ik_control_parameters/slide/Object_Slide_U_2", pOS);
        use_slide &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_slide ){
            Object_PreSlide = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Slide = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 8: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for sliding. There might be some problems." << std::endl;
        return false;
    }
    }
}

bool semantic_to_cartesian_converter::set_hand_pose_tilting(geometry_msgs::Pose source_position, geometry_msgs::Pose target_position)
{
    ros::NodeHandle node;
    std::vector<double> pOP(6,0), pOS(6,0); //Parameters for Object_Pretilt and Object_Tilt

    // Here we check if the yaw angles of the two objects differ more than 45° -> if so, send warning
    KDL::Frame source_frame;
    tf::poseMsgToKDL(source_position, source_frame);
    KDL::Frame target_frame;
    tf::poseMsgToKDL(target_position, target_frame);
    double roll_s, pitch_s, yaw_s;
    double roll_t, pitch_t, yaw_t;
    source_frame.M.GetRPY(roll_s, pitch_s, yaw_s);
    target_frame.M.GetRPY(roll_t, pitch_t, yaw_t);

    // extracting x_s, y_s, -x_s, -y_s vectors of source object frame and the diff_v vector (difference between two object positions)
    KDL::Vector x_s = source_frame.M.UnitX();
    KDL::Vector _x_s = x_s;
    _x_s.ReverseSign();
    KDL::Vector y_s = source_frame.M.UnitY();
    KDL::Vector _y_s = y_s;
    _y_s.ReverseSign();
    KDL::Vector diff_v = target_frame.p - source_frame.p;

    // Defining the 4 dot products between diff_v and x_s, y_s, -x_s, -y_s vectors
    double xdot = dot(diff_v, x_s);
    double _xdot = dot(diff_v, _x_s);
    double ydot = dot(diff_v, y_s);
    double _ydot = dot(diff_v, _y_s);

    // choose the most negative dot product and set the hand pose but first check the correct side (sCbt or sCtt)
    // case 1: current source grasp is sCbt
    if(current_source_grasp == sCbt || current_source_grasp == sCbe)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_L", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_L", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 1: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_R", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_R", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 2: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_D", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_D", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 3: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_U", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_U", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 4: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for tilting. There might be some problems." << std::endl;
        return false;
    }
    }
    // case 2: current source grasp is sCtt
    else if(current_source_grasp == sCtt || current_source_grasp == sCte)
    {
    if(xdot < _xdot && xdot < ydot && xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_L_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_L_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 5: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_xdot < xdot && _xdot < ydot && _xdot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_R_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_R_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 6: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(ydot < xdot && ydot < _xdot && ydot < _ydot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_D_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_D_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 7: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else if(_ydot < xdot && _ydot < ydot && _ydot < _xdot){
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_PreTilt_U_2", pOP);
        use_tilt &= node.getParam("ik_control_parameters/tilt/Object_Tilt_U_2", pOS);
        use_tilt &= ((pOP.size() >= 6) && (pOS.size() >= 6));
    
        if( use_tilt ){
            Object_PreTilt = KDL::Frame(KDL::Rotation::EulerZYX(pOP.at(3),pOP.at(4),pOP.at(5)), KDL::Vector(pOP.at(0),pOP.at(1),pOP.at(2)));
            Object_Tilt = KDL::Frame(KDL::Rotation::EulerZYX(pOS.at(3),pOS.at(4),pOS.at(5)), KDL::Vector(pOS.at(0),pOS.at(1),pOS.at(2)));
            return true;
        }
        else{    
            std::cout << CLASS_NAMESPACE << __func__ << " 8: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
            return false;
        }
    }
    else{
        std::cout << CLASS_NAMESPACE << __func__ << " These source and target poses are quite strange for tilting. There might be some problems." << std::endl;
        return false;
    }
    }
}

node_info semantic_to_cartesian_converter::find_node_properties(const std::vector< dual_manipulation_shared::planner_item >& path, const std::vector< dual_manipulation_shared::planner_item >::const_iterator& node, std::vector< dual_manipulation_shared::planner_item >::const_iterator& next_node) const
{
    node_info result;
    auto ee_id = database.Grasps.at(node->grasp_id).ee_id;
    bool movable = database.EndEffectors.at(ee_id).movable;
    
    if(++next_node != path.end())
    {
        // check for supported cases:
        // all supported cases come from databaseMapper::getTransitionInfo
        transition_info t_info;
        object_state source_state(node->grasp_id,node->workspace_id);
        object_state target_state(next_node->grasp_id,next_node->workspace_id);
        #if DEBUG
        ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : source: " << source_state);
        ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : target: " << target_state);
        #endif
        bool supported_node = database.getTransitionInfo(source_state,target_state,t_info);
        if(supported_node)
        {
            ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : I'm supported! Type:" << t_info.grasp_transition_type_);
            result.type = t_info.grasp_transition_type_;
            result.busy_ees.insert(result.busy_ees.end(),t_info.ee_ids_.begin(),t_info.ee_ids_.end());
        }
        else
        {
            ROS_FATAL_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : there was a change of grasp which is NOT supported and should NEVER happen!");
            ROS_FATAL_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : source(g,w)=(" << node->grasp_id << "," << node->workspace_id << ") > target(g,w):(" << next_node->grasp_id << "," << next_node->workspace_id << ")");
            usleep(5000);
            abort();
        }
        
        // updating what depends on next_node
        result.next_ee_id = database.Grasps.at(next_node->grasp_id).ee_id;
        result.next_grasp_id = next_node->grasp_id;
        result.next_workspace_id = next_node->workspace_id;
    }
    else
    {
        // there is no next_node, use meaningless values for its find_node_properties
        result.next_ee_id = -1;
        result.next_grasp_id = -1;
        result.next_workspace_id = -1;
        
        if (!movable) result.type=node_properties::LAST_EE_FIXED;             //3.4.1) if not found, than ee_id is the last end effector in the path //not found not movable
        else result.type=node_properties::LAST_EE_MOVABLE;            //else //not found->last e.e, movable
    }
    
    result.current_ee_id=ee_id;
    result.current_grasp_id=node->grasp_id;
    result.current_workspace_id=node->workspace_id;
    return result;
}

bool semantic_to_cartesian_converter::convert(std::vector< std::pair< endeffector_id, cartesian_command > >& result, const std::vector< dual_manipulation_shared::planner_item >& path, const shared_memory& data, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes)
{
    // 1) Clearing result vector
    result.clear();
    
    // 2) Clear any previously found intergrasp configuration
    s2cik->clearCachedIkSolutions();
#if DEBUG
    // 2.1) Printing s and t positions for debugging
    std::cout<<"Starting obj_pose is: "<<std::endl<<source_position<<std::endl;
    std::cout<<"Arriving obj_pose is: "<<std::endl<<target_position<<std::endl;
#endif
    // 2.2) Setting the correct hand pose for a hypothetical case of a sliding
    bool hand_positioning = set_hand_pose_sliding(source_position, target_position);
    if (!hand_positioning){
        std::cout << CLASS_NAMESPACE << __func__ << " In convert: Parameters for Object Slide and Object preSlide not set correctly" << std::endl;
    }

    // 2.3) Setting the correct hand pose for a hypothetical case of a tilting
    bool tilt_positioning = set_hand_pose_tilting(source_position, target_position);
    if (!tilt_positioning){
        std::cout << CLASS_NAMESPACE << __func__ << " In convert: Parameters for Object Tilt and Object preTilt not set correctly" << std::endl;
    }
    
    // 3) Start of the main conversion loop
    for (auto node_it=path.begin();node_it!=path.end();)//++node)
    {
        // get info for the current node
        std::vector< dual_manipulation_shared::planner_item >::const_iterator next_node_it=node_it;
        node_info node = find_node_properties(path,node_it,next_node_it);
        //---------------------------
        
        if(!manage_transition_by_type.count(node.type))
        {
            ROS_FATAL_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : there is no implementation managing transitions of type \'" << node.type << "\'!");
            usleep(5000);
            abort();
        }
        else
        {
// #if DEBUG
            std::cout << CLASS_NAMESPACE << __func__ << " : transition of type \'" << node.type << "\'" << std::endl;
// #endif
            if(!(this->*manage_transition_by_type.at(node.type))(result,node,node_it,next_node_it,path,data,filtered_source_nodes,filtered_target_nodes))
                return false;
        }
        
        node_it=next_node_it;
    }
    // 4) return
    return true;
}

bool semantic_to_cartesian_converter::checkSingleGrasp(KDL::Frame& World_Object, node_info node, const shared_memory& data, bool first_node, bool last_node, dual_manipulation_shared::planner_item& filtered_source_nodes, dual_manipulation_shared::planner_item& filtered_target_nodes) const
{
    return s2cik->checkSingleGrasp(World_Object,node,data,first_node,last_node,filtered_source_nodes,filtered_target_nodes);
}

bool semantic_to_cartesian_converter::compute_intergrasp_orientation(KDL::Frame& World_Object, const node_info& node, object_id object) const
{
    return s2cik->compute_intergrasp_orientation(World_Object,node,object);
}

bool semantic_to_cartesian_converter::getGraspMatrixes(object_id object, node_info node, Object_GraspMatrixes& Object)
{
    return s2c_ik_converter::getGraspMatrixes(object,node,Object);
}

void semantic_to_cartesian_converter::getGraspMatrixesFatal(const shared_memory& data, node_info node, Object_GraspMatrixes& Object)
{
    if (!s2c_ik_converter::getGraspMatrixes(data.obj_id, node, Object))
    {
        std::cout << CLASS_NAMESPACE << __func__ << " : unable to get grasp matrixes for object " << data.object_name << " | grasps #" << node.current_grasp_id << "," << node.next_grasp_id << std::endl;
        abort();
    }
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_unknown)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_UNKNOWN." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    std::cout << CLASS_NAMESPACE << " : ERROR, the planner returned two nodes for which no known transition is implemented!!"<<std::endl;
    return false;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_last_ee_movable)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_LEEM." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    // 3.4.2) We move the last==current end effector in the final workspace centroid, equal to the final desired position
    cartesian_command move_command;
    move_command.command=cartesian_commands::MOVE;
    move_command.seq_num = 1;
    move_command.ee_grasp_id=node.current_grasp_id;
    KDL::Frame World_Object;
    tf::poseMsgToKDL(data.target_position,World_Object);
    tf::poseKDLToMsg(World_Object*Object.PostGraspFirstEE,move_command.cartesian_task);
    //TODO: what if this is not feasible? test other grasps? future work...
    result.push_back(std::make_pair(node.current_ee_id,move_command));
    
    return true;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_last_ee_fixed)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_LEEF." <<std::endl;
    #endif
    
    return true;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_grasp)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_GRASP." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    KDL::Frame World_Object,World_GraspSecondEE;
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    // 3.6) compute a rough position of the place where the change of grasp will happen
    if (!s2cik->checkSingleGrasp(World_Object,node,data,node_it==path.begin(),false,filtered_source_nodes,filtered_target_nodes))
        return false;
    World_GraspSecondEE = World_Object*Object.PreGraspSecondEE;
    cartesian_command move_command(cartesian_commands::MOVE_BEST_EFFORT, 1, node.next_grasp_id);
    tf::poseKDLToMsg(World_GraspSecondEE,move_command.cartesian_task);
    result.push_back(std::make_pair(node.next_ee_id,move_command)); //move the next
    
    //From fixed to movable we will grasp the object
    cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
    tf::poseKDLToMsg(World_Object,grasp.cartesian_task);
    result.push_back(std::make_pair(node.next_ee_id,grasp));
    // cartesian_command move_no_coll_command(cartesian_commands::MOVE_CLOSE_BEST_EFFORT, 1, node.next_grasp_id);
    // KDL::Frame World_postGraspSecondEE;
    // World_postGraspSecondEE = World_Object*Object.GraspFirstEE*(Object.PreGraspFirstEE.Inverse())*Object.PostGraspSecondEE;
    // tf::poseKDLToMsg(World_postGraspSecondEE,move_no_coll_command.cartesian_task);
    // result.push_back(std::make_pair(node.next_ee_id,move_no_coll_command));
    
    return true;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_ungrasp)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_UNGRASP." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    KDL::Frame World_Object;
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    cartesian_command move_command;
    move_command.command=cartesian_commands::MOVE_BEST_EFFORT;
    move_command.ee_grasp_id=node.current_grasp_id;
    move_command.seq_num=1;//do not parallelize with the fixed ee :)
    cartesian_command move_no_coll_command(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, node.current_grasp_id);
    // 3.6) compute a rough position of the place where the change of grasp will happen
    if (!s2cik->checkSingleGrasp(World_Object,node,data,false,((next_node_it+1) == path.end()),filtered_source_nodes,filtered_target_nodes))
        return false;
    KDL::Frame World_PreGraspSecondEE = World_Object*Object.GraspSecondEE*(Object.PreGraspSecondEE.Inverse())*Object.PostGraspFirstEE;
    tf::poseKDLToMsg(World_PreGraspSecondEE,move_command.cartesian_task);
    result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
    KDL::Frame World_GraspSecondEE = World_Object*Object.PostGraspFirstEE;
    tf::poseKDLToMsg(World_GraspSecondEE,move_no_coll_command.cartesian_task);
    result.push_back(std::make_pair(node.current_ee_id,move_no_coll_command)); //move the first
    cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
    // TODO: check the following transformation, should be more precisely something like 
    // TODO: "World_Object*Object_PostGraspFirstEE*(Object_GraspFirstEE.Inverse())"
    tf::poseKDLToMsg(World_Object,ungrasp.cartesian_task);
    result.push_back(std::make_pair(node.current_ee_id,ungrasp));
    cartesian_command move_away(cartesian_commands::HOME,0,-1);
    result.push_back(std::make_pair(node.current_ee_id,move_away));
    
    return true;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_exchange_grasp)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_EXCHANGE_GRASP." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    KDL::Frame World_Object,World_GraspSecondEE;
    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    cartesian_command move_command(cartesian_commands::MOVE,0,-1); // Care, we are parallelizing here!
    // 3.6) compute a rough position of the place where the change of grasp will happen
    if(!s2cik->compute_intergrasp_orientation(World_Object,node,data.obj_id,filtered_source_nodes,filtered_target_nodes))
        return false;
    KDL::Frame World_GraspFirstEE = World_Object*Object.PostGraspFirstEE;
    tf::poseKDLToMsg(World_GraspFirstEE,move_command.cartesian_task);
    result.push_back(std::make_pair(node.current_ee_id,move_command)); //move the first
    cartesian_command second_move_command(cartesian_commands::MOVE,1,-1); // do NOT parallelize;
    World_GraspSecondEE = World_Object*Object.PreGraspSecondEE;
    tf::poseKDLToMsg(World_GraspSecondEE,second_move_command.cartesian_task);
    result.push_back(std::make_pair(node.next_ee_id,second_move_command)); //move the next
    //From movable to movable we will grasp the object and ungrasp it
    cartesian_command grasp(cartesian_commands::GRASP,1,node.next_grasp_id);
    // make sure that the grasp/ungrasp actions have the object frame
    tf::poseKDLToMsg(World_Object,grasp.cartesian_task);
    result.push_back(std::make_pair(node.next_ee_id,grasp));
    cartesian_command ungrasp(cartesian_commands::UNGRASP,1,node.current_grasp_id);
    tf::poseKDLToMsg(World_Object,ungrasp.cartesian_task);
    result.push_back(std::make_pair(node.current_ee_id,ungrasp));
    cartesian_command move_away(cartesian_commands::HOME,0,-1);
    result.push_back(std::make_pair(node.current_ee_id,move_away));
    
    return true;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_move_nonblocking)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_MNB." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    Object_GraspMatrixes Object;
    getGraspMatrixesFatal(data,node,Object);
    
    // move the end-effector in a non-blocking way (i.e., set the target and possibly set new ones later on
    cartesian_command move_command;
    move_command.command=cartesian_commands::MOVE;
    move_command.seq_num = 0;
    move_command.ee_grasp_id=node.current_grasp_id;
    KDL::Frame World_Object;
    tf::poseMsgToKDL(data.target_position,World_Object);
    tf::poseKDLToMsg(World_Object*Object.PostGraspFirstEE,move_command.cartesian_task);
    result.push_back(std::make_pair(node.current_ee_id,move_command));
    
    return true;
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_slide)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_SLIDE." <<std::endl;
    #endif
    
    current_transition_tilting = false;

    if (!use_slide)
        return false;

    KDL::Frame World_Object,World_GraspSecondEE;
    Object_GraspMatrixes Object;
    
    endeffector_id ee_id_to_use = 0;
    
    for (endeffector_id ee:node.busy_ees)
    {
        if (    data.db_mapper.Reachability.count(ee) && data.db_mapper.Reachability.at(ee).count(node.current_workspace_id) 
                && data.db_mapper.Reachability.at(ee).count(node.next_workspace_id) )
        {
            ee_id_to_use = ee;
            break;
        }
    }
    
    assert(ee_id_to_use != 0);

    std::vector<KDL::Frame> object_ee_poses({Object_PreSlide, Object_Slide});
    std::vector<KDL::Frame> World_Object_Poses;
    
    std::string ee_name = data.db_mapper.EndEffectors.at(ee_id_to_use).name;
    if (!s2cik->checkSlidePoses(World_Object_Poses,node,data,node_it==path.begin(),((next_node_it+1) == path.end()),filtered_source_nodes,filtered_target_nodes, object_ee_poses, ee_name))
        return false;
    
    cartesian_command move_command(cartesian_commands::MOVE_BEST_EFFORT, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(0)*Object_PreSlide ,move_command.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, move_command)); //move to PreSliding 
    
    cartesian_command move_command_2(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(0)*Object_Slide ,move_command_2.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, move_command_2)); //move to pose for Sliding 
    
    cartesian_command slide_command(cartesian_commands::SLIDE, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(1)*Object_Slide ,slide_command.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, slide_command)); //do Sliding

    cartesian_command move_command_3(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(1)*Object_PreSlide ,move_command_3.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, move_command_3)); //move to PostSliding 
    
    cartesian_command move_away(cartesian_commands::HOME,0,-1);
    result.push_back(std::make_pair(ee_id_to_use,move_away));
     
   
    return true;
    
}

MANAGE_TRANSITION_FUN_MACRO(semantic_to_cartesian_converter::manage_transition_tilt)
{
    #if DEBUG
    std::cout<< "I ENTERED MANAGE_TRANSITION_TILT." <<std::endl;
    #endif

    current_transition_tilting = true;

    if (!use_tilt)
        return false;

    KDL::Frame World_Object,World_GraspSecondEE;
    Object_GraspMatrixes Object;
    
    endeffector_id ee_id_to_use = 0;
    
    for (endeffector_id ee:node.busy_ees)
    {
        if (    data.db_mapper.Reachability.count(ee) && data.db_mapper.Reachability.at(ee).count(node.current_workspace_id) 
                && data.db_mapper.Reachability.at(ee).count(node.next_workspace_id) )
        {
            ee_id_to_use = ee;
            break;
        }
    }
    
    assert(ee_id_to_use != 0);

    std::vector<KDL::Frame> object_ee_poses({Object_PreTilt, Object_Tilt});
    std::vector<KDL::Frame> World_Object_Poses;
    
    std::string ee_name = data.db_mapper.EndEffectors.at(ee_id_to_use).name;
    if (!s2cik->checkSlidePoses(World_Object_Poses,node,data,node_it==path.begin(),((next_node_it+1) == path.end()),filtered_source_nodes,filtered_target_nodes, object_ee_poses, ee_name))
        return false;
    
    cartesian_command move_command(cartesian_commands::MOVE_BEST_EFFORT, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(0)*Object_PreTilt ,move_command.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, move_command)); //move to PreTilting 
    
    cartesian_command move_command_2(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(0)*Object_Tilt ,move_command_2.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, move_command_2)); //move to pose for Tilting 
    
    cartesian_command slide_command(cartesian_commands::SLIDE, 1, -1);
    // As current cartesian command is TILT -> set bool tilt_only_now to true
    slide_command.tilt_only_now = true;
    tf::poseKDLToMsg(World_Object_Poses.at(1)*Object_Tilt ,slide_command.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, slide_command)); //do Tilting

    cartesian_command move_command_3(cartesian_commands::MOVE_NO_COLLISION_CHECK, 1, -1);
    tf::poseKDLToMsg(World_Object_Poses.at(1)*Object_PreTilt ,move_command_3.cartesian_task);
    result.push_back(std::make_pair(ee_id_to_use, move_command_3)); //move to PostTilting 
    
    cartesian_command move_away(cartesian_commands::HOME,0,-1);
    result.push_back(std::make_pair(ee_id_to_use,move_away));
   
    return true;
    
}
