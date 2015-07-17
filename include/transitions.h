#ifndef TRANSITIONS_H
#define TRANSITIONS_H

/**
 * @brief these transitions are used in the main state machine inside state manager
 * 
 */
enum class transition {
    started, /**<modules up and running, vision system up, GUI up*/
    get_info, /**<user cmd from GUI*/
    got_info, /**<objects found, graphs created*/
    plan, /**<user cmd from GUI, requires informations loaded*/
    abort_plan, /**<user cmd from GUI*/
    re_plan, /**<cartesian conversion failed, going to re_plan*/
    failed_plan, /**<plan was bad, no cartesian commands were produced*/
    good_plan, /**< plan was good, ready to move*/
    start_moving, /**<user cmd from GUI*/
    abort_move, /**<user cmd from GUI*/
    task_accomplished, /**<object was successfully put in final place*/
    planning_done, /**<plan is done*/
    exit, /**<exit the program*/
    failed /**<a generic failure condition*/
};


/**
 * @brief these transitions are used in the ik control state machine
 * 
 */
enum class ik_transition {
    plan, /**<require a low level ik planning*/
    move, /**<ask the robot to move following the low level ik plan*/
    grasp, /**<not used*/
    done, /**<everything went fine*/
    check_grasp, /**<move state decided to check if a grasp is as planned*/
    check_done, /**<not used*/
    soft_fail, /**<the state has failed, but application can continue to run*/
    fail, /**<the state can decide to fail without any recover option*/
    need_replan /**<a backtracking on the semantic graph is needed*/
};

typedef std::pair<transition,bool> transition_type;

typedef std::pair<ik_transition,bool> ik_transition_type;


#endif //TRANSITIONS_H