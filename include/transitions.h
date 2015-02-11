#ifndef TRANSITIONS_H
#define TRANSITIONS_H

enum class transition {
    //TODO
    started, //modules up and running, vision system up, GUI up
    get_info, //user cmd from GUI
    got_info, //objects found, graphs created
    plan, //user cmd from GUI, requires informations loaded
    abort_plan, //user cmd from GUI
    failed_plan, //plan was bad, no cartesian commands were produced
    good_plan, // plan was good, ready to move
    start_moving, //user cmd from GUI
    abort_move, //user cmd from GUI
    task_accomplished, //object was successfully put in final place
    planning_done, //plan is done
    exit, //exit the program
};

enum class ik_transition {
    plan,
    move,
    grasp,
    done,
    checkgrasp,
    check_done
};

typedef std::pair<transition,bool> transition_type;

typedef std::pair<ik_transition,bool> ik_transition_type;


#endif //TRANSITIONS_H