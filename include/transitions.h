#ifndef TRANSITIONS_H
#define TRANSITIONS_H

enum class transition {
    //TODO
    started, //modules up and running, vision system up, GUI up
    get_info, //user cmd from GUI
    got_info, //objects found, graphs created
    plan, //user cmd from GUI, requires informations loaded
    abort_plan, //user cmd from GUI
    start_moving, //user cmd from GUI
    task_accomplished, //object was successfully put in final place
    planning_done, //plan is done
    exit, //exit the program
};


typedef std::pair<transition,bool> transition_type;




#endif //TRANSITIONS_H