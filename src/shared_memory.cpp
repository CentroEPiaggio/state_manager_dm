#include "../include/shared_memory.h"

shared_memory::shared_memory()
{
    reset();
}

void shared_memory::reset()
{
    cartesian_plan.clear();
    filtered_source_nodes.clear();
    filtered_target_nodes.clear();
    obj_id=-1;
    object_name="";
    source_grasp=-1;
    target_grasp=-1;
    target_position=geometry_msgs::Pose();
    source_position=geometry_msgs::Pose();
}


std::ostream& operator<<(std::ostream &output, const cartesian_command &o) {
    output<<std::endl;
    output<<o.cartesian_task<<"  "<<(o.command==cartesian_commands::HOME?"home":o.command==cartesian_commands::MOVE?"move":o.command==cartesian_commands::GRASP?"grasp":o.command==cartesian_commands::UNGRASP?"ungrasp":"")<<" "<<o.seq_num<<std::endl;
    return output;
}