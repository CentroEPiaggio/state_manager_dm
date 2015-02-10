#include "../include/shared_memory.h"

shared_memory::shared_memory()
{

}

std::ostream& operator<<(std::ostream &output, const cartesian_command &o) {
    output<<std::endl;
    output<<o.cartesian_task<<"  "<<(o.command==cartesian_commands::MOVE?"move":o.command==cartesian_commands::GRASP?"grasp":o.command==cartesian_commands::UNGRASP?"ungrasp":"")<<" "<<o.seq_num<<std::endl;
    return output;
}