#include "semantic_planning_state.h"
#include <dual_manipulation_shared/stream_utils.h>
int main(int argc, char *argv[])
{
    shared_memory data;
    semantic_planning_state state(data);
    std::vector<std::pair<endeffector_id,cartesian_command>> result;
    dual_manipulation_shared::planner_serviceResponse::_path_type path;
    dual_manipulation_shared::planner_item item;
/*
    5 3
    5 2
    3 2
    3 1
  */  
    item.grasp_id=5;
    item.workspace_id=3;
    path.push_back(item);
    item.grasp_id=5;
    item.workspace_id=2;
    path.push_back(item);
    item.grasp_id=3;
    item.workspace_id=2;
    item.grasp_id=3;
    path.push_back(item);
    item.workspace_id=1;
    path.push_back(item);
    state.semantic_to_cartesian(result,path);
    std::cout<<result<<std::endl;
    return 0;
}
