#include "semantic_planning_state.h"
#include <dual_manipulation_shared/stream_utils.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "conversion_test");
//     sleep(10);
    shared_memory data;
    data.source_position.position.x = 0.25;
    data.source_position.position.y = 0.3;
    data.source_position.position.z = 0;
    data.target_position.position.x = -0.312;
    data.target_position.position.y = 0.545;
    data.target_position.position.z = 0;
    semantic_planning_state state(data);
    std::vector<std::pair<endeffector_id,cartesian_command>> result;
    dual_manipulation_shared::planner_serviceResponse::_path_type path,path1;
    dual_manipulation_shared::planner_item item;
/*
    5 3
    5 2
    3 2
    3 1
  */  
    item.grasp_id=7;
    item.workspace_id=3;
    path.push_back(item);
    item.grasp_id=5;
    item.workspace_id=3;
    path.push_back(item);
    item.grasp_id=5;
    item.workspace_id=2;
    path.push_back(item);
    item.grasp_id=3;
    item.workspace_id=2;
    path.push_back(item);
    item.grasp_id=3;
    item.workspace_id=1;
    path.push_back(item);
    state.semantic_to_cartesian(result,path);
    for (auto i:result)
        std::cout<<i<<std::endl;
    auto item_it=path.begin();
    item_it++;
    for (;item_it!=path.end();++item_it)
    {
        path1.push_back(*item_it);
    }
    state.semantic_to_cartesian(result,path1);
    for (auto i:result)
        std::cout<<i<<std::endl;
    return 0;
}
