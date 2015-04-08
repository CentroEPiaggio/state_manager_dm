#include "semantic_planning_state.h"
#include <dual_manipulation_shared/stream_utils.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "conversion_test");
//     sleep(10);
    databaseMapper database;
    shared_memory data;
    data.source_position.position.x = 0.25;
    data.source_position.position.y = 0.3;
    data.source_position.position.z = 0;
    data.target_position.position.x = -0.312;
    data.target_position.position.y = 0.545;
    data.target_position.position.z = 0;
    data.obj_id = 1;
    semantic_to_cartesian_converter state(database);
    std::vector<std::pair<endeffector_id,cartesian_command>> result;
    dual_manipulation_shared::planner_serviceResponse::_path_type path,path1;
    dual_manipulation_shared::planner_item item;
/*
    5 3
    5 2
    3 2
    3 1
  */  
    item.grasp_id=3;
    item.workspace_id=3;
    path.push_back(item);
    item.grasp_id=1;
    item.workspace_id=3;
    path.push_back(item);
    item.grasp_id=1;
    item.workspace_id=2;
    path.push_back(item);
    item.grasp_id=3;
    item.workspace_id=2;
    path.push_back(item);
    item.grasp_id=2;
    item.workspace_id=2;
    path.push_back(item);
    item.grasp_id=2;
    item.workspace_id=1;
    path.push_back(item);
    item.grasp_id=3;
    item.workspace_id=1;
    path.push_back(item);
    
    std::cout << "path: " << path << std::endl;
    
    std::vector<dual_manipulation_shared::planner_item> a,b; 
    state.convert(result,path,data,a,b);
    for (auto i:result)
        std::cout<<i<<std::endl;
    auto item_it=path.begin();
    item_it++;
    for (;item_it!=path.end();++item_it)
    {
        path1.push_back(*item_it);
    }
    state.convert(result,path,data,a,b);
    for (auto i:result)
        std::cout<<i<<std::endl;
    return 0;
}
