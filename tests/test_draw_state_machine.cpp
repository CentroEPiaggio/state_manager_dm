#include <draw_state_machine.hpp>

int main(int argc, char *argv[])
{
    dual_manipulation::state_manager::draw_state_machine<std::string,std::string> sm;
    std::vector<std::tuple<std::string,std::string,std::string>> transition_table{
        //------initial state---------+--------- command -----------------------------------+-- final state---- +
        std::make_tuple( "starting"     , "transition::started"            ,    "steady"         ),
        std::make_tuple( "steady"       , "transition::get_info"           ,    "getting_info"   ),
        std::make_tuple( "getting_info" , "transition::got_info"           ,    "ready"          ),
        std::make_tuple( "ready"        , "transition::plan"               ,    "planning"       ),
        std::make_tuple( "ready"        , "transition::get_info"           ,    "getting_info"   ),
        std::make_tuple( "planning"     , "transition::abort_plan"         ,    "steady"         ),
        std::make_tuple( "planning"     , "transition::start_moving"       ,    "moving"         ),
        std::make_tuple( "moving"       , "transition::task_accomplished"  ,    "steady"         )
    };
    sm.insert(transition_table);
    sm.draw_on_file("test.eps");
    return 0;
}
