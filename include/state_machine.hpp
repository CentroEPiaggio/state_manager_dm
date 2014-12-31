#ifndef BASIC_STATE_MACHINE_HPP
#define BASIC_STATE_MACHINE_HPP

#include <map>
#include <vector>

namespace dual_manipulation{
    namespace state_manager{
        
        /** 
         * @brief STATE MACHINE EXAMPLE
         *
         std::vector<std::tuple<state,transition,state>> transition_table{
         //--------------initial state ----------+--------- command ---------+------ final state--------- +
         std::make_tuple( state::idle            ,   COMMAND_VALVE_DATA_SENT ,    state::ready            ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::ready           ,   COMMAND_REACH           ,    state::reaching         ),
         std::make_tuple( state::ready           ,   COMMAND_VALVE_DONE      ,    state::idle             ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::reaching        ,   COMMAND_ACTION_DONE     ,    state::reached          ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::reached         ,   COMMAND_APPROACH        ,    state::approaching      ),
         std::make_tuple( state::reached         ,   COMMAND_MOVE_AWAY       ,    state::moving_away      ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::approaching     ,   COMMAND_ACTION_DONE     ,    state::approached       ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::approached      ,   COMMAND_REACH           ,    state::reaching         ),
         std::make_tuple( state::approached      ,   COMMAND_MOVE_AWAY       ,    state::moving_away      ),
         std::make_tuple( state::approached      ,   COMMAND_GRASP           ,    state::grasping         ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::grasping        ,   COMMAND_HAND_DONE       ,    state::grasped          ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::grasped         ,   COMMAND_TURN            ,    state::valve_rotating   ),
         std::make_tuple( state::grasped         ,   COMMAND_UNGRASP         ,    state::ungrasping       ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::valve_rotating  ,   COMMAND_ACTION_DONE     ,    state::valve_rotated    ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::valve_rotated   ,   COMMAND_UNGRASP         ,    state::ungrasping       ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::ungrasping      ,   COMMAND_HAND_DONE       ,    state::ungrasped        ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::ungrasped       ,   COMMAND_GRASP           ,    state::grasping         ),
         std::make_tuple( state::ungrasped       ,   COMMAND_REACH           ,    state::reaching         ),
         std::make_tuple( state::ungrasped       ,   COMMAND_MOVE_AWAY       ,    state::moving_away      ),
         std::make_tuple( state::ungrasped       ,   COMMAND_VALVE_DONE      ,    state::idle             ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::moving_away     ,   COMMAND_ACTION_DONE     ,    state::moved_away       ),
         //--------------------------------------+---------------------------+----------------------------+
         std::make_tuple( state::moved_away      ,   COMMAND_REACH           ,    state::reaching         ),
         std::make_tuple( state::moved_away      ,   COMMAND_VALVE_DONE      ,    state::idle             ),
    };*/
        
        template <class state_type, class transition_type>
        class state_machine
        {
        public:
            std::map<state_type,std::map<transition_type,state_type>> transition_table;
            state_type evolve_state_machine(state_type current_state, transition_type command)
            {
                if (transition_table.count(current_state))
                {
                    if (transition_table.at(current_state).count(command))
                        return transition_table.at(current_state).at(command);
                }
                return current_state;
            }
            
            void insert(std::vector< std::tuple< state_type, transition_type, state_type > > table)
            {
                for (auto row:table)
                    insert(row);
            }
            
            void insert(std::tuple< state_type, transition_type, state_type > row)
            {
                transition_table[std::get<0>(row)][std::get<1>(row)]=std::get<2>(row);
            }
            
            state_machine()
            {
            }
        };
        
    }
}
#endif // BASIC_STATE_MACHINE_HPP