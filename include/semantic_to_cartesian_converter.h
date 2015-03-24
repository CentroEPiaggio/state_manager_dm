#ifndef SEMANTIC_TO_CARTESIAN_CONVERTER_H
#define SEMANTIC_TO_CARTESIAN_CONVERTER_H
#include <dual_manipulation_shared/databasemapper.h>

enum class node_properties
{
    FINAL_NODE,                //  if (final_node==path.end())
    MOVABLE_TO_FIXED,          //one is movable, change on ground
    FIXED_TO_MOVABLE,          //one is movable, change on ground
    MOVABLE_TO_MOVABLE,        //both ee are movable: change above ground
    FIXED_TO_FIXED,            //if (!movable && !next_movable)
    LAST_EE_FIXED,             //3.4.1) if not found, than ee_id is the last end effector in the path //not found not movable
    LAST_EE_MOVABLE            //else //not found->last e.e, movable
};


class semantic_to_cartesian_converter
{
public:
     semantic_to_cartesian_converter(const databaseMapper& database);
    
private:
    
     private:
     databaseMapper database;
};

#endif // SEMANTIC_TO_CARTESIAN_CONVERTER_H
