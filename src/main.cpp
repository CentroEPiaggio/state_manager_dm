#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "state_manager", ros::init_options::AnonymousName );
    }
    
    return 0;
}
