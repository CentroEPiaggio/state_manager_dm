#include <ros/ros.h>
#include <ros_server.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init(argc, argv, "dual_manipulation_state_manager");
    }
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> State manager running"<<std::endl;
    std::cout<<std::endl;
    
    dual_manipulation::state_manager::ros_server server;
    server.join();
    return 0;
}
