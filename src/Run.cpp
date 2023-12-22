#include "NodePreProcess.h"
#include "laserMapping.hpp"
//#include "NodeCloudSegment.h"
//#include "NodeERASOR.h"
#include "NodeOctomap.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto cloudprocess=std::make_shared<NodeCloudProcess>("cloudprocess");
    auto laserReg=std::make_shared<LaserMappingNode>("laserReg");
    //auto dynfilter=std::make_shared<NodeERASOR>("dynfilter");
    auto octomap=std::make_shared<NodeOctomap>("octomap");
    // auto laserSeg=std::make_shared<NodeCloudSegment>("laserSeg");

    executor.add_node(cloudprocess);
    executor.add_node(laserReg);
    //executor.add_node(dynfilter);
    executor.add_node(octomap);
    // executor.add_node(laserSeg);

    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    
    return 0;
}