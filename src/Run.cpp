#include "NodePreProcess.h"
#include "laserMapping.hpp"
//#include "NodeCloudSegment.h"
//#include "NodeERASOR.h"
#include "NodeOctomap.h"
#include "NodePublishMap.h"
#include "NodeSegment.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto cloudprocess=std::make_shared<NodeCloudProcess>("cloudprocess");
    auto laserReg=std::make_shared<LaserMappingNode>("laserReg");
    //auto dynfilter=std::make_shared<NodeERASOR>("dynfilter");
    auto octomap=std::make_shared<NodeOctomap>("octomap");
    auto publishmap=std::make_shared<NodePublishMap>("publishmap");
    // auto laserSeg=std::make_shared<NodeCloudSegment>("laserSeg");
    auto laserSeg=std::make_shared<NodeSegment>("laserSeg");

    executor.add_node(cloudprocess);
    executor.add_node(laserReg);
    //executor.add_node(dynfilter);
    executor.add_node(octomap);
    // executor.add_node(laserSeg);
    executor.add_node(publishmap);
    executor.add_node(laserSeg);

    executor.spin();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();

        raw_points_list.clear();
        down_size_points_list.clear();
        time_buffer.clear();
        lidar_buffer.clear();
        imu_buffer.clear();

        m_octree->clear();
    }

    return 0;
}