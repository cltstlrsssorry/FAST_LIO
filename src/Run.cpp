#include "NodePreProcess.h"
#include "laserMapping.hpp"
//#include "NodeCloudSegment.h"
//#include "NodeERASOR.h"
#include "NodeOctomap.h"
#include "NodeFilterDynamic.h"
#include "NodeSegment.h"
#include "NodePCA.h"
#include "NodeCloudCompare.h"
#include "NodeReconstruction.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto cloudprocess=std::make_shared<NodeCloudProcess>("cloudprocess");
    auto laserReg=std::make_shared<LaserMappingNode>("laserReg");
    //auto dynfilter=std::make_shared<NodeERASOR>("dynfilter");
    auto octomap=std::make_shared<NodeOctomap>("octomap");
    auto publishmap=std::make_shared<NodeFilterDynamic>("publishmap");
    // auto laserSeg=std::make_shared<NodeCloudSegment>("laserSeg");
    auto laserSeg=std::make_shared<NodeSegment>("laserSeg");
    auto laserPCA=std::make_shared<NodePCA>("laserPCA");
    auto laserCC=std::make_shared<NodeCloudCompare>("laserCC");
    auto lasertriangles=std::make_shared<NodeReconstruction>("triangles");

    executor.add_node(cloudprocess);
    executor.add_node(laserReg);
    //executor.add_node(dynfilter);
    executor.add_node(octomap);
    // executor.add_node(laserSeg);
    executor.add_node(publishmap);
    executor.add_node(laserSeg);
    executor.add_node(laserPCA);
    executor.add_node(laserCC);
    executor.add_node(lasertriangles);

    executor.spin();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();

        raw_points_list.clear();
        wait_octomap_points_list.clear();
        time_buffer.clear();
        lidar_buffer.clear();
        imu_buffer.clear();

        m_octree->clear();
    }
    
    // 等待直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    return 0;
}