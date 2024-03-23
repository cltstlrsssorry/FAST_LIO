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

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int thread_num=std::thread::hardware_concurrency();
    rclcpp::executors::MultiThreadedExecutor multi_executor(rclcpp::ExecutorOptions(), thread_num-3);
    
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

    multi_executor.add_node(cloudprocess);
    multi_executor.add_node(laserReg);
    //executor.add_node(dynfilter);
    multi_executor.add_node(octomap);
    // executor.add_node(laserSeg);
    multi_executor.add_node(publishmap);
    multi_executor.add_node(laserSeg);
    multi_executor.add_node(laserPCA);
    multi_executor.add_node(laserCC);
    multi_executor.add_node(lasertriangles);

    multi_executor.spin();

    /**
    while(rclcpp::ok())
    {
        multi_executor.spin_some();
    }
    */

    if(rclcpp::shutdown())
    {
        raw_points_list.clear();
        wait_octomap_points_list.clear();
        time_buffer.clear();
        lidar_buffer.clear();
        imu_buffer.clear();

        m_octree->clear();
    }

    return 0;
}