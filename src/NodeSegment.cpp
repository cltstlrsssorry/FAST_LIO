#include "NodeSegment.h"

NodeSegment::NodeSegment(const std::string & name):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "----NodeSegment----"); // 输出信息，初始化结束

    getConfig();
    
    uniformed_map_ptr_=PointCloudXYZI::Ptr(new PointCloudXYZI());

    pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/publish_segment", 20);

    auto timer_seg = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    reg_map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), timer_seg, std::bind(&NodeSegment::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodeSegment init finished.----"); // 输出信息，初始化结束

}

void NodeSegment::getConfig()
{
    this->declare_parameter<bool>("segment.segment_en", false);
    this->declare_parameter<bool>("segment.publish_map_en", false);
    this->declare_parameter<double>("segment.uniform_size", 0.1);

    this->get_parameter_or<bool>("segment.segment_en", config.segment_en, false);
    this->get_parameter_or<bool>("segment.publish_map_en", config.publish_map_en, false);
    this->get_parameter_or<double>("segment.uniform_size", config.uniform_size, 0.1); 
}

void NodeSegment::timer_callback()
{
    if(!config.segment_en) return;   

    if(filter_dynamic_map.empty()) return;
 
    PointCloudXYZI::Ptr filter_dynamic_map_ptr_ = filter_dynamic_map.begin()->points;
    double timestamp = filter_dynamic_map.begin()->time;
    filter_dynamic_map.pop_front();

    // 点云分割
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>); // kd树对象
    tree->setInputCloud(filter_dynamic_map_ptr_);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointType> ec; // 创建欧式聚类分割对象
    ec.setClusterTolerance(0.1);                       // 设置近邻搜索的搜索半径
    ec.setMinClusterSize(5000);                         // 设置最小聚类尺寸
    ec.setMaxClusterSize(1000000);                      // 设置最大聚类尺寸
    ec.setSearchMethod(tree);
    ec.setInputCloud(filter_dynamic_map_ptr_);
    ec.extract(clusters);

    int maxnum=0;
    pcl::PointIndices Max_clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
    {
        if (it->indices.size() > maxnum)
        {
            maxnum = it->indices.size();
            Max_clusters = *it;
        }
    }

    PointCloudXYZI::Ptr segment_map_ptr_=PointCloudXYZI::Ptr(new PointCloudXYZI());
    for (std::vector<int>::const_iterator pit = Max_clusters.indices.begin(); pit != Max_clusters.indices.end(); pit++)
        segment_map_ptr_->points.push_back(filter_dynamic_map_ptr_->points[*pit]);
    
    segment_map_ptr_->width = segment_map_ptr_->points.size();
    segment_map_ptr_->height = 1;
    segment_map_ptr_->is_dense = true;

    if (config.publish_map_en)
    {
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*segment_map_ptr_, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(timestamp);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudMap->publish(laserCloudmsg);
    }

    segment_map.push_back(PointLists(timestamp,segment_map_ptr_));
}

void NodeSegment::VoxelPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, double leaf_size)
{
    if (cloud->empty())
        return;
    
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud_voxelized);
}

void NodeSegment::UniformSamplingPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_uniformed, double leaf_size)
{
    if (cloud->empty())
        return;
    
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(leaf_size);
    uniform_sampling.filter(*cloud_uniformed);
}