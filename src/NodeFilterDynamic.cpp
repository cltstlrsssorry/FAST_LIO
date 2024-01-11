#include "NodeFilterDynamic.h"

NodeFilterDynamic::NodeFilterDynamic(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
        RCLCPP_INFO(this->get_logger(), "----NodeFilterDynamic----");
        
        publish_count = 1;
        
        raw_map_ptr_.reset(new PointCloudXYZI());
        
        getConfig();
        
        Octomap_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/publish_down_dynamic_map", 10);
        
        auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        
        timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodeFilterDynamic::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "----NodeFilterDynamic init finished.----"); // 输出信息，初始化结束
}

void NodeFilterDynamic::timer_callback()
{
    if (raw_points_list.empty()) return;

    if (m_octree == NULL) return;

    PointCloudXYZI::Ptr temp_pc = raw_points_list.begin()->points;

    timestamp = raw_points_list.begin()->time;

    raw_points_list.pop_front();

    publish_count++;

    if (temp_pc->points.size() < 50) return;

    *raw_map_ptr_ += *temp_pc;

    if (publish_count % 5 == 0) UniformSamplingPointCloud(raw_map_ptr_, raw_map_ptr_, uniform_size);

    if (publish_count % 50 == 0)
    {
        PointCloudXYZI::Ptr temp_points_ptr(new PointCloudXYZI());
        PointCloudXYZI::Ptr temp_points_pca_ptr(new PointCloudXYZI());

        for (auto &pt : raw_map_ptr_->points)
        {
            octomap::point3d point(pt.x, pt.y, pt.z);
            octomap::OcTreeNode *node = m_octree->search(point);
            if (node == NULL) continue;
            if (m_octree->isNodeOccupied(node)) temp_points_ptr->push_back(pt);
        }

        if (temp_points_ptr->size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "octomap_map_ is empty, no map is saved");
            return;
        }

        //传值到filter_dynamic_map
        raw_map_ptr_->clear();
        pcl::copyPointCloud(*temp_points_ptr, *raw_map_ptr_);
        filter_dynamic_map.push_back(PointLists(timestamp, temp_points_ptr));

        if(filter_dynamic_map_PCA.empty())
        {
            pcl::copyPointCloud(*temp_points_ptr,*temp_points_pca_ptr);
            filter_dynamic_map_PCA.push_back(PointLists(timestamp, temp_points_pca_ptr));
        }

        //发布点云
        if(publish_map_en)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*temp_points_ptr, laserCloudmsg);
            // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
            laserCloudmsg.header.stamp = get_ros_time(timestamp);
            laserCloudmsg.header.frame_id = "camera_init";
            Octomap_publisher->publish(laserCloudmsg);
        }

        publish_count = 0;

    }
}

void NodeFilterDynamic::getConfig()
{
        this->declare_parameter<double>("publishmap.size", 0.02);
        this->declare_parameter<bool>("publishmap.publish_map_en", true);

        this->get_parameter_or<double>("publishmap.size", uniform_size, 0.02);
        this->get_parameter_or<bool>("publishmap.publish_map_en", publish_map_en, true);
}

void NodeFilterDynamic::VoxelPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, double size)
{
        if (cloud->empty())
                return;

        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(size, size, size);
        voxel_grid.filter(*cloud_voxelized);
}

void NodeFilterDynamic::UniformSamplingPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_uniformed, double size)
{
        if (cloud->empty())
                return;

        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud(cloud);
        uniform_sampling.setRadiusSearch(size);
        uniform_sampling.filter(*cloud_uniformed);
}