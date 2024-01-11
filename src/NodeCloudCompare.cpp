#include "NodeCloudCompare.h"

NodeCloudCompare::NodeCloudCompare(const std::string & name):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
        RCLCPP_INFO(this->get_logger(), "----NodeCloudCompare----");

        setConfig();

        inited = false;

        octree_detect.reset(new pcl::octree::OctreePointCloudChangeDetector<PointType>(config.resolution));

        last_cloud.reset(new PointCloudXYZI());

        curr_cloud.reset(new PointCloudXYZI());

        changed_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("changed_points", 10);

        auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0));

        timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodeCloudCompare::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "----NodeCloudCompare init finished.----"); // 输出信息，初始化结束
}

void NodeCloudCompare::timer_callback()
{
        if (segment_map.empty()) return;

        curr_cloud = segment_map.begin()->points;
        timestamp = segment_map.begin()->time;
        segment_map.pop_front();

        //粗获取变化点
        PointCloudXYZI::Ptr changed_points_ptr(new PointCloudXYZI);
        get_changed_points(curr_cloud, changed_points_ptr);

        //发布变化点
        PointCloudXYZI::Ptr collection_points_ptr(new PointCloudXYZI);
        get_collection_changed_points(changed_points_ptr, collection_points_ptr);


        if (collection_points_ptr->size() == 0) return;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*collection_points_ptr, msg);
        msg.header.frame_id = "camera_init";
        msg.header.stamp = get_ros_time(timestamp);
        changed_points_pub->publish(msg);

        PointCloudXYZI::Ptr last_cloud = curr_cloud;
}

void NodeCloudCompare::setConfig()
{
        this->declare_parameter<float>("cloudcompare.resolution", 0.1);
        this->declare_parameter<float>("cloudcompare.radius", 0.1);
        this->declare_parameter<float>("cloudcompare.clusterTolerance", 0.02);
        this->declare_parameter<int>("cloudcompare.minClusterSize",10);
        this->declare_parameter<int>("cloudcompare.maxClusterSize",100);

        this->get_parameter_or<float>("cloudcompare.resolution",config.resolution, 0.1);
        this->get_parameter_or<float>("cloudcompare.radius",config.radius, 0.1);
        this->get_parameter_or<float>("cloudcompare.clusterTolerance", config.clusterTolerance, 0.02);
        this->get_parameter_or<int>("cloudcompare.minClusterSize", config.minClusterSize, 10);
        this->get_parameter_or<int>("cloudcompare.maxClusterSize", config.maxClusterSize, 100);
}

void NodeCloudCompare::get_changed_points(PointCloudXYZI::Ptr &in_cloud, PointCloudXYZI::Ptr &out_cloud)
{
        std::vector<int> newPointIdxVector;

        if(!inited)
        {
                octree_detect->setInputCloud(in_cloud);
                octree_detect->addPointsFromInputCloud ();
                inited = true;
                return;
        }

        octree_detect->switchBuffers ();
        octree_detect->setInputCloud(in_cloud);
        octree_detect->addPointsFromInputCloud ();

        octree_detect->getPointIndicesFromNewVoxels (newPointIdxVector);
        
        pcl::copyPointCloud(*in_cloud, newPointIdxVector, *out_cloud);

}

void NodeCloudCompare::get_collection_changed_points(PointCloudXYZI::Ptr &in_cloud, PointCloudXYZI::Ptr &out_cloud)
{
	// 创建KdTreee对象作为搜索方法
	pcl::search::KdTree<PointType>::Ptr in_kdtree(new pcl::search::KdTree<PointType>);
        if(!in_cloud->empty()) in_kdtree->setInputCloud(in_cloud);

        // 创建KdTreee对象作为搜索方法
	pcl::search::KdTree<PointType>::Ptr last_kdtree(new pcl::search::KdTree<PointType>);
        if(!last_cloud->empty()) last_kdtree->setInputCloud(last_cloud);

        // 创建KdTreee对象作为搜索方法
	pcl::search::KdTree<PointType>::Ptr curr_kdtree(new pcl::search::KdTree<PointType>);
        if(!curr_cloud->empty()) curr_kdtree->setInputCloud(curr_cloud);
 
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(config.clusterTolerance); // 2cm
	ec.setMinClusterSize(config.minClusterSize);
	ec.setMaxClusterSize(config.maxClusterSize);
	ec.setSearchMethod(in_kdtree);
	ec.setInputCloud(in_cloud);
	//聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
	ec.extract(cluster_indices);

        pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		//通过下标，逐个填充
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                {
                        out_cloud->points.push_back(in_cloud->points[*pit]);
                }

	}

        //设置点云属性
        out_cloud->width = out_cloud->points.size();
        out_cloud->height = 1;
        out_cloud->is_dense = true;

        if(0)
        {
                PointType point = in_cloud->points[1];
                std::vector<int> indices;
                std::vector<float> distances;
                
                int num=last_kdtree->radiusSearch(point, config.radius, indices, distances);
                
                for(PointCloudXYZI::iterator it=cloud_cluster->begin();it!=cloud_cluster->end();it++)
                {
                        if( num >10 && num<200)
                        {
                                for(int i=0;i<indices.size();i++)
                                {
                                        PointType point = last_cloud->points[indices[i]];
                                        out_cloud->push_back(point);
                                }

                        }
                        else
                        {
                                continue;
                        }

                }
        }

}



