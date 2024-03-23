#include "NodeReconstruction.h"

NodeReconstruction::NodeReconstruction(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "----NodeReconstruction init finished.----"); // 输出信息，初始化结束

    raw_map_reconstruction_ptr_.reset(new PointCloudXYZI());

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    timer_reconstruction_ = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodeReconstruction::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodeReconstruction init finished.----"); // 输出信息，初始化结束
}

// 定时器回调函数
void NodeReconstruction::timer_callback()
{
    sub_reconstruction = this->create_subscription<sensor_msgs::msg::PointCloud2>("/publish_filter_dynamic_map", 20, std::bind(&NodeReconstruction::cloud_callback, this, std::placeholders::_1));

    if (raw_map_reconstruction_ptr_->empty())
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());

    CloudSmoothed(raw_map_reconstruction_ptr_, cloud_smoothed);

    CloudNormals(cloud_smoothed, cloud_normal);

    // Triangle Projection
    ConcatenateFields(cloud_smoothed, cloud_normal, cloud_with_normals);

    CloudTriangle(cloud_with_normals);
    
}

// 点云数据回调函数
void NodeReconstruction::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    timestamp = msg->header.stamp;
    pcl::fromROSMsg(*msg, *raw_map_reconstruction_ptr_);
}

// 点云平滑
void NodeReconstruction::CloudSmoothed(PointCloudXYZI::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &smoothed)
{
    if (cloud->empty())
        return;

    pcl::search::KdTree<PointType>::Ptr treeSampling(new pcl::search::KdTree<PointType>);

    pcl::PointCloud<pcl::PointXYZ> mls_point;

    pcl::MovingLeastSquares<PointType, pcl::PointXYZ> mls;
    
    mls.setComputeNormals(false);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(treeSampling);
    mls.setSearchRadius(0.05);
    mls.process(mls_point);
    // 输出重采样结果
    smoothed = mls_point.makeShared();

}

// 点云法线估计
void NodeReconstruction::CloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{

    if (cloud->empty())
        return;

    // 法线估计
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(tree);

    normalEstimation.setKSearch(10);
    normalEstimation.compute(*normals);

}

// 点云数据合并
void NodeReconstruction::ConcatenateFields(const pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz,
                                           const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals)
{

    if (xyz->empty() || normals->empty())
        return;

    cloud_with_normals->resize(xyz->size());
    for (size_t i = 0; i < xyz->size(); i++)
    {
        cloud_with_normals->points[i].x = xyz->points[i].x;
        cloud_with_normals->points[i].y = xyz->points[i].y;
        cloud_with_normals->points[i].z = xyz->points[i].z;
        cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
        cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
        cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
    }

}

// 点云三角化
void NodeReconstruction::CloudTriangle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
    if (cloud->empty())
        return;

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    
    tree2->setInputCloud(cloud);

    pcl::PolygonMesh triangle;

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    gp3.setSearchRadius(0.1);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(400);

    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumSurfaceAngle(M_PI / 4);

    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangle);
    
    // std::string filename = "/home/hl/meshlog/mesh" + std::to_string(count) + ".ply";

    // pcl::io::savePLYFile(filename, triangles);

    // std::vector<int> states = gp3.getPointStates();

}