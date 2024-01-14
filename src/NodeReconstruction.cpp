#include "NodeReconstruction.h"

NodeReconstruction::NodeReconstruction(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "----NodeFilterDynamic----");

    raw_map_ptr_.reset(new PointCloudXYZI());
    cloud_smoothed.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_normal.reset(new pcl::PointCloud<pcl::Normal>());

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->getRenderWindow()->GlobalWarningDisplayOff (); // Add This Line

    getConfig();

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(10000));

    timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodeReconstruction::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodeFilterDynamic init finished.----"); // 输出信息，初始化结束

}

void NodeReconstruction::timer_callback()
{
    if(filter_dynamic_map_triangle.empty())
        return;
    
    raw_map_ptr_ = filter_dynamic_map_triangle.begin()->points;
    timestamp = filter_dynamic_map_triangle.begin()->time;
    filter_dynamic_map_triangle.pop_front();

    CloudSmoothed(raw_map_ptr_, cloud_smoothed);
    CloudNormals(raw_map_ptr_, cloud_normal);

    // Triangle Projection
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(cloud_smoothed, cloud_normal, cloud_with_normals);

    CloudTriangle(cloud_with_normals);

    viewer->addPolygonMesh(triangles, "my");
    viewer->spinOnce(300);
    viewer->removePointCloud();// 可视化结束后清除点云数据

}

void NodeReconstruction::getConfig()
{

}

void NodeReconstruction::CloudSmoothed(PointCloudXYZI::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &smoothed)
{
    pcl::search::KdTree<PointType>::Ptr treeSampling (new pcl::search::KdTree<PointType>);
    pcl::PointCloud<pcl::PointXYZ> mls_point;
    pcl::MovingLeastSquares<PointType, pcl::PointXYZ> mls;
    mls.setComputeNormals(false);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(treeSampling);
    mls.setSearchRadius(0.05);
    mls.process(mls_point);
    // 输出重采样结果
    smoothed = mls_point.makeShared();

}

void NodeReconstruction::CloudNormals(PointCloudXYZI::Ptr &cloud,pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // 法线估计
    pcl::NormalEstimation<PointType,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    normalEstimation.setSearchMethod(tree);

    normalEstimation.setKSearch(10);
    normalEstimation.compute(*normals);
}


void NodeReconstruction::concatenateFields(const pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz,
                                           const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals)
{
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


void NodeReconstruction::CloudTriangle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud);
 
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
 
	gp3.setSearchRadius(0.1);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(52);
 
	gp3.setMaximumAngle(2*M_PI/3);
	gp3.setMinimumAngle(M_PI/18);
 
	gp3.setMaximumSurfaceAngle(M_PI/4);
	gp3.setNormalConsistency(false);
 
	gp3.setInputCloud(cloud);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

    std::vector<int> states = gp3.getPointStates();
}