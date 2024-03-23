#include "NodePCA.h"

NodePCA::NodePCA(const std::string & name):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{

    RCLCPP_INFO(this->get_logger(), "----NodePCA----");

    pca_cloud_ptr_.reset(new PointCloudXYZI());

    setConfig();

    pubLasercentroid = this->create_publisher<sensor_msgs::msg::PointCloud2>("/publish_conteroid", 20);
    pubLaserPCA_X = this->create_publisher<geometry_msgs::msg::PoseStamped>("/publish_pca_X", 20);
    pubLaserPCA_Y = this->create_publisher<geometry_msgs::msg::PoseStamped>("/publish_pca_Y", 20);
    pubLaserPCA_Z = this->create_publisher<geometry_msgs::msg::PoseStamped>("/publish_pca_Z", 20);

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodePCA::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodePCA init finished.----"); // 输出信息，初始化结束
}

void NodePCA::timer_callback()
{
    sub_pca=this->create_subscription<sensor_msgs::msg::PointCloud2>("/publish_filter_dynamic_map", 20, std::bind(&NodePCA::cloud_callback, this, std::placeholders::_1));

    if(pca_cloud_ptr_->empty()) return;

    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*pca_cloud_ptr_, pcaCentroid);//计算点云质心
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*pca_cloud_ptr_, pcaCentroid, covariance);//计算目标点云协方差矩阵

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);//构造一个特定的自伴随矩阵类便于后续分解
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//计算特征向量
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//计算特征值
    //eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
    //eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    //eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 


    PointCloudXYZI centeroid;
    PointType centeroid_point;
	Eigen::Vector3f c;
	c(0) = pcaCentroid(0);
	c(1) = pcaCentroid(1);
	c(2) = pcaCentroid(2);
    centeroid_point.x = c(0);
    centeroid_point.y = c(1);
    centeroid_point.z = c(2);
    centeroid.push_back(centeroid_point);

	Eigen::Vector3f pcX;
	pcX(0) = 1000 * eigenVectorsPCA(0, 0) + c(0);
	pcX(1) = 1000 * eigenVectorsPCA(1, 0) + c(1);
	pcX(2) = 1000 * eigenVectorsPCA(2, 0) + c(2);
    VectorToQuaternionf(c, pcX, pubLaserPCA_X);

	Eigen::Vector3f pcY;
	pcY(0) = 1000 * eigenVectorsPCA(0, 1) + c(0);
	pcY(1) = 1000 * eigenVectorsPCA(1, 1) + c(1);
	pcY(2) = 1000 * eigenVectorsPCA(2, 1) + c(2);
    VectorToQuaternionf(c, pcY, pubLaserPCA_Y);

	Eigen::Vector3f pcZ;
	pcZ(0) = 1000 * eigenVectorsPCA(0, 2) + c(0);
	pcZ(1) = 1000 * eigenVectorsPCA(1, 2) + c(1);
	pcZ(2) = 1000 * eigenVectorsPCA(2, 2) + c(2);
    VectorToQuaternionf(c, pcZ, pubLaserPCA_Z);

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(centeroid, laserCloudmsg);
    laserCloudmsg.header.stamp = timestamp;
    laserCloudmsg.header.frame_id = "camera_init";
    pubLasercentroid->publish(laserCloudmsg);
}

void NodePCA::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    timestamp = msg->header.stamp;
    pcl::fromROSMsg(*msg, *pca_cloud_ptr_);
}

void NodePCA::setConfig()
{
    this->declare_parameter<bool>("PCA.publish_map_en", true);

    this->get_parameter_or<bool>("PCA.publish_map_en", publish_map_en, true);

}

void NodePCA::VectorToQuaternionf(Eigen::Vector3f &pos, 
                                    Eigen::Vector3f &vector,
                                    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLaserPCA)
{
    geometry_msgs::msg::PoseStamped::SharedPtr pca_pose(new geometry_msgs::msg::PoseStamped);
    pca_pose->header.stamp = timestamp;
    pca_pose->header.frame_id = "camera_init";

    float rxx = vector(0) * PI / 180;
    float ryy = vector(1) * PI / 180;
    float rzz = vector(2) * PI / 180;

    float w_abs = sqrt(rxx * rxx + ryy * ryy + rzz * rzz);

    float q_w = cos(w_abs / 2);
    float q_x = sin(w_abs / 2) * rxx / w_abs;
    float q_y = sin(w_abs / 2) * ryy / w_abs;
    float q_z = sin(w_abs / 2) * rzz / w_abs;

    pca_pose->pose.position.x = pos(0);
    pca_pose->pose.position.y = pos(1);
    pca_pose->pose.position.z = pos(2);
    pca_pose->pose.orientation.x = q_x;
    pca_pose->pose.orientation.y = q_y;
    pca_pose->pose.orientation.z = q_z;
    pca_pose->pose.orientation.w = q_w;

    pubLaserPCA->publish(*pca_pose);

}




