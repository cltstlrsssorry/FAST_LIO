# include "NodeCloudProcess.h"

// 定义全局变量
std::deque<double> time_buffer;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;

NodeCloudProcess::NodeCloudProcess(const std::string & name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{

    cout << "----NodeCloudProcess----" << endl;
    // 声明和初始化一个参数
    this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<bool>("common.time_sync_en", false);
    this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);

    this->declare_parameter<double>("preprocess.blind", 0.01);
    this->declare_parameter<int>("preprocess.lidar_type", AVIA);
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.timestamp_unit", US);
    this->declare_parameter<int>("preprocess.scan_rate", 10);


    // 此命令将配置文件加载到ROS2参数服务器中。然后，您可以使用以下命令从参数服务器中获取参数的值：
    //  Get the parameter value, or the "alternative value" if not set, and assign it to "value".
    //  If the parameter was not set, then the "value" argument is assigned the "alternative_value". In all cases, the parameter remains not set after this function is called.

    this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
    this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
    this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
    this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);

    this->get_parameter_or<double>("preprocess.blind", preprocess->blind, 0.01);
    this->get_parameter_or<int>("preprocess.lidar_type", preprocess->lidar_type, AVIA);
    this->get_parameter_or<int>("preprocess.scan_line", preprocess->N_SCANS, 16);
    this->get_parameter_or<int>("preprocess.timestamp_unit", preprocess->time_unit, US);
    this->get_parameter_or<int>("preprocess.scan_rate", preprocess->SCAN_RATE, 10);

    // 其中FOV_DEG是激光雷达的全视场角度,HALF_FOV_COS是角度的一半对应的余弦值。这些值会在后续滤波或特征提取时用到,因此在初始化时先进行计算。而路径消息path的头信息是发布里程计结果时需要的总体上,这段代码主要是进行了一些初始化操作,计算参数并打印,为算法运行做准备。
    RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", preprocess->lidar_type); // 打印当前的激光雷达类型p_pre->lidar_type

    // 创建订阅器
    /*** ROS subscribe initialization ***/
    if (preprocess->lidar_type == AVIA)
    {
        sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, std::bind(&NodeCloudProcess::livox_pcl_cbk, this,std::placeholders::_1));
        // 数字 20 表示消息队列的最大长度，即最多可以缓存多少条消息。
    }
    else
    {
        sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 20, std::bind(&NodeCloudProcess::standard_pcl_cbk, this,std::placeholders::_1));
    }

    /**
     * 在 C++ 中，可以使用 create_subscription 方法来创建订阅器。
     * 例如，下面的代码创建了一个名为 subscription 的订阅器，它将订阅名为 imu_topic 的主题，消息类型为 sensor_msgs::msg::Imu，队列大小为 10，回调函数为 imu_cbk
     * */
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&NodeCloudProcess::imu_cbk, this,std::placeholders::_1)); // 10 表示消息队列的最大长度，即最多可以缓存多少条消息。

    RCLCPP_INFO(this->get_logger(), "NodeCloudProcess init finished."); // 输出信息，初始化结束
}

NodeCloudProcess::~NodeCloudProcess()
{

}

void NodeCloudProcess::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = fastlio::get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    preprocess->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void NodeCloudProcess::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    double cur_time = fastlio::get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    preprocess->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void NodeCloudProcess::imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = fastlio::get_ros_time(fastlio::get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =rclcpp::Time(timediff_lidar_wrt_imu + fastlio::get_time_sec(msg_in->header.stamp));
    }

    double timestamp = fastlio::get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}