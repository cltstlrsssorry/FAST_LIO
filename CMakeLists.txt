cmake_minimum_required(VERSION 3.8) # 设置CMake的最低版本要求
project(fast_lio) # 为项目设置名称


if(NOT CMAKE_BUILD_TYPE) # 如果编译类型未设置
  set(CMAKE_BUILD_TYPE Release) # 则设置为Release类型
endif()

ADD_COMPILE_OPTIONS(-std=c++14) # 为编译添加C++14标准选项
set(CMAKE_CXX_FLAGS "-std=c++14 -O3") # 设置C++编译flags

cmake_policy(SET CMP0074 NEW) 

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions") # 为C编译添加异常处理选项
set(CMAKE_CXX_STANDARD 14) # 设置C++标准为14
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 设置C++标准为必需
set(CMAKE_CXX_EXTENSIONS OFF) # 关闭C++扩展
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions") # 为C++编译添加更多选项
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 使代码独立于位置

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}") # 输出当前CPU架构

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)") # 如果CPU架构匹配特定条件
  include(ProcessorCount) # 包含用于获取CPU核心数量的宏
  ProcessorCount(N) # 定义一个名为N的变量，用于存储CPU核心数量
  message("Processer number:  ${N}") # 输出CPU核心数量

  if(N GREATER 4) # 如果核心数量大于4
    add_definitions(-DMP_EN) # 添加一个名为MP_EN的宏定义
    add_definitions(-DMP_PROC_NUM=3) # 添加一个名为MP_PROC_NUM的宏定义，值为3
    message("core for MP: 3") # 输出用于多进程的CPU核心数量
  elseif(N GREATER 3) # 如果核心数量大于3
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else() # 如果核心数量小于等于3
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else() # 如果CPU架构不匹配特定条件
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP REQUIRED) # 查找OpenMP库
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # 为C++编译添加OpenMP选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}") # 为C编译添加OpenMP选项

find_package(PythonLibs REQUIRED) # 查找Python库
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h") # 查找matplotlibcpp.h文件

# ROS dependencies  查找ROS依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(octomap REQUIRED) # 查找octomap库
find_package(octomap_msgs REQUIRED) # 查找octomap_msgs库

# 设置依赖项
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  octomap_msgs
  octomap
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED) # 查找Eigen库
find_package(PCL REQUIRED COMPONENTS common io) # 查找PCL库

message(Eigen: ${EIGEN3_INCLUDE_DIR}) # 输出Eigen包含路径
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}") # 输出PCL包含路径

# 设置消息文件
set(msg_files
  "msg/Pose6D.msg"
)

set(ROOT_DIR ${CMAKE_SOURCE_DIR})
# add_definitions(-D ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}//") # 为项目添加宏定义
add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
# add_definitions(-D ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/") # 为项目添加宏定义
# add_definitions(-DROOT_DIR="${ROOT_DIR}")

# 为项目生成ROS消息接口
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
# 导出依赖项
ament_export_dependencies(rosidl_default_runtime)

# 添加可执行文件fastlio_mapping
add_executable(fastlio_mapping

  include/common_lib.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/fastlio.cpp 
  include/use-ikfom.cpp

  src/IMU_Processing.cpp
  src/preprocess.cpp
  
  # src/ERASOR/ERASOR.cpp
  # src/segment/pointsCorrect.cpp 
  # src/segment/segment.cpp

  src/NodePreProcess.cpp 
  src/NodeSegment.cpp
  # src/NodeERASOR.cpp
  src/NodeOctomap.cpp
  src/NodeFilterDynamic.cpp
  src/NodePCA.cpp
  src/NodeCloudCompare.cpp
  src/NodeReconstruction.cpp

  src/Run.cpp
  
)



# 设置fastlio_mapping的公共头文件目录
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# 设置PCL链接库
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} Eigen3::Eigen)
# 设置fastlio_mapping的私有头文件目录
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal") # 添加一个名为EOL_LIST的列表，用于支持多个ROS发行版

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Custommsg to support foxy & galactic
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()