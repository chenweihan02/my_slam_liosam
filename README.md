# my_slam_liosam
手写LIO-SAM

---

- [ ] 验证imageProjection去除畸变的效果
- [ ] 聚类
- [ ] mapOptmization

---

- [x] imageProjection
- [x] featureExtraction 

---

参考：[smilefacehh/LIO-SAM-DetailedNote](https://github.com/smilefacehh/LIO-SAM-DetailedNote)

---

http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html
http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html
http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

补充内容：

sensor_msgs/PointCloud2.msg
```
std_msgs/Header     header        帧头信息
uint32              height        点云高度，1表示二维点云，大于1表示三维点云
uint32              width         点云宽度，表示每行点云的个数
sensor_msgs/PointField[] fields   点云中每个字段的定义
bool                is_bigendian  数据是否为大端存储
uint32              point_step    单个点的字节数
uint32              row_step      一行点云数据的字节数
uint8[]             data          点云数据，连续存储的一段二进制数据
bool                is_dense
```

sensor_msgs/LaserScan.msg
```
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());


- nh：ROS节点句柄（NodeHandle）对象，用于创建ROS节点。
- subscribe<sensor_msgs::Imu>：订阅者对象的类型，表示要订阅的消息类型是sensor_msgs::Imu，即IMU消息。
- imuTopic：要订阅的IMU消息话题的名称。
- 2000：消息队列的长度，表示ROS节点可以缓存的未处理消息数量。
- &ImageProjection::imuHandler：IMU消息的回调函数，在接收到IMU消息后会自动调用该函数进行处理。
- this：当前对象的指针，用于在回调函数中访问ImageProjection类的成员变量和成员函数。
- ros::TransportHints().tcpNoDelay()：用于设置ROS通信的传输选项，这里设置了TCP无延迟（tcpNoDelay），以便实现更高效的数据传输。

---
TODO: cpp lock用法

---

http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
sensor_msgs/Imu

std_msgs/Header header                                消息头部，包含了时间戳和消息的frame_id等信息。

geometry_msgs/Quaternion orientation                  表示IMU在参考坐标系中的姿态（方向），通常用四元数来表示。
float64[9] orientation_covariance                     表示IMU姿态的协方差矩阵，是一个9元素的一维数组，按照行优先的方式存储。

geometry_msgs/Vector3 angular_velocity                表示IMU在参考坐标系中的角速度，用三维向量表示。
float64[9] angular_velocity_covariance                表示IMU角速度的协方差矩阵，是一个9元素的一维数组，按照行优先的方式存储。

geometry_msgs/Vector3 linear_acceleration             表示IMU在参考坐标系中的线性加速度，用三维向量表示。
float64[9] linear_acceleration_covariance             表示IMU线性加速度的协方差矩阵，是一个9元素的一维数组，按照行优先的方式存储。


其中，姿态、角速度和线性加速度是IMU最基本的测量参数，协方差矩阵则表示了测量参数的精度和可信度，通常用于进行数据融合和滤波等处理。需要注意的是，协方差矩阵通常是对角矩阵，表示各个测量参数之间是独立的。

IMU模块中6轴、9轴包含哪些模块？：https://blog.csdn.net/ybhuangfugui/article/details/118004677

在ROS中，6轴IMU包含三轴(XYZ)加速度计和三轴(XYZ)陀螺仪，
而9轴IMU包含6轴IMU和三轴(XYZ)磁场传感器。
6轴IMU可以构成垂直参考单元(VRU)和惯性测量单元(IMU)，
而9轴IMU可以构成航姿参考系统(AHRS) IMU。

---
 pcl::PointCloud<PointXYZIRT>

```
struct PointXYZIRT
{
  PCL_ADD_POINT4D;                  宏定义，将x,y,z,pad合并成Eigen向量
  float intensity;                  该点的反射强度
  uint16_t ring;                    该点所在的环号
  double time;                      该点信息采集时间戳
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   宏定义，对齐内存，提高数据的读写效率
} EIGEN_ALIGN16;
```

https://blog.csdn.net/qfpkzheng/article/details/120394346
Ring: 在激光雷达中，Ring 是指激光束的序号，从 0 开始递增。例如，Velodyne HDL-64E 激光雷达有 64 束激光，每束激光的序号从 0 到 63。因此，每个点都有一个 Ring 值，表示该点是哪一束激光返回的。Ring 值可以用于计算激光束的高度角度和水平角度

---

https://zhuanlan.zhihu.com/p/103700110
https://www.cnblogs.com/li-yao7758258/p/6651326.html
pcl::moveFromROSMsg 将 ROS 消息类型 sensor_msgs::PointCloud2 转换为 PCL 数据类型 pcl::PointClou

---


LIO-SAM 是一种激光雷达和惯性测量单元（IMU）紧耦合的算法，它可以通过平滑与建图实现高精度、实时的移动机器人轨迹估计和地图构建。在 LIO-SAM 中，imuDeskewInfo 函数的作用是对 IMU 数据进行去畸变处理。具体来说，它对 IMU 数据进行了预积分，并将其与激光雷达数据进行了紧耦合优化。这个函数的实现可以在 LIO-SAM 的源代码中找到。

特征提取：提取角点和面点，用于去除运动畸变、定位和建图？

https://blog.csdn.net/JaydenQ/article/details/119903251
https://blog.csdn.net/zkk9527/article/details/117957067
https://blog.csdn.net/weixin_42141088/article/details/118000544

boost？


TODO: CMakeLists区别