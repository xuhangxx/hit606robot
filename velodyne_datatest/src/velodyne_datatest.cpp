#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <velodyne_datatest/velodyne_datatest.h>
#include <velodyne_datatest/velodyne_obstacle.h> //自定义消息类型自动生成的头文件

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI PointType;

#include <vector>
#include <serial/serial.h>
#include <std_msgs/String.h> 


#define	sBUFFERSIZE	14//send buffer size 串口发送缓存长度 
#define	rBUFFERSIZE	11//receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];
unsigned char r_buffer[rBUFFERSIZE];
serial::Serial ser;
namespace velodyne_datatest
{
  
  VelodyneDataTest::VelodyneDataTest(ros::NodeHandle &nh) : nh_(nh)
  {
    /*try
    {
        ser.setPort("/dev/car");  //串口驱动号需要修改
        ser.setBaudrate(115200);     //波特率设置
        serial::Timeout to = serial::Timeout::simpleTimeout(25);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port_1 ");
    }

    if(ser.isOpen())
	  {
        ROS_INFO_STREAM("Serial Port_1 initialized");
    }*/

   subLaserCloud= nh_.subscribe("velodyne_points", 10, &VelodyneDataTest::laserCloudHandler,this);
   std::cout<<"Subscribe：/velodyne_points"<<std::endl;

   //pubLaserScan= nh_.advertise<sensor_msgs::LaserScan>("scan2", 10);
   //std::cout<<"Publisher：/scan2"<<std::endl;    

   pubVelodyneObstacle= nh_.advertise<velodyne_datatest::velodyne_obstacle>("Velodyne_obstacle", 10);
   std::cout<<"Publisher：/Velodyne_obstacle"<<std::endl; 

   //pubLaserCloud= nh_.advertise<sensor_msgs::PointCloud2>("velodyne_points2", 10);
   //std::cout<<"Publisher：/velodyne_points2"<<std::endl;
  }

  

 //激光雷达点云处理模块回调函数转换为laserscan二维数据
  void VelodyneDataTest::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
  {  
    /***************************************************************************************/
    //velodyne_points数据转换为laserscan的scan数据
    const float RESOLUTION = 0.007;//分辨率2PI/2016  0.001~0.5
    const size_t SIZE = 2016;  //一圈2016个点

    //sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
    velodyne_datatest::velodyne_obstacle obs_;

    //scan->header = laserCloudMsg->header;
    //scan->angle_increment = RESOLUTION;
    //scan->angle_min = -M_PI;
    //scan->angle_max = M_PI;
    //scan->range_min = 0.0;
    //scan->range_max = 200.0;
    //scan->time_increment = 0.0;
    //scan->ranges.resize(SIZE, INFINITY);
    //scan->intensities.resize(SIZE);

    for (sensor_msgs::PointCloud2ConstIterator<float> it(*laserCloudMsg, "x"); it != it.end(); ++it)
    {   
          //把对应域中的的变量解析存放在数组中
          const uint16_t r = *((const uint16_t*)(&it[4]));  // ring 4
          const float x = it[0];  // x
          const float y = it[1];  // y
          //const float i = it[3];  // intensity 3
          const float yaw_angle=atan2f(y, x);//激光雷达坐标系向前为y，右为x，上为z;
          const float a=(yaw_angle*180)/M_PI; //弧度制转换为角度
          const float distance=sqrtf(x * x + y * y);
          if (r==13||r==14||r==15)     //判断环数 确定当前点属于那个水平激光柱
          {  
            const int bin = (yaw_angle + static_cast<float>(M_PI)) / RESOLUTION;   //加PI的原因是因为从X负半轴起始值
            if ((bin >= 0) && (bin < static_cast<int>(SIZE)))
            {
              if(yaw_angle>=(-M_PI/3)&&yaw_angle<=(M_PI/3))    //设置可视化区域的角度范围 18
              { 
                
                if (distance>=0&&distance<=0.8)                  //障碍物点距离激光雷达坐标系远点的距离
                { 
                  //std::cout<<"xxxx"<<x<<std::endl;
                  //std::cout<<"yyyy"<<y<<std::endl;
                  //ROS_INFO("sending data distance response: [%f]", distance);
                  //ROS_INFO("sending data yaw_angle response: [%f]", a); 
                  //scan->ranges[bin] = distance;    
                  //scan->intensities[bin] = i;  //当前点数组中存储的强度值 障碍物点的显示
                
                  obs_.distance= distance;
                  obs_.yaw_angle= a;     
                  pubVelodyneObstacle.publish(obs_);
                  //std::cout<<" Velodyne_Obstacle run"<<std::endl;
	                /*memset(s_buffer,0,sizeof(s_buffer));
                  s_buffer[0]='!';
                  s_buffer[1]='M';
                  s_buffer[2]=' ';
                  s_buffer[3]='0';
                  s_buffer[4]=' ';
                  s_buffer[5]='0';
	                ser.write(s_buffer,sBUFFERSIZE);
	                ros::Duration(0.03).sleep();
                  memset(r_buffer,0,rBUFFERSIZE);
	                ser.read(r_buffer,rBUFFERSIZE);
                  std::cout<<r_buffer[0]<<std::endl;
                  std::cout<<r_buffer[1]<<std::endl;
                  std::cout<<r_buffer[2]<<std::endl;
                  std::cout<<r_buffer[3]<<std::endl;
                  std::cout<<r_buffer[4]<<std::endl;
                  std::cout<<r_buffer[5]<<std::endl;
                  std::cout<<r_buffer[6]<<std::endl;
                  std::cout<<r_buffer[7]<<std::endl;
                  std::cout<<r_buffer[8]<<std::endl;
                  std::cout<<r_buffer[9]<<std::endl;
                  std::cout<<r_buffer[10]<<std::endl;
                  std::cout<<"sdsdsdsdsddddddddddddddddddddddd"<<std::endl;*/

                }
              }
            }
          }

          //pubLaserScan.publish(scan);
          //std::cout<<" laserCloudHandler run"<<std::endl;
          /********************************ros转为pcl生成点云处理障碍物*****************************/
     
          /*const int N_SCANS = 16;
          std::vector<int> scanStartInd(N_SCANS, 0);
          std::vector<int> scanEndInd(N_SCANS, 0);
        
        // Lidar的时间戳
          double timeScanCur = laserCloudMsg->header.stamp.toSec();
          pcl::PointCloud<pcl::PointXYZ> laserCloudIn;  

        // fromROSmsg(input,cloud1) 转为为模板点云laserCloudIn
          pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
        
        //去除无效值
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        
          int cloudSize = laserCloudIn.points.size();
        
        //计算点云的起始角度/终止角度
          float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
          float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                        laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

          if (endOri - startOri > 3 * M_PI) 
          {
            endOri -= 2 * M_PI;
          } 
          else if (endOri - startOri < M_PI) 
          {
              endOri += 2 * M_PI;
          }*/

         //接下来的处理是根据角度将点划入不同数组中
          /*bool halfPassed = false;
          int count = cloudSize;
          PointType point;
          std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
          for (int i = 0; i < cloudSize; i++)
          {
            point.x = laserCloudIn.points[i].y;
            point.y = laserCloudIn.points[i].z;
            point.z = laserCloudIn.points[i].x;

            float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
            
            int scanID;
            int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5)); 
            if (roundedAngle > 0)
            {
                scanID = roundedAngle;
            }
            else
            {
                  // 角度大于0，由小到大划入偶数线0-16；角度小于0，由大到小划入奇数线15-1
                scanID = roundedAngle + (N_SCANS - 1);
            }

            if (scanID > (N_SCANS - 1) || scanID < 0 )
            {
                 // 不在16线附近的点作为杂点进行剔除
                   count--;
                   continue;
            }
           


          }*/

        

            
        /*
            sensor_msgs::PointCloud2 laserCloudOutMsg;
            pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
            laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
            laserCloudOutMsg.header.frame_id = "/velodyne";
            pubLaserCloud.publish(laserCloudOutMsg);
         */

    }


  }
}  




