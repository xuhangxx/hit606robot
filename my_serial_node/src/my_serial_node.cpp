#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <tf/transform_broadcaster.h>
#define	sBUFFERSIZE	15//send buffer size 串口发送缓存长度
#define	rBUFFERSIZE	58//receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

geometry_msgs::Quaternion quat;
const double pi = 3.1415926535898;
double num;
//联合体，用于浮点数与16进制的快速转换
int p=0;
int flag=0;
serial::Serial ser;
int pp=0;
sensor_msgs::Imu imu_data;
int main (int argc, char** argv)
{
    ros::init(argc, argv, "serial_GPS_IMU_node");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key

	//ros::Subscriber write_sub = nh.subscribe("/turtle1/cmd_vel",1000,cmd_vel_callback);

	//发布位置和姿态话题  position  GPS位置信息？ IMU姿态信息？
	//ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);

    //定义GPS数据消息发布
    ros::Publisher GPS_pub = nh.advertise<sensor_msgs::NavSatFix>("GPS_data", 1);
	//定义IMU数据消息发布
	ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 1);
    //定义速度数据消息发布
	//ros::Publisher CmdVel_pub = nh.advertise<geometry_msgs::Twist>("CmdVel_data", 20);
    

    nh_private.param<double>("num",  num, 15);
    
	sensor_msgs::NavSatFix gps_data;
	gps_data.header.frame_id = "base_link";
	gps_data.position_covariance_type=1;
    gps_data.status.service=1;
	//串口数据接收
    try
    {
        ser.setPort("/dev/gps");
        ser.setBaudrate(230400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port_0 ");
        return -1;
    }

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("Serial Port_0 initialized");
    }
	else
	{
        return -1;
    }

    //double i=0;
    //double j=0;
    //10hz频率执行
    ros::Rate loop_rate(100);

    while(ros::ok())
	{
        //imu_data.header.stamp = ros::Time::now();
        if(ser.available())
		{

            //ROS_INFO_STREAM("Reading from serial port_0");
			ser.read(r_buffer,rBUFFERSIZE);
           	
			int a;
				
            uint8_t datatype,checksum,DataLen;
	        int32_t buf;
            DataLen = 58;
            checksum = 0;
			
		    for(int i=0;i<rBUFFERSIZE;i++)
		    {
		        
				if (r_buffer[i] == 0xBD && r_buffer[i+1] == 0xDB && r_buffer[i+2] == 0x0B )
				{

				 	a=i;
					if(a!=0)
					{
						ser.read(r_buffer,i);
						//std::cout<<"dddddddddddddddddddddddddddddddddddddddd"<<a<<std::endl;
					}
					else
					{
						//std::cout<<"        "<<a<<std::endl;
                 		//校验位
                 		for (int z=a ; z < 57+a; z++)
                  		{
                    		checksum ^= r_buffer[z];    //异或运算校验字节的正确
                  		}

                 		//ROS_INFO("Checksum:0x%02x",checksum);
				  
                 		if (r_buffer[57+a] == checksum)  //校验位正确
                 		{
                    		imu_data.header.stamp = ros::Time::now();
                    		imu_data.header.frame_id = "base_link";

							double Roll,Pitch,heading;
							buf = (int16_t)(r_buffer[a+3] + (r_buffer[a+4] << 8));
                    		Roll = ((double)buf) * 360 / 32768;      //解析横滚角
 
                    		buf = (int16_t)(r_buffer[a+5] + (r_buffer[a+6] << 8));
                    		Pitch = ((double)buf) * 360 / 32768;     //解析俯仰角

                    		buf = (int16_t)(r_buffer[a+7] + (r_buffer[a+8] << 8));
                    		heading = ((double)buf) * 360 / 32768;   //解析偏航角
                    		if(r_buffer[a+56] == 0x20)
                    		{
                         		buf = (int16_t)(r_buffer[a+48]+ (r_buffer[a+49] << 8));
                         		p=buf;
								//std::cout<<" GPS                 "<<p<<std::endl;
								//std::cout<<" GPS2                 "<<(int16_t)(r_buffer[a+50]+ (r_buffer[a+51] << 8))<<std::endl;
                    		}
						
                        
                        	quat = tf::createQuaternionMsgFromRollPitchYaw(Roll*pi/180, Pitch*pi/180, heading*pi/180);
							imu_data.orientation=quat;
                    		//std::cout<<(int16_t)(r_buffer[a+56])<<std::endl;
                    		//q.setRPY(Roll,Pitch,heading);
							//imu_data.orientation.x=Roll;
							//imu_data.orientation.y=Pitch;
							imu_data.orientation.z=heading;
                    		//imu_data.orientation.w=p;
							//imu_data.orientation.w=q[3];

							buf = (int16_t)(r_buffer[a+9] + (r_buffer[a+10] << 8));
                     		imu_data.angular_velocity.x = ((float)buf) * 300 / 32768;        //解析x轴角速度

                     		buf = (int16_t)(r_buffer[a+11] + (r_buffer[a+12] << 8));
                     		imu_data.angular_velocity.y = ((float)buf) * 300 / 32768;        //解析y轴角速度

                     		buf = (int16_t)(r_buffer[a+13] + (r_buffer[a+14] << 8));
                     		imu_data.angular_velocity.z = ((float)buf) * 300 / 32768;        //解析z轴角速度

                     		buf = (int16_t)(r_buffer[a+15] + (r_buffer[a+16] << 8));
                     		imu_data.linear_acceleration.x = ((float)buf) * 12 / 32768;          //解析x轴加速度

                     		buf = (int16_t)(r_buffer[a+17] + (r_buffer[a+18] << 8));
                     		imu_data.linear_acceleration.y = ((float)buf) * 12 / 32768;          //解析y轴加速度
   
                     		buf = (int16_t)(r_buffer[a+19] + (r_buffer[a+20] << 8));
                     		imu_data.linear_acceleration.z = ((float)buf) * 12 / 32768;          //解析z轴加速度

                     		IMU_pub.publish(imu_data);  //IMU数据发布
                        	// std::cout<<"sssss"<<imu_data.header.stamp<<std::endl;
                        	buf = (int32_t)(r_buffer[a+21] + (r_buffer[a+22] << 8) + (r_buffer[a+23] << 16) + (r_buffer[a+24] << 24));
                    		//定义GPS的数据消息对象
							//std::cout<<"       ds dsdsdsdds"<<((double)buf) / 1e7f<<std::endl;
                       		if(p>num && ((double)buf) / 1e7f > 10)
							{
								flag=1;
							}
							else
							{
								gps_data.latitude = 0;
								gps_data.longitude = 0; 
                     			gps_data.altitude = 0;   
								gps_data.status.status=-1;  
								gps_data.position_covariance[0]=1e9;
								gps_data.position_covariance[4]=1e9;
								gps_data.position_covariance[8]=1e9;
								gps_data.header.stamp =ros::Time::now();

							}
							//{
								//gps_data.latitude = 31;
								//gps_data.longitude = 130; 
                     			//gps_data.altitude = 0;   
								//gps_data.status.status=-1;            
							//}
							//else
							//{
							if(flag)
							{

								gps_data.header.stamp =ros::Time::now();
								gps_data.status.status=1; 
                     			gps_data.latitude = ((double)buf) / 1e7f;              //解析纬度  
                     			//gps_data.latitude=45.7458007+i;


                     			buf = (int32_t)(r_buffer[a+25] + (r_buffer[a+26] << 8) + (r_buffer[a+27] << 16) + (r_buffer[a+28] << 24));
                     			gps_data.longitude = ((double)buf) / 1e7f;               //解析经度
                     			//gps_data.longitude=126.6261953+j;
                     			//i=i-0.00000000008;
	                 			//j=j-0.000000000009;

                     			buf = (int32_t)(r_buffer[a+29] + (r_buffer[a+30] << 8) + (r_buffer[a+31] << 16) + (r_buffer[a+32] << 24));
                     			gps_data.altitude = ((float)buf) / 1e3f;                //解析高度
								if(r_buffer[a+56] == 0x00)
                    			{
                         			buf = (int16_t)(r_buffer[a+46]+ (r_buffer[a+47] << 8));
									//std::cout<<"buf1"<<buf<<std::endl;
									gps_data.position_covariance[0]=pow(2.718281,((double)buf*0.02));
									buf = (int16_t)(r_buffer[a+48]+ (r_buffer[a+49] << 8));
									//std::cout<<"buf2"<<buf<<std::endl;
                         			gps_data.position_covariance[4]=pow(2.718281,((double)buf*0.02));
									buf = (int16_t)(r_buffer[a+50]+ (r_buffer[a+51] << 8));
									//std::cout<<"buf3"<<buf<<std::endl;
                         			gps_data.position_covariance[8]=pow(2.718281,((double)buf*0.02));
                    			}
							}

		             		GPS_pub.publish(gps_data);   //GPS数据发布
        

	                    	//定义速度数据消息对象

           					//geometry_msgs::Twist cmdvel_data;

           					//buf = (int16_t)(r_buffer[a+33] + (r_buffer[a+34] << 8));
           					//cmdvel_data.linear.x= ((float)buf) * 100/ 32768;          //解析北向速度   
           					//buf = (int16_t)(r_buffer[a+35] + (r_buffer[a+36] << 8));
           					//cmdvel_data.linear.y= ((float)buf) * 100/ 32768;          //解析东向速度
           					//buf = (int16_t)(r_buffer[a+37] + (r_buffer[a+38] << 8));
           					//cmdvel_data.linear.z = ((float)buf) * 100/ 32768;          //解析地向速度
            
           					//CmdVel_pub.publish(cmdvel_data);   //速度数据发布  
				 		}
						else
						{
						std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<a<<std::endl;
						}
					}   
					
			  	}
		    } 

			memset(r_buffer,0,rBUFFERSIZE);
        }


		//gps_data.header.stamp =ros::Time::now();
       // std::cout<<"^4444444444444444444444444444444444444444444"<<gps_data.header.stamp-imu_data.header.stamp<<std::endl;
        loop_rate.sleep();

    }
}


