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

/************************************
 * 串口数据发送格式共15字节
 * head head linear_v_x  linear_v_y angular_v  CRC
 * 0xff 0xff float       float      float      u8
 * **********************************/
/**********************************************************
 * 串口接收数据格式共58字节
 * head    head      head    roll   yaw   pitch   gX    gY   gZ     ax     ay      az   
 * 0xBD    0xDB      0x0B    I16    I16   I16     I16   I16  I16    I16    I16     I16           
 longitude latitude  high    vN     vE    vD     state   -   Date1  Date2  Date3   time  type  CRC
   I32     I32       I32     I16    I16   I16     U8     -   I16    I16    I16     U32    U8   U8

 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union;

serial::Serial ser;

/**********************************************************
 * 数据打包，将获取的cmd_vel信息打包并通过串口发送
 * ********************************************************/
void data_pack(const geometry_msgs::Twist& cmd_vel)
{
	//unsigned char i;
	float_union Vx,Vy,Ang_v;
	Vx.fvalue = cmd_vel.linear.x;
	Vy.fvalue = cmd_vel.linear.y;
	Ang_v.fvalue = cmd_vel.angular.z;
	
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	//Vx
	s_buffer[2] = Vx.cvalue[0];
	s_buffer[3] = Vx.cvalue[1];
	s_buffer[4] = Vx.cvalue[2];
	s_buffer[5] = Vx.cvalue[3];
	//Vy
	s_buffer[6] = Vy.cvalue[0];
	s_buffer[7] = Vy.cvalue[1];
	s_buffer[8] = Vy.cvalue[2];
	s_buffer[9] = Vy.cvalue[3];
	//Ang_v
	s_buffer[10] = Ang_v.cvalue[0];
	s_buffer[11] = Ang_v.cvalue[1];
	s_buffer[12] = Ang_v.cvalue[2];
	s_buffer[13] = Ang_v.cvalue[3];
	//CRC
	s_buffer[14] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^
					s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13];
	/*
	for(i=0;i<15;i++){
		ROS_INFO("0x%02x",s_buffer[i]);
	}
	*/
	ser.write(s_buffer,sBUFFERSIZE);
}





//接收数据分析与校验
unsigned char data_analysis(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	uint8_t DataLen;
	DataLen = 58;
	//int i;
	if((buffer[0]==0xBD)&&(buffer[1]==0xDB)&&(buffer[2]==0x0B))
	{
        
	   for (int i = 0; i < DataLen - 1; i++)
       {
         csum ^= buffer[i];    //异或运算校验字节的正确
       }	
		/*
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9]^buffer[10]^buffer[11]^buffer[12]^buffer[13]^
				buffer[14]^buffer[15]^buffer[16]^buffer[17]^buffer[18]^buffer[19]^
				buffer[20]^buffer[21]^buffer[22]^buffer[23]^buffer[24]^buffer[25];
        */
		ROS_INFO("check sum:0x%02x",csum);
	
	
		if(csum == buffer[58]){
			ret = 1;//校验通过，数据包正确
		}
		else 
		  ret =0;//校验失败，丢弃数据包
	}
	/*
	for(i=0;i<rBUFFERSIZE;i++)
	  ROS_INFO("0x%02x",buffer[i]);
	*/
	return ret;

}


/*
void ReadBD_DB(uint8_t* Data)
{
	// INSResDataStruct* ResData = GetINSResData();

	uint8_t datatype,checksum,DataLen;
	int32_t buf;
    DataLen = 58;
    checksum = 0;
	if (Data[0] == 0xBD && Data[1] == 0xDB && Data[2] == 0x0B)    //判断起始位正确
	{

    for (i = 0; i < DataLen - 1; i++)
    {
        checksum ^= Data[i];    //异或运算校验字节的正确
    }

    if (Data[DataLen - 1] == checksum)  //校验位正确
    {
        buf = (int16_t)(Data[3] + (Data[4] << 8));
        ResData->Roll = ((float)buf) * 360 / 32768;      //解析横滚角

        buf = (int16_t)(Data[5] + (Data[6] << 8));
        ResData->Pitch = ((float)buf) * 360 / 32768;     //解析俯仰角

        buf = (int16_t)(Data[7] + (Data[8] << 8));
        ResData->heading = ((float)buf) * 360 / 32768;   //解析偏航角

        buf = (int16_t)(Data[9] + (Data[10] << 8));
        ResData->gx = ((float)buf) * 300 / 32768;        //解析x轴角速度

        buf = (int16_t)(Data[11] + (Data[12] << 8));
        ResData->gy = ((float)buf) * 300 / 32768;        //解析y轴角速度

        buf = (int16_t)(Data[13] + (Data[14] << 8));
        ResData->gz = ((float)buf) * 300 / 32768;        //解析z轴角速度

        buf = (int16_t)(Data[15] + (Data[16] << 8));
        ResData->ax = ((float)buf) * 12 / 32768;          //解析x轴加速度

        buf = (int16_t)(Data[17] + (Data[18] << 8));
        ResData->ay = ((float)buf) * 12 / 32768;          //解析y轴加速度
  
        buf = (int16_t)(Data[19] + (Data[20] << 8));
        ResData->az = ((float)buf) * 12 / 32768;          //解析z轴加速度


        buf = (int32_t)(Data[21] + (Data[22] << 8) + (Data[23] << 16) + (Data[24] << 24));
        ResData->lat = ((double)buf) / 1e7f;              //解析纬度  

        buf = (int32_t)(Data[25] + (Data[26] << 8) + (Data[27] << 16) + (Data[28] << 24));
        ResData->lon = ((double)buf) / 1e7f;               //解析经度
 
        buf = (int32_t)(Data[29] + (Data[30] << 8) + (Data[31] << 16) + (Data[32] << 24));
        ResData->alt = ((float)buf) / 1e3f;                //解析高度

        buf = (int16_t)(Data[33] + (Data[34] << 8));
        ResData->Vn = ((float)buf) * 100/ 32768;          //解析北向速度   
        buf = (int16_t)(Data[35] + (Data[36] << 8));
        ResData->Ve = ((float)buf) * 100/ 32768;          //解析东向速度
        buf = (int16_t)(Data[37] + (Data[38] << 8));
        ResData->Vd = ((float)buf) * 100/ 32768;          //解析地向速度

        buf = (int32_t)(Data[52] + (Data[53] << 8) + (Data[54] << 16) + (Data[55] << 24));
        ResData->INSTime = ((float)buf) / 4000.0f;         //解析时间

        ResData->status =  Data[39];//HEADING: 0x08 ATT:0x04 VEL:0x02 POS:0x01 init flag
				ResData->NavStatus = 0;
				ResData->ChannelInfo = 0;   
				
     }
		    
  }
}
*/



//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	data_pack(cmd_vel);
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "serial_GPS_IMU_node");
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key

	//ros::Subscriber write_sub = nh.subscribe("/turtle1/cmd_vel",1000,cmd_vel_callback);

	//发布位置和姿态话题  position  GPS位置信息？ IMU姿态信息？
	//ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);

    //定义GPS数据消息发布
    ros::Publisher GPS_pub = nh.advertise<sensor_msgs::NavSatFix>("GPS_data", 20);
	//定义IMU数据消息发布
	ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 20);
    //定义轨迹数据消息发布
	ros::Publisher Path_pub = nh.advertise<nav_msgs::Path>("Path_data", 20);
    //定义速度数据消息发布
	ros::Publisher CmdVel_pub = nh.advertise<geometry_msgs::Twist>("CmdVel_data", 20);

    ros::Time current_time,lase_time;
    current_time=ros::Time::now();
    lase_time=ros::Time::now(); 

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
    ros::Rate loop_rate(10);

    while(ros::ok())
	{

        ros::spinOnce();

        if(ser.available())
		{

            ROS_INFO_STREAM("Reading from serial port_0");
			ser.read(r_buffer,rBUFFERSIZE);
           	
			int a;
				
            uint8_t datatype,checksum,DataLen;
	        int32_t buf;
            DataLen = 58;
            checksum = 0;
			
		for(int i=0;i<rBUFFERSIZE;i++)
		   {
		    if (r_buffer[i] == 0xBD && r_buffer[i+1] == 0xDB && r_buffer[i+2] == 0x0B)    //判断起始位正确
		      { 
                 ROS_INFO_STREAM("Start reading from serial port");
				 a=i;   
				 //调试语句
				 for(int j=a;j<58+a;j++)
				  {
					  ROS_INFO("0x%02x",r_buffer[j]);
				  }
                 //校验位
                for (int z=a ; z < 57+a; z++)
                  {
                    checksum ^= r_buffer[z];    //异或运算校验字节的正确
                  }

                 ROS_INFO("Checksum:0x%02x",checksum);
				  
               //  if (r_buffer[57+a] == checksum)  //校验位正确
               //    {
                     //串口数据解析
		         	ROS_INFO_STREAM("The port date is right!");
		         	//定义IMU数据消息对象
                    sensor_msgs::Imu imu_data;
                    imu_data.header.stamp = ros::Time::now();
                    imu_data.header.frame_id = "base_link";


		        	
		        	float Roll,Pitch,heading;
					tf::Quaternion q;
                    buf = (int16_t)(r_buffer[a+3] + (r_buffer[a+4] << 8));
                    Roll = ((float)buf) * 360 / 32768;      //解析横滚角
 
                    buf = (int16_t)(r_buffer[a+5] + (r_buffer[a+6] << 8));
                    Pitch = ((float)buf) * 360 / 32768;     //解析俯仰角

                    buf = (int16_t)(r_buffer[a+7] + (r_buffer[a+8] << 8));
                    heading = ((float)buf) * 360 / 32768;   //解析偏航角

                    //q.setRPY(Roll,Pitch,heading);
					imu_data.orientation.x=Roll;
					imu_data.orientation.y=Pitch;
					imu_data.orientation.z=heading;
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
                              
                    //定义GPS的数据消息对象
	                 sensor_msgs::NavSatFix gps_data;
		             gps_data.header.stamp =ros::Time::now();
		             gps_data.header.frame_id = "base_link";
                    
                     
                     buf = (int32_t)(r_buffer[a+21] + (r_buffer[a+22] << 8) + (r_buffer[a+23] << 16) + (r_buffer[a+24] << 24));
                     gps_data.latitude = ((double)buf) / 1e7f;              //解析纬度  
                     //gps_data.latitude=45.7458007+i;


                     buf = (int32_t)(r_buffer[a+25] + (r_buffer[a+26] << 8) + (r_buffer[a+27] << 16) + (r_buffer[a+28] << 24));
                     gps_data.longitude = ((double)buf) / 1e7f;               //解析经度
                     //gps_data.longitude=126.6261953+j;
                     //i=i-0.00000000008;
	                 //j=j-0.000000000009;

                     buf = (int32_t)(r_buffer[a+29] + (r_buffer[a+30] << 8) + (r_buffer[a+31] << 16) + (r_buffer[a+32] << 24));
                     gps_data.altitude = ((float)buf) / 1e3f;                //解析高度

		             GPS_pub.publish(gps_data);   //GPS数据发布
               //   }

	       //定义速度数据消息对象

           geometry_msgs::Twist cmdvel_data;

           buf = (int16_t)(r_buffer[a+33] + (r_buffer[a+34] << 8));
           cmdvel_data.linear.x= ((float)buf) * 100/ 32768;          //解析北向速度   
           buf = (int16_t)(r_buffer[a+35] + (r_buffer[a+36] << 8));
           cmdvel_data.linear.y= ((float)buf) * 100/ 32768;          //解析东向速度
           buf = (int16_t)(r_buffer[a+37] + (r_buffer[a+38] << 8));
           cmdvel_data.linear.z = ((float)buf) * 100/ 32768;          //解析地向速度
            
           CmdVel_pub.publish(cmdvel_data);   //速度数据发布  
           //buf = (int32_t)(r_buffer[a+52] + (r_buffer[a+53] << 8) + (r_buffer[a+54] << 16) + (r_buffer[a+55] << 24));
           //ResData->INSTime = ((float)buf) / 4000.0f;         //解析时间

           //ResData->status =  Data[39];//HEADING: 0x08 ATT:0x04 VEL:0x02 POS:0x01 init flag
		   //ResData->NavStatus = 0;
		   //ResData->ChannelInfo = 0;   
        

	
		   // 	else
		   //	     {
           //            ROS_ERROR_STREAM("The port date is not right! ");
		   //     }		

                  nav_msgs::Path path;
                  path.header.stamp=current_time;
                  path.header.frame_id="world";

                  geometry_msgs::PoseStamped this_pose_stamped; 
             
		          this_pose_stamped.pose.position.x = gps_data.latitude; 
		          this_pose_stamped.pose.position.y = gps_data.longitude;
                  this_pose_stamped.pose.position.z = gps_data.altitude;

		         // geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th); 
	              this_pose_stamped.pose.orientation.x = q[0]; 
		          this_pose_stamped.pose.orientation.y = q[1]; 
		          this_pose_stamped.pose.orientation.z = q[2]; 
		          this_pose_stamped.pose.orientation.w = q[3]; 

		          this_pose_stamped.header.stamp=current_time; 
	        	  this_pose_stamped.header.frame_id="world"; 

	              path.poses.push_back(this_pose_stamped); 
		          Path_pub.publish(path); 
                  
                 ROS_INFO_STREAM("End reading from serial port");
			  }
		    } 
		          
		  }
			memset(r_buffer,0,rBUFFERSIZE);
        }
        loop_rate.sleep();

    }


