/*********************************
 * 串口节点，订阅cmd_vel话题控制底盘进行运动
 * 从决策节点的数据cmd_vel话题中分解出速度值通过串口送到移动底盘进行控制 
 * 代码修改需要删除CAN形式的轮式里程计
 * @ZSJ
 * *******************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>       //发布串口指令
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>  //订阅决策层的cmd_vel消息数据
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <vector>

#define	sBUFFERSIZE	14//send buffer size 串口发送缓存长度 
#define	rBUFFERSIZE	27//receive buffer size 串口接收缓存长度

using namespace std;

double linear_x_amplifier = 800;
double angular_z_amplifier = 100;

unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

/************************************
 * 串口数据发送格式为字符
 * 前进 ！M 0 正数 \r
 * 后退 ！M 0 负数 \r
 * 左转 ！M 正数 0 \r
 * 右转 ！M 负数 0 \r
 * **********************************/
/**********************************************************
 * 串口接收数据格式共27字节暂时不用
 * head head x-position y-position x-speed y-speed angular-speed pose-angular CRC
 * 0xaa 0xaa float      float      float   float   float         float(yaw)   u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union
{
	unsigned char cvalue[4];
	float fvalue;
}float_union;

serial::Serial ser;

/**********************************************************
 * 检测决策层的控制数据打包，将获取的cmd_vel信息打包并通过串口发送
 * ********************************************************/
void get_command ( const int& velocity_1, const int& velocity_2 )
{
	
    vector <unsigned char> ucv;
    char c1[6];
	char c2[6];
	memset(s_buffer,0,sizeof(s_buffer));

    ucv.push_back('!');
    ucv.push_back('M');
    ucv.push_back(' ');
    
    // velocity 1 angular
    sprintf(c1, "%d", velocity_1);
    ucv.push_back(c1[0]);
    if (velocity_1 >9)
      ucv.push_back(c1[1]);
	      if (velocity_1 >99)
      ucv.push_back(c1[2]); 
	  // minus 0
	  if (velocity_1 < 0)
	        ucv.push_back(c1[1]);
				  if (velocity_1 < -9)
	        ucv.push_back(c1[2]);
				  if (velocity_1 < -99)
	        ucv.push_back(c1[3]);
    ucv.push_back(' ');
    // velocity 2
    sprintf(c2, "%d", velocity_2);
    ucv.push_back(c2[0]);
    if (velocity_2 >9)
      ucv.push_back(c2[1]);
	      if (velocity_2 >99)
      ucv.push_back(c2[2]); 
	  // minus 0
	  if (velocity_2 < 0)
	        ucv.push_back(c2[1]);
				  if (velocity_2 < -9)
	        ucv.push_back(c2[2]);
				  if (velocity_2 < -99)
	        ucv.push_back(c2[3]);
    ucv.push_back(' ');
    
    ucv.push_back('\r');
    
    int index = 0;
    for (auto uc:ucv)
    {
      s_buffer[index] = uc;
      index ++;
    }

	ser.write(s_buffer,sBUFFERSIZE);
	ros::Duration(0.03).sleep();
}

void data_pack(const geometry_msgs::Twist& cmd_vel)
{
	unsigned char i;
	float_union Vx,Vy,Ang_v;

	//Vx.fvalue = cmd_vel.linear.x;
	//Vy.fvalue = cmd_vel.linear.y;
	//Ang_v.fvalue = cmd_vel.angular.z;
	
	int velocity_1  = int(angular_z_amplifier* cmd_vel.angular.z);	
	int velocity_2  = int(linear_x_amplifier* cmd_vel.linear.x);

	get_command(velocity_1, velocity_2);
	
}



//接收数据分析与校验   接受小车地盘的CAN总线反馈数据
unsigned char data_analysis(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	//int i;
	if((buffer[0]==0xaa) && (buffer[1]==0xaa))
	{
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9]^buffer[10]^buffer[11]^buffer[12]^buffer[13]^
				buffer[14]^buffer[15]^buffer[16]^buffer[17]^buffer[18]^buffer[19]^
				buffer[20]^buffer[21]^buffer[22]^buffer[23]^buffer[24]^buffer[25];
		//ROS_INFO("check sum:0x%02x",csum);
		if(csum == buffer[26])
		{
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


//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
	ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	data_pack(cmd_vel);
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key
	ros::Subscriber write_sub = nh.subscribe("/cmd_vel",10,cmd_vel_callback);
	//发布里程计话题 odom
	ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);

    try
    {
        ser.setPort("/dev/ttyUSB1");  //串口驱动号需要修改
        ser.setBaudrate(115200);     //波特率设置
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port_1 ");
        return -1;
    }

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("Serial Port_1 initialized");
    }
	else
	{
        return -1;
    }


	//定义tf 对象
	static tf::TransformBroadcaster odom_broadcaster;
	//定义tf发布时需要的类型消息
	geometry_msgs::TransformStamped odom_trans;
	//定义里程计消息对象
	nav_msgs::Odometry odom;
	//定义四元数变量
	geometry_msgs::Quaternion odom_quat;
	//位置 速度 角速度
	float_union posx,posy,vx,vy,va,yaw;

	//定义时间
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();


    while(ros::ok())
	{

        ros::spinOnce();

    }
}
