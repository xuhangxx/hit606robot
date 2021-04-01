#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <indoor_navigation/soc.h>
#include <indoor_navigation/panangle.h>
#include <indoor_navigation/panctrmode.h>
#include <indoor_navigation/panimu.h>
#define	sBUFFERSIZE	7//send buffer size 串口发送缓存长度
#define	rBUFFERSIZE	14//receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

const double pi = 3.1415926535898;
double wx=0;
double wy=0;
double wz=0;
double imuyaw=0;
double imupitch=0;
double imuroll=0;
double Imuyaw=0;
double Imupitch=0;
double Imuroll=0;
double yaw=0;
double pitch=0;
double roll=0;
double p1;
double p2;
int flag=0;
int p=0;
//int p1=0;
//int p2=0;
serial::Serial ser;

int mode=1;
int mode2=5;
int v1=0;
int v2=0;
indoor_navigation::panangle panangle;
indoor_navigation::panctrmode panmode;


void imu_callback(const sensor_msgs::Imu& imu){
    tf::Transform transform;
	transform.setRotation( tf::Quaternion(imu.orientation.x,
                                          imu.orientation.y,
                                          imu.orientation.z,
                                          imu.orientation.w) );
    transform.getBasis().getEulerYPR(Imuyaw, Imupitch, Imuroll);
    wx=imu.angular_velocity.x;
    wy=imu.angular_velocity.y;
    wz=imu.angular_velocity.z;
    v1=wy*32/0.24;
}

void imu_callback2(const indoor_navigation::panimu& panimu){
    imuyaw=panimu.yaw;
    imupitch=panimu.pitch;
    imuroll=panimu.roll;
}

void soc_callback(const indoor_navigation::soc& soc)
{
    if(soc.code == 7 && soc.action ==1)
    {
        mode=1;
        panmode.panctrmodeback=1;
    }
    else if(soc.code == 7 && soc.action ==2)
    {
        mode=2;
        panmode.panctrmodeback=2;
    }
    else if(soc.code == 7 && soc.action ==3)
    {
        mode=3;
        panmode.panctrmodeback=3;
    }
    else if(soc.code == 7 && soc.action ==4)
    {
        mode2=1;
    }
    else if(soc.code == 7 && soc.action ==5)
    {
        mode2=2;
    }
    else if(soc.code == 7 && soc.action ==6)
    {
        mode2=3;
    }
    else if(soc.code == 7 && soc.action ==7)
    {
        mode2=4;
    }
    else if(soc.code == 7 && soc.action ==8)
    {
        mode2=5;
        yaw=0;
        pitch=0;
    }
    else if(soc.code == 7 && soc.action ==9)
    {
        mode2=0;
    }
    
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "ytnode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //ros::Subscriber angle_sub = nh.subscribe("/panangle", 2,angle_callback);
    //ros::Publisher CmdVel_pub = nh.advertise<geometry_msgs::Twist>("CmdVel_data", 20);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 2,imu_callback);
    ros::Subscriber imu_sub2 = nh.subscribe("/imu2", 2,imu_callback2);
    ros::Subscriber soc_sub = nh.subscribe("/soc",1,soc_callback);
    ros::Publisher angle_pub= nh.advertise<indoor_navigation::panangle>("/panangle",1);
    ros::Publisher ctrmode_pub= nh.advertise<indoor_navigation::panctrmode>("/panctrmode",1);
	panmode.panctrmodeback=1;
    
    nh_private.param<double>("p1",  p1, 1);
    nh_private.param<double>("p2",  p2, 2);
    try
    {
        ser.setPort("/dev/yt");
        ser.setBaudrate(2400);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port_yt ");
        return -1;
    }

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("Serial Port_yt initialized");
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
        if(mode2==1)
        {
            pitch=pitch-0.5;
        }
        else if(mode2==2)
        {
            pitch=pitch+0.5;
        }
        else if(mode2==3)
        {
            yaw=yaw+0.5;
        }
        else if(mode2==4)
        {
            yaw=yaw-0.5;
        }
        else if(!mode2)
        {
	        pitch=imupitch*180/pi;
	        yaw=imuyaw*180/pi-Imuyaw*180/pi;
        }
        else if(mode2==5)
        {

        }

        if(mode==1)
        {
            v1=(v1+p1*(imupitch*180/pi-pitch))/(1+p1);
            panangle.yaw=0;
            panangle.pitch=imupitch*180/pi;
            panangle.roll=0;
            angle_pub.publish(panangle);
            memset(s_buffer,0,sBUFFERSIZE);
            if(v1<0)
            {
                if(v1<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x10;
		            s_buffer[4]=0x00;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0x75;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x10;
		            s_buffer[4]=0x00;
		            s_buffer[5] += -v1;
		            s_buffer[6] += -v1+17;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v1>0)
            {
                if(v1>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x08;
		            s_buffer[4]=0x00;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0x6d;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x08;
		            s_buffer[4]=0x00;
		            s_buffer[5] += v1;
		            s_buffer[6] += v1+9;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else
            {
                s_buffer[0]=0xff;
		        s_buffer[1]=0x01;
		        s_buffer[2]=0x00;
		        s_buffer[3]=0x00;
		        s_buffer[4]=0x00;
		        s_buffer[5]=0x00;
		        s_buffer[6]=0x01;
		        ser.write(s_buffer,sBUFFERSIZE);
            }
            //std::cout<<"buffer   "<<s_buffer<<std::endl;
        }
        else if(mode==2)
        {
            v1=(v1+p1*(imupitch*180/pi-pitch))/(1+p1);
            v2=p2*(imuyaw*180/pi-Imuyaw*180/pi-yaw);//+(wy2-wy)*2/0.24;
            panangle.yaw=imuyaw*180/pi-Imuyaw*180/pi;
            panangle.pitch=imupitch*180/pi;
            panangle.roll=0;
            angle_pub.publish(panangle);
            memset(s_buffer,0,sBUFFERSIZE);
            if(v1<0 && v2<0)
            {
                if(v1<-100 && v2<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x14;
		            s_buffer[4]=0x64;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0xdd;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v2<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x14;
		            s_buffer[4]=0x64;
		            s_buffer[5] += -v1;
		            s_buffer[6] += -v1+121;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v1<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x14;
		            s_buffer[4] += -v2;
		            s_buffer[5]=0x64;
		            s_buffer[6] += -v2+121;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x14;
		            s_buffer[4] += -v2;
		            s_buffer[5] += -v1;
		            s_buffer[6] += -v2-v1+21;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v1<0 && v2>0)
            {
                if(v1<-100 && v2>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x12;
		            s_buffer[4]=0x64;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0xdb;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v2>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x12;
		            s_buffer[4]=0x64;
		            s_buffer[5] += -v1;
		            s_buffer[6] += -v1+119;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v1<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x12;
		            s_buffer[4] += v2;
		            s_buffer[5]=0x64;
		            s_buffer[6] += v2+119;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x12;
		            s_buffer[4] += v2;
		            s_buffer[5] += -v1;
		            s_buffer[6] += v2-v1+19;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v1>0 && v2>0)
            {
                if(v1>100 && v2>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0a;
		            s_buffer[4]=0x64;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0xd3;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v2>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0a;
		            s_buffer[4]=0x64;
		            s_buffer[5] += v1;
		            s_buffer[6] += v1+111;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v1>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0a;
		            s_buffer[4] += v2;
		            s_buffer[5]=0x64;
		            s_buffer[6] += v2+111;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0a;
		            s_buffer[4] += v2;
		            s_buffer[5] += v1;
		            s_buffer[6] += v2+v1+11;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v1>0 && v2<0)
            {
                if(v1>100 && v2<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0c;
		            s_buffer[4]=0x64;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0xd5;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v2<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0c;
		            s_buffer[4]=0x64;
		            s_buffer[5] += v1;
		            s_buffer[6] += v1+113;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else if(v1>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0c;
		            s_buffer[4] += -v2;
		            s_buffer[5]=0x64;
		            s_buffer[6] += -v2+113;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x0c;
		            s_buffer[4] += -v2;
		            s_buffer[5] += v1;
		            s_buffer[6] += -v2+v1+13;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v1>0)
            {
                if(v1>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x08;
		            s_buffer[4]=0x00;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0x6d;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x08;
		            s_buffer[4]=0x00;
		            s_buffer[5] += v1;
		            s_buffer[6] += v1+9;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v2<0)
            {
                if(v2<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x04;
		            s_buffer[4]=0x64;
		            s_buffer[5]=0x00;
		            s_buffer[6]=0x69;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x04;
		            s_buffer[4] +=-v2;
		            s_buffer[5]=0x00;
		            s_buffer[6] += -v2+5;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v1<0)
            {
                if(v1<-100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x10;
		            s_buffer[4]=0x00;
		            s_buffer[5]=0x64;
		            s_buffer[6]=0x75;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x10;
		            s_buffer[4]=0x00;
		            s_buffer[5] += -v1;
		            s_buffer[6] += -v1+17;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else if(v2>0)
            {
                if(v2>100)
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x02;
		            s_buffer[4]=0x64;
		            s_buffer[5]=0x00;
		            s_buffer[6]=0x67;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
                else 
                {
                    s_buffer[0]=0xff;
		            s_buffer[1]=0x01;
		            s_buffer[2]=0x00;
		            s_buffer[3]=0x02;
		            s_buffer[4] +=v2;
		            s_buffer[5]=0x00;
		            s_buffer[6] += v2+3;
                    ser.write(s_buffer,sBUFFERSIZE);
                }
            }
            else
            {
                s_buffer[0]=0xff;
		        s_buffer[1]=0x01;
		        s_buffer[2]=0x00;
		        s_buffer[3]=0x00;
		        s_buffer[4]=0x00;
		        s_buffer[5]=0x00;
		        s_buffer[6]=0x01;
                ser.write(s_buffer,sBUFFERSIZE);
            }
            //std::cout<<"buffer0   "<<v1<<std::endl;
            //std::cout<<"buffer1   "<<v2<<std::endl;
        }
        else if(mode==3)
        {
            panangle.yaw=0;
            panangle.pitch=0;
            panangle.roll=0;
            angle_pub.publish(panangle);
        }
        ctrmode_pub.publish(panmode);
        loop_rate.sleep();
    }

    
}


