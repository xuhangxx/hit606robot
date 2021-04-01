/*
*程序功能：
*读入gps轨迹信息并转换成UTM坐标 
*订阅/GPS_data与/IMU_data信息并进入回调函数进行决策
*发布/cmd_vel控制信息
*（或者发布/control_cmdvel亚控制信息，结合避障等线程进行后续决策）
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <list>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <vector>


using namespace std;
const double pi = 3.1415926535898;
double a=6378160;   
double b=6356775;  
double c=(a-b)/a;  

//注意： 经度是横坐标x，维度是纵坐标y
// GPS录制轨迹txt里 第一个量是维度，第二个量是经度

//全局变量
vector<vector<float>> gps_x_y; // 存储读取的GPS轨迹

vector<vector<float>> utm_x_y; // 存储UTM轨迹


vector<float> origin_x_y; //GPS原点坐标

vector<float> current_utm_x_y; // 当前位置点UTM坐标

double current_yaw; // 当前北偏东航向

double target_utm_x; // 目标点UTM坐标
double target_utm_y;

double heading_next_utm_x;
double heading_next_utm_y;

//标志位
bool flag_first_time = 1; // 首次决策
bool flag_reached_target = 1; // 已到达当前目标，可进行下一步规划

//目标点搜寻常值
double utm_distance_threshold = 0.5; // 距离阈值设定，用于判断是否达到目标点
double delta_threshold = 10; // 目标点偏移角度阈值，单位：度
double forward_distance = 1.0; // 下一个目标点(沿轨迹)前进距离

//运动控制常值
double normal_linear_x = 0.5; // 直行线速度
double turning_linear_x = 0.2; // 转向线速度
double left_angular_z = 0.5; // 转向角速度
double right_angular_z = -0.5;



// 工具函数
vector<float> LL2UTM_respective(float lon, float lat)
{
    double e=2*c-c*c;
    double ee=e/(1-e);
    float lonRad=(lon+180)-int((lon+180)/360)*360-180;
    float latRad = lat*pi/180;
    lonRad = lonRad*pi/180;

    double V = a/sqrt(1-e*pow(sin(latRad),2));
    double T = pow(tan(latRad),2);
    double C = ee*pow(cos(latRad),2);
    double A = cos(latRad)*lonRad;
    double M = a*((1-e/4-3*e*e/64-5*e*e*e/256)*latRad-(3*e/8+3*e*e/32+45*e*e*e/1024)*sin(2*latRad)
    +(15*e*e/256+45*e*e*e/1024)*sin(4*latRad)-(35*e*e*e/3072)*sin(6*latRad));
 
    vector<float> UTM;
    UTM[0] = 0.9996*V*(A+(1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*ee)*pow(A,5)/120);//+ 500000.0
    UTM[1] = 0.9996*(M+V*tan(latRad)*(A*A/2+(5-T+9*C+4*C*C)*pow(A,4)/24 +(61-58*T+T*T+600*C-330*ee)*pow(A,6)/720));


    return UTM;
}

//类定义：订阅GPS与IMU，发布cmd_vel
class SubsGPSIMUPubControlCmdvel  
{  
public:  
    SubsGPSIMUPubControlCmdvel ()  
    {       
        sub_GPS_ = n_.subscribe("/GPS_data", 1, &SubsGPSIMUPubControlCmdvel::callback_GPS, this);
        sub_IMU_ = n_.subscribe("/IMU_data", 1, &SubsGPSIMUPubControlCmdvel::callback_IMU, this);
        pub_control_cmdvel_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

//两个callback将会顺序执行 ，先GPS后IMU
    void callback_GPS (const sensor_msgs::NavSatFixConstPtr& msg)
    {
        current_utm_x_y = LL2UTM_respective(msg->longitude-origin_x_y[0], msg->latitude-origin_x_y[1]);
    }

//两个callback将会顺序执行 ，先GPS后IMU
    void callback_IMU (const sensor_msgs::ImuConstPtr& msg)
    {
        current_yaw = msg->orientation.z;

        //测试输出
        //SubsGPSIMUPubControlCmdvel::test_cout();
        // 当前数据接受完毕，进入决策过程
        if (flag_reached_target)// 如果已经到达目标点，则进行下一次目标点寻找
        {
            SubsGPSIMUPubControlCmdvel::find_target();
        }else // 未到达目标点，继续按照原目标点进行控制
        {
            SubsGPSIMUPubControlCmdvel::pub_control_cmdvel();
        }
    }
// 找下一个点
    void find_target()
    {
        /*
        // 记得更新target坐标值
        heading_next_utm_x = sin(current_yaw * pi / 180) * forward_distance + current_utm_x;
        heading_next_utm_y = cos(current_yaw * pi / 180) * forward_distance + current_utm_y;
        */
        SubsGPSIMUPubControlCmdvel::get_nearest_point(current_utm_x_y); // 在函数中更新目标utm坐标值
        
        //目标点已确定，设定状态，进入决策
        flag_reached_target = 0;
        ROS_INFO("Planning next point.");
        SubsGPSIMUPubControlCmdvel::pub_control_cmdvel();
    }

// 处理数据 发布cmdvel 
    void pub_control_cmdvel()
    {   
        double delta = SubsGPSIMUPubControlCmdvel::calculate_delta(current_utm_x, current_utm_y, target_utm_x, target_utm_y, current_yaw);
        cout << "当前UTM位置：( "<< current_utm_x << ", " << current_utm_y << " )   目标UTM位置：(" << target_utm_x << ", " << target_utm_y << " )" << endl; 
        SubsGPSIMUPubControlCmdvel::test_out_target_state(delta, 
        SubsGPSIMUPubControlCmdvel::calculate_utm_distance(current_utm_x, current_utm_y, target_utm_x, target_utm_y));
        
        geometry_msgs::Twist cmd_;
        // 判断是否已经达到目标点
        if (SubsGPSIMUPubControlCmdvel::calculate_utm_distance(current_utm_x, current_utm_y, target_utm_x, target_utm_y) 
        < utm_distance_threshold) // 已经达到目标点
        {
            flag_reached_target = 1; 
            ROS_INFO("------------------Reached current target.-----------------");
            cmd_.linear.x = normal_linear_x;
            pub_control_cmdvel_.publish(cmd_);
            ros::Duration(0.05).sleep();
        }else // 还未达到目标点
        {
            if (delta < delta_threshold && delta > -delta_threshold) // 目标点在直行范围内
            {
                cmd_.linear.x = normal_linear_x;
                ROS_INFO(" ||||||straight||||| ");
            }else if (delta > delta_threshold) // 目标点在左侧
            {
                cmd_.linear.x = turning_linear_x;
                cmd_.angular.z = left_angular_z;
                ROS_INFO(" ||||||||||| left ");
            }else //目标点在右侧
            {
                cmd_.linear.x = turning_linear_x;
                cmd_.angular.z = right_angular_z;
                ROS_INFO(" right ||||||||||| ");
            }
            pub_control_cmdvel_.publish(cmd_);
            ros::Duration(0.05).sleep();
            flag_reached_target = 1; // 走一步一找
        }
        // 用于重新规划
        if (SubsGPSIMUPubControlCmdvel::calculate_utm_distance(current_utm_x, current_utm_y, target_utm_x, target_utm_y) > 2)
        {
            flag_reached_target = 1;
            cout <<"距离目标太远，重新规划。" << endl;
        }

    }

////////////////////////////////////类内工具函数//////////////////////////////////

//测试输出
    void test_cout()
    {
        cout << "current latitude:  " << current_y_latitude << endl;
        cout << "current longitude: " << current_x_longitude << endl;
        cout << "current umt x: " << current_utm_x << endl;
        cout << "current umt y: " << current_utm_y << endl;
        cout << "current yaw: " << current_yaw << endl;
    }
    void test_out_target_state( double delta, double distance)
    {
        if (delta < 0 )
        {
            cout << "目标在我右侧 " << -delta << "度." << endl;
        }else{
            cout << "目标在我左侧 " << delta << "度." << endl;
        }

        cout << "距离目标 " << distance << " 米."<< endl;
    }
////////////////////////////////////////////////////////////////////
// UTM距离计算
    double calculate_utm_distance(double cur_x, double cur_y, double tar_x, double tar_y)
    {
        return sqrt(pow((cur_x - tar_x),2) + pow((cur_y - tar_y),2));
    }
////////////////////////////////////////////////////////////////////
// 目标点与当前方向之间偏角计算，使用向量内积
// 单位：度
// 目标点在左侧，内积为正，角度为正
    double calculate_delta(double cur_x, double cur_y, double tar_x, double tar_y, double current_yaw_angle)
    {
        double heading_x = sin(current_yaw_angle * pi / 180) + cur_x;
        double heading_y = cos(current_yaw_angle * pi / 180) + cur_y;
        double length_cur_tar = sqrt(pow((cur_x - tar_x),2) + pow((cur_y - tar_y),2)); 
        double delta_abs = acos( ( (heading_x - cur_x)*(tar_x - cur_x) + (heading_y - cur_y)*(tar_y - cur_y) )/(1*length_cur_tar) );   
        double outer_prod = (heading_x - cur_x)*(tar_y - cur_y) - (heading_y - cur_y)*(tar_x - cur_x);   
        double dir;
        if (outer_prod > 0)
        {
	        dir = 1;
        }else{
	        dir = -1;
        }
        return dir*delta_abs*180/pi;
    }
////////////////////////////////////////////////////////////////////
// 寻找最近点
// 寻找与传入点最近的 轨迹上的点, 并向后推移一定距离作为目标点
    void get_nearest_point( vector<double> heading_next_utm_x_y)
    {
        double dis_min = 10;
        int index_min;
        int index_final;
        double distance_ = 0;
        for (int i=0;i<utm_x.size();i++)
        {
            double dis = SubsGPSIMUPubControlCmdvel::calculate_utm_distance(heading_next_utm_x,heading_next_utm_y,utm_x[i], utm_y[i]);
            if (dis < dis_min)
            {
                dis_min = dis;
                index_min = i;
            }
        }

        index_final = index_min;
        while ( distance_ < forward_distance && index_final < utm_x.size())
        {
            distance_ += SubsGPSIMUPubControlCmdvel::calculate_utm_distance(utm_x[index_final],utm_y[index_final],utm_x[index_final+1],utm_y[index_final+1]);
            index_final += 1;
        }
        target_utm_x = utm_x[index_final];
        target_utm_y = utm_y[index_final];

        
    }
////////////////////////////////////////////////////////////////////

    private:  
      ros::NodeHandle n_;
      ros::Subscriber sub_GPS_;
      ros::Subscriber sub_IMU_;   
      ros::Publisher pub_control_cmdvel_;
}; 
      


int main( int argc, char** argv)
{
    //读入gps轨迹数据
    ifstream in("/home/robot/catkin_805/src/decision_node/data/1123gps_lat&lon",ios::in);
    if (!in)
    {
        ROS_INFO("No data readin.");
        return -1;
    }
    while (!in.eof())
    {
        vector<float> gps;
        in >> gps[1] >> gps[0];
        gps_x_y.push_back(gps);
    }
    //建立原点经纬度
    origin_x_y = gps_x_y[0];
    //去原点utm坐标
    for (auto gps:gps_x_y)
    {
        utm_x_y.push_back(LL2UTM_respective(gps[0]-origin_x_y[0],gps[1]-origin_x_y[1]));
    }
   

    // 输出txt测试
    /*
    ofstream fout1;
    fout1.open("index_points.txt",ios_base::out);
    for (int i=0;i<utm_x.size(); i++)
    {
        fout1 << utm_x[i] << " ";
        fout1 << utm_y[i];
        fout1 << endl;
    }
    fout1.close();
    ROS_INFO("Trace saved.");
    */

    

//建立在已知轨迹数据基础上，进行ROS话题订阅、决策、控制

    ROS_INFO("Loaded GPS data, now subscribing data ... ");

    ros::init(argc, argv, "decision_node_gps_imu"); 
    SubsGPSIMUPubControlCmdvel sub_pub;  
    ros::spin();
    
    return 0;
}
