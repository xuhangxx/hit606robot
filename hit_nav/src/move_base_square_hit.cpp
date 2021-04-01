#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <indoor_navigation/soc.h>
using namespace std;
double a,b,c;
int i=0;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
geometry_msgs::Pose pose_list[1000];
ros::Publisher cmdVelPub;
ros::Publisher marker_pub;
geometry_msgs::Point current_point;  //a pose consisting of a position and orientation in the map frame.

geometry_msgs::Point setPoint(double _x, double _y, double _z);
geometry_msgs::Quaternion setQuaternion(double _angleRan);
void init_goalList();
void shutdown(int sig);
void init_markers(visualization_msgs::Marker *marker);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal);
string mulu0;
string filename;
int flag=0;
int socflag=0;
int flagfirst=1;
int flagmulu0=0;
double x=0;
double y=0;
double xx=0;
double yy=0;
double yawlll=0;
double dis;
double mindis=100;


inline bool exist(const std::string& name){
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0);
}

void init_goalList()
{
	//How big is the square we want the robot to navigate?
	//Create a list to hold the targ
	geometry_msgs::Quaternion quaternions;
	geometry_msgs::Point point;
	//std::cout<<filename<<std::endl;
	double square_size = 1.5;
    ifstream myfile(filename);
	while(!myfile.eof())
	{
		myfile>>a>>b>>c;
        point = setPoint(a, b, 0.000);
		quaternions = setQuaternion( c );
		pose_list[i].position = point;
		pose_list[i].orientation = quaternions;
		i++;
	}
	//std::cout<<"e"<<std::endl;
	myfile.close();
}

geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
	geometry_msgs::Point m_point;
	m_point.x = _x;
	m_point.y = _y;
	m_point.z = _z;
	return m_point;
}

geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
	geometry_msgs::Quaternion m_quaternion;
	m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan);
	return m_quaternion;
}

// Shutdown
void shutdown(int sig)
{
	cmdVelPub.publish(geometry_msgs::Twist());
	ros::Duration(1).sleep(); // sleep for  a second
	ROS_INFO("move_base_square_hit.cpp ended!");
	ros::shutdown();
}

// Init markers
void init_markers(visualization_msgs::Marker *marker)
{
	marker->ns = "waypoints";
	marker->id = 0;
	marker->type = visualization_msgs::Marker::CUBE_LIST;
	marker->action = visualization_msgs::Marker::ADD;
	marker->lifetime = ros::Duration();//0 is forever
	marker->scale.x = 0.2;
	marker->scale.y = 0.2;
	marker->color.r = 1.0;
	marker->color.g = 0.7;
	marker->color.b = 1.0;
	marker->color.a = 1.0;

	marker->header.frame_id = "map";
	marker->header.stamp = ros::Time::now();
}


// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal Received");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
//	ROS_INFO("Got base_position of Feedback");
	current_point.x = feedback->base_position.pose.position.x;
	current_point.y = feedback->base_position.pose.position.y;
	current_point.z = feedback->base_position.pose.position.z;
}

double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal)
{
	double m_distance;
	m_distance = sqrt(pow(fabs(m_goal.x - m_current_point.x), 2) + pow(fabs(m_goal.y - m_current_point.y), 2));
	return m_distance;
}

void soc_callback(const indoor_navigation::soc& soc)
{
    if(soc.code == 5 && soc.action ==1)
    {
        filename="/home/x805/catkin_ws/src/indoor_navigation/scripts/guiji/guiji"+soc.filename;
		//std::cout<<filename<<std::endl;
		//std::cout<<soc.filename<<std::endl;
		socflag=1;
        
    }
    else if(soc.code == 1 || soc.code == 3 || soc.code == 4)
    {
        flag=0;
		socflag=0;
    }
	else if(soc.code == 2 && socflag==1)
	{
		flag=1;
	}
	else if(soc.code == 7 && soc.action==3)
    {
        flag=0;
		socflag=0;
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_move_base");
	ros::NodeHandle node;
    ros::Subscriber soc_sub = node.subscribe("/soc",1,soc_callback); 
	ros::Rate loop_rate(10);
	//Define a marker publisher.
	marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);
	//Publisher to manually control the robot (e.g. to stop it, queue_size=5)
	cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    //Subscribe to the move_base action server
	Client ac("move_base", true);
	//for init_markers function
	visualization_msgs::Marker  marker_list;
			
    //Intialize the waypoint goal
	move_base_msgs::MoveBaseGoal goal;
	int count=0;
    double distance=0.0;
	//Cycle through the four waypoints
	while (ros::ok())
	{
		//std::cout<<flag<<std::endl;
		if(flag)
		{
			//std::cout<<"2"<<std::endl;
			if(flagfirst)
			{
				signal(SIGINT, shutdown);
				//ROS_INFO("move_base_square.cpp start...");
				//Initialize the list of goal
				init_goalList();

				//Initialize the visualization markers for RViz
				init_markers(&marker_list);
				//Set a visualization marker at each waypoint
				for (int j = 0; j < i; j++)
				{
					marker_list.points.push_back(pose_list[j].position);
				}
				mulu0="/home/x805/catkin_ws/src/indoor_navigation/maps/chushi.txt";
    			while(!flagmulu0)
    			{
					//std::cout<<"4"<<std::endl;
        			stringstream mulu2;
        			mulu2<<mulu0;
        			if(exist(mulu2.str()))
        			{
            			ifstream myfile("/home/x805/catkin_ws/src/indoor_navigation/maps/chushi.txt");
	        			myfile>>x>>y>>yawlll;
	        			myfile.close();
            			break;
        			}
        			else
        			{
            			flagmulu0=1;
        			}
    			}

				

				ROS_INFO("Waiting for move_base action server...");

				//Wait 60 seconds for the action server to become available
				if (!ac.waitForServer(ros::Duration(60)))
				{
					ROS_INFO("Can't connected to move base server");
					return 1;
				}

				ROS_INFO("Connected to move base server");
				ROS_INFO("Starting navigation test");

				//Initialize a counter to track waypoints
				for (int j = 0; j < i; j++)
				{
					xx=pose_list[j].position.x;
					yy=pose_list[j].position.y;
					float angle=atan2(yy-y,xx-x);
					if ((angle<1)||(angle>-1))
					{
						dis=sqrt(pow(fabs(yy-y), 2) + pow(fabs(xx-x), 2));
						if(dis<mindis)
						{
							mindis=dis;
							count=j;
						}
					}
				}
				//Use the map frame to define goal poses
				goal.target_pose.header.frame_id = "map";

				//Set the time stamp to "now"
				goal.target_pose.header.stamp = ros::Time::now();

				//Set the goal pose to the i-th waypoint
				goal.target_pose.pose = pose_list[count];

				//Start the robot moving toward the goal
				ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);

				flagfirst=0;
			}
			else
			{
				//std::cout<<"5"<<std::endl;
				//Update the marker display
				marker_pub.publish(marker_list);

				distance = computeDistance(current_point, goal.target_pose.pose.position);
				//ROS_INFO("distance = %f", distance);

				if (distance <= 0.5)
				{
					if (i == count)
					{
						count = 0;
					}

					//Use the map frame to define goal poses
					goal.target_pose.header.frame_id = "map";

					//Set the time stamp to "now"
					goal.target_pose.header.stamp = ros::Time::now();

					//Set the goal pose to the i-th waypoint
					goal.target_pose.pose = pose_list[count];
					std::cout<<"cout                          "<<count<<std::endl;

					ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
					count++;
				}     
			}
			
		}
		else
		{
			//std::cout<<"6"<<std::endl;
			flagfirst=1;
			count=0;
			i=0;
			marker_list.points.clear();
		}
		ros::spinOnce(); 
		loop_rate.sleep();

	}

	//ROS_INFO("move_base_square_lx.cpp end...");
	return 0;
}
