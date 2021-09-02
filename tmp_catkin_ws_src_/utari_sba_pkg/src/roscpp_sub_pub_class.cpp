#include "ros/ros.h"
#include "std_msgs/String.h"


class myclass
{
public: // public vars
	
private: // private vars 
	ros::NodeHandle * n;
	ros::Subscriber sub;
	
public: // public methods
	myclass(ros::NodeHandle nh) // constructor
	{
		n = &nh;
		sub = nh.subscribe("chatter", 10, &myclass::callback, this);
		
	}
	
private: // private methods
	void callback(const std_msgs::String::ConstPtr& msg)
	{
	  ROS_INFO("I heard: [%s]", msg->data.c_str());
	}
	

};




int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_class");
	ros::NodeHandle nh;
	
	myclass MC = myclass(nh);
	
	ros::spin();
	
	return 0;
	
}
