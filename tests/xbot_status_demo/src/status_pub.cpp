#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xbot_status");	//node
	ros::NodeHandle n;
	ros::Publisher status_pub = n.advertise<std_msgs::Int16>("status", 1000);//topic
	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok())
	{
		std_msgs::Int16 battery;
		std_msgs::Int16 temperature;
		std_msgs::Int16 speed;
		battery.data = count;
		temperature.data = count + 2;
		speed.data = count + 3;
		
		ROS_INFO("bat: %d", battery.data);
		status_pub.publish(battery);
		status_pub.publish(temperature);
		status_pub.publish(speed);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
