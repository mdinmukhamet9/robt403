#include "ros/ros.h"
#include "turtlesim/Pose.h"
//Line was added for exercise 3
#include "geometry_msgs/Twist.h"
//This line was added for exercise 4
#include "turtlesim/Spawn.h"

//global variable pub
ros::Publisher pub;
ros::ServiceClient client1;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",
	msg->x, msg->y, msg->theta);

    //following lines of code was added for exercise 3
    geometry_msgs::Twist my_vel;
    my_vel.linear.x=1.0;
    my_vel.angular.z=1.0;
    pub.publish(my_vel);

    //following lines of code was added for exercise 4
    turtlesim::Spawn srv1;
    srv1.request.x = 1.0;
    srv1.request.y = 5.0;
    srv1.request.theta =0.0;
    srv1.request.name = "Turtle_Dima";
    client1.call(srv1);
}


int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS
	//system
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;

    //line was added for exercise 3
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
	// Define the subscriber to turtle's position
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1,turtleCallback);
	ros::spin();
	return 0;
}
