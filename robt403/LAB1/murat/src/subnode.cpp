#include "ros/ros.h"
#include "std_msgs/Int32.h"

void Callback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("ID: %d", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("murat", 1000, Callback);
    ros::spin();
    return 0;
}
