#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "pub");
    ros::NodeHandle n;
    ros::Publisher id_pub = n.advertise<std_msgs::Int32>("murat", 1000);
    ros::Rate loop_rate(1);
    int id[] = {2, 0, 1, 9, 1, 7, 5, 2, 6};
    int counter = 0;
    int val = (sizeof(id)/sizeof(int));

    while (ros::ok()){
        // for (counter; counter<val;counter++)
        // {
        std_msgs::Int32 msg;
        msg.data = id[counter];
        id_pub.publish(msg);
        ros::spinOnce();
        ros::Rate loop_rate(50); //I changed it from 1 to 50 Hz for 2nd part of task
        loop_rate.sleep();
        //}
        //ROS_INFO("////");
        counter = (counter+1) % val;

    }
    return 0;
}
 