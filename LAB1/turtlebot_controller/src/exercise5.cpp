#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"


ros::Publisher pub;
ros::ServiceClient client_spawn, client_kill, client_teleport, client_clear;

void moveStraight(double speed, double distance)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = speed;
    vel_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(100);
    while (current_distance < distance)
    {
        pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        loop_rate.sleep();
    }

    vel_msg.linear.x = 0;
    pub.publish(vel_msg);
}


void rotate(double angular_speed, double angle)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = angular_speed;

    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;

    ros::Rate loop_rate(100);
    while (current_angle <= angle)
    {
        pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        loop_rate.sleep();
    }


    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "turtlebot_controller");
    ros::NodeHandle nh;


    pub = nh.advertise<geometry_msgs::Twist>("turtle_dima/cmd_vel", 10);
    client_spawn = nh.serviceClient<turtlesim::Spawn>("/spawn");
    client_kill = nh.serviceClient<turtlesim::Kill>("/kill");
    client_teleport = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle_dima/teleport_absolute");
    client_clear = nh.serviceClient<std_srvs::Empty>("/clear");

    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";
    client_kill.call(kill_srv);


    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.5;
    spawn_srv.request.y = 5.5;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle_dima";
    client_spawn.call(spawn_srv);

    ROS_INFO("Pause to show that initially it was spawned at center");
    ros::Duration(1.5).sleep(); 

    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.x = 0.5;
    teleport_srv.request.y = 0.5;
    teleport_srv.request.theta = 0.0;
    client_teleport.call(teleport_srv);
    

    std_srvs::Empty clear_srv;
    client_clear.call(clear_srv);
    

    double speed = 2.0;
    double distance = 10;
    double angular_speed = 1.559; // Best approximation for 90 degree turn
    double angle = 1.559;

    for (int i = 0; i < 4; ++i)
    {
        moveStraight(speed, distance);  
        rotate(angular_speed, angle);   
    }
    ROS_INFO("Pause before clearing the square path, to start triangular path");
    ros::Duration(2.0).sleep(); 

    client_teleport.call(teleport_srv);
    client_clear.call(clear_srv);
    for (int i = 0; i < 2; ++i)
    {
        moveStraight(speed, distance);  
        rotate(angular_speed, angle);   
    }
  
    distance = 14.2;
    angular_speed = 0.77;  //best approximation for turning 135
    angle = 0.77;
    rotate(angular_speed, angle);
    moveStraight(speed, distance);  

    ros::spin();
    return 0;
}
