#include "ros/ros.h"
#include "std_msgs/String.h"



int main(int argc, char *argv[])
{
    //2.初始化ROS节点；
    ros::init(argc,argv,"publisher");
    //3.创建节点句柄；
    ros::NodeHandle nh;
    //4.创建发布者对象
    ros::Publisher pub = nh.advertise<std_msgs::String>("huati",12);
    //5.编写发布逻辑并发布数据
    //先创建被发布的消息
    std_msgs::String msg;
    //编写循环，循环中发布数据
    while (ros::ok())
    {
        msg.data = "Hello subscriber!";
        pub.publish(msg);
    }
return 0;
}