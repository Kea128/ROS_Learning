//1.包含ros头文件
#include "ros/ros.h"

//2.编写main函数
int main(int argc, char *argv[])
{
	
    //3.执行 ros 节点初始化
    ros::init(argc,argv,"hello_node");
    //控制台输出 hello world
    ROS_INFO("hello world!");
    return 0;
}