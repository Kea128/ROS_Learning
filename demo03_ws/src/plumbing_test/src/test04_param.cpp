#include "ros/ros.h"
// #include "turtlesim/Spawn.h"

/*
    需求：修改参数服务器中turtlesim背景色相关的参数       
        1.包含头文件
        2.初始化ros节点
        3.不一定要创建节点句柄，和后续API有关
        4.修改参数
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化ros节点
    ros::init(argc,argv,"change_bkColor");
    // 3.不一定要创建节点句柄，和后续API有关
    // 3.1. 如果调用ros::param，不需要创建句柄
    // ros::param::set("/turtlesim/background_r",0);
    // ros::param::set("/turtlesim/background_g",0);
    // ros::param::set("/turtlesim/background_b",0);  
    // 3.2. 借助句柄实现
    ros::NodeHandle nh("turtlesim");//调用构造函数，turtlesim为命名空间
    //或 ros::NodeHandle nh;
    //   nh.setParam("/turtlesim/background_r",255);
    nh.setParam("background_r",255);
    nh.setParam("background_g",255);
    nh.setParam("background_b",255);
    return 0;
}
