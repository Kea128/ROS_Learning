#include "ros/ros.h"

/*
    实现参数的新增与修改
    需求：首先设置机器人的共享参数，类型，半径(0.15)
        再修改半径(0.2m)
    实现：
        ros::NodeHandle
            setParam("键",值)
        ros::param
            set("键","值")
    修改，继续调用，覆盖
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); 
    // 2.初始化ros节点
    ros::init(argc,argv,"set_param_c");//节点名称需要保持唯一
    // 3.创建节点句柄
    ros::NodeHandle nh;
    // 4.参数增
    // 方案1
    nh.setParam("type","xiaoHuang");
    nh.setParam("radius",0.15);

    // 方案2
    ros::param::set("type_param","xiaoBai");
    ros::param::set("radius_param",0.15);

    // 5.参数改
    // 方案1
    nh.setParam("radius",0.2);
    // 方案2
    ros::param::set("radius_param",0.25);
    return 0;
}
