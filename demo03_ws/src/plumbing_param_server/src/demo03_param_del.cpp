#include "ros/ros.h"

/*
    实现参数删除
    实现：
        ros::NodeHandle
            deleteParam()
        ros::param
            del()
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); 
    // 2.初始化ros节点
    ros::init(argc,argv,"param_del_c");//节点名称需要保持唯一
    // 3.创建节点句柄
    ros::NodeHandle nh;
   
    // 4.参数删
    // 方案1 ros::NodeHandle
    bool flag1 = nh.deleteParam("radius");
    if(flag1)
    {
        ROS_INFO("删除成功");
    }
    else
    {
        ROS_INFO("删除失败");
    }
    // 方案2 ros::param
    bool flag2 = ros::param::del("radius_param");
    if(flag2)
    {
        ROS_INFO("radius_param删除成功");
    }
    else
    {
        ROS_INFO("radius_param删除失败");
    }

    return 0;
}
