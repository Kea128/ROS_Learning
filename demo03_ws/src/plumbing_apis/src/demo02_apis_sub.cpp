#include "ros/ros.h"
#include "std_msgs/String.h"
/*
    订阅方实现：
    1.包含头文件
      ros中的文本类型 --->std_msgs/String.h
    2.初始化ros节点
    3.创建节点句柄
    4.创建订阅者对象
    5.处理订阅到的数据
    6.spin()函数
*/

void doMsg(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("订阅的数据:%s",msg->data.c_str());
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化ros节点
    ros::init(argc,argv,"cuiHua");
    // 3.创建节点句柄
    ros::NodeHandle nh;
    // 4.创建订阅者对象
    ros::Subscriber sub = nh.subscribe("fang",10,doMsg);
    // 5.处理订阅到的数据
    ros::spin();
    
    //spin()后的语句不会执行
    ROS_INFO("——————一轮回调执行完毕——————");
    
    return 0;
}
