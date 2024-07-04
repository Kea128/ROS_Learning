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

// const std_msgs::String::ConstPtr &msg 是函数的参数列表。
// 这里定义了一个参数 msg，它是一个对 std_msgs::String 类型的 ConstPtr 智能指针的常量引用。
// 这意味着 msg 是一个不可变的引用，指向一个 std_msgs::String 类型的对象。

void doMsg(const std_msgs::String::ConstPtr &msg)
{
    // %s：C风格的字符串
    ROS_INFO("订阅的数据:%s",msg->data.c_str());
    
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); //防止中文乱码
    // 2.初始化ros节点
    ros::init(argc,argv,"cuiHua");
    // 3.创建节点句柄
    ros::NodeHandle nh;
    // 4.创建订阅者对象 并 处理订阅到的数据
    ros::Subscriber sub = nh.subscribe("fang",10, doMsg);
    // 5.处理订阅到的数据
    ros::spin();

    return 0;
}
