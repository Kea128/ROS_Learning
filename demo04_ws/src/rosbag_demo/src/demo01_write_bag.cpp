#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h" //普通字符串


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"bag_write");
    ros::NodeHandle nh;
    //创建bag对象
    rosbag::Bag bag;
    //打开文件流
    bag.open("hello.bag",rosbag::BagMode::Write);
    //写数据
    std_msgs::String msg;
    msg.data = "hello world";
    bag.write("/chatter",ros::Time::now(),msg);
    bag.write("/chatter",ros::Time::now(),msg);
    bag.write("/chatter",ros::Time::now(),msg);
    bag.write("/chatter",ros::Time::now(),msg);
    //关闭文件流
    bag.close();

    return 0;
}
