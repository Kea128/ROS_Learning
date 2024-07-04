#include "ros/ros.h"
#include "std_msgs/String.h" //ros中的文本类型 --->std_msgs/String.h
#include <sstream>
/*
    发布方实现：
    1.包含头文件
      ros中的文本类型 --->std_msgs/String.h
    2.初始化ros节点
    3.创建节点句柄
    4.创建发布者对象
    5.编写发布逻辑与发布数据
*/


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化ros节点
    /*
        作用：ros初始化函数

        参数：
            1.argc      封装实参的个数（调用时传入了n个参数，实际封装个数n+1，+1是文件自身）
            2.argv      封装参数的数组（字符串数组）
            3.name      为节点命名（唯一性）
            4.options   节点启动选项
            返回值：void
        使用：
            1.argc与argv的使用
                如果按照ros中的特点格式传入实参，那么ros可以加以使用，比如用来设置全局参数，给节点重命名。。。
            2.options的使用
                节点名称需要保持唯一，导致同一个节点不能重复启动。
                结果：ROS中当有重名节点启动时，之前的节点会被关闭
                需求：特点场景下需要节点多次启动，使用options
    */
    ros::init(argc,argv,"erGouzi"); 
    // 3.创建节点句柄
    ros::NodeHandle nh;
    // 4.创建发布者对象
    ros::Publisher pub = nh.advertise<std_msgs::String>("fang", 10);
    // 5.编写发布逻辑与发布数据
    // 要求以10HZ的频率发布数据，并且文本后添加编号
    // 先创建被发布的消息
    std_msgs::String msg;
    //发布频率
    ros::Rate rate(1); //构造函数中为数字，单位HZ，后续循环中调用rate.sleep();
    //设置编号
    int count = 0;  
    
    ros::Duration(3).sleep();//在这里休眠3s,让pub现在master先注册再发布将

    // 循环中发布数据
    while (ros::ok()) //如果节点还活着
    {
        count++;
        
        //字符串拼接数字
        std::stringstream ss;
        ss << "hello ---> "<< count;
        
        //msg.data="hello";
        msg.data = ss.str(); //将流里面数据提取为字符串
        pub.publish(msg);
        
        //添加日志
        ROS_INFO("发布的数据是：%s",ss.str().c_str());
        rate.sleep();

        ros::spinOnce();//官方建议处理回调函数，写在循环中，每次循环处理一次
    }

    return 0;
}