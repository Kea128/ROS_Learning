#include "ros/ros.h"
#include "std_msgs/String.h"
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
                节点名称需要保证唯一，会导致一个问题：同一个节点不能重复启动
                结果：ros中当有重名的节点启动时，之前的节点会被关闭掉

                需求：特定场景下，需要同一节点多次启动且能正常运行，怎么办
                方法：使用options(ros::init_options::AnonymousName)解决重复启动问题
                     当启动ros节点时，会在节点名称后缀随机数，避免重名问题
    */
    ros::init(argc,argv,"erGouzi",ros::init_options::AnonymousName); // 在erGouzi后面加后缀让两个节点不重复
    ros::NodeHandle nh;
    // 4.创建发布者对象
    /*  nh.advertise
        作用：创建发布者对象
        
        模板：被发布的消息的类型
        
        参数：
            1. 话题名称
            2. 队列长度
            3. latch（可选）默认为false，如果设置为true，会保存发布方的最后一条消息，
               并且新的订阅对象连接到发布方时，发布方会将这条消息发送给订阅者

        使用：
            1. latch 设置为 true 的作用？
               以静态地图发布为例，方案1：可以用固定频率发送地图数据，但是效率底；
                                方案2：将地图发布对象的latch设置为true，并且发布方只发送一次数据，每当订阅者连接时，只发送一次地图数据
                                提高数据发送效率
    */
    ros::Publisher pub = nh.advertise<std_msgs::String>("fang", 10, true); //latch属性设置为true
    // 5.编写发布逻辑与发布数据
    // 要求以10HZ的频率发布数据，并且文本后添加编号
    // 先创建被发布的消息
    std_msgs::String msg;
    //发布频率
    ros::Rate rate(1);
    //设置编号  
    int count = 0;
    ros::Duration(3).sleep();

    // 循环中发布数据
    while (ros::ok())
    {
        count++;
        //字符串拼接数字
        std::stringstream ss;
        ss << "hello ---> "<< count;
        //msg.data="hello";
        msg.data = ss.str();
        if(count<=10)
        {
            pub.publish(msg);
            //添加日志
            ROS_INFO("发布的数据是：%s",ss.str().c_str());
        }
        
        //如果count>=20,关闭节点
        if(count>=20)
        {
            ros::shutdown();
        }
        rate.sleep();
        ros::spinOnce();//官方建议，处理回调函数、
        // spinOnce()后的语句还会执行
        ROS_INFO("——————一轮回调执行完毕——————");
    }

    return 0;
}