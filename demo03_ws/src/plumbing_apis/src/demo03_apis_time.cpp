#include "ros/ros.h"
#include "std_msgs/String.h"

/*
    需求1：演示时间相关操作（获取当前时刻 + 设置指定时刻）
    实现：
        1. 准备（头文件、节点初始化、Nodehandle创建）
        2. 获取当前时刻
        3. 设置指定时刻
    
    需求2：程序执行中停顿
    实现：
        1. 创建持续时间对象
        2. 休眠

    需求3：已知程序可开始运行的时刻和运行时间，求运行结束的时刻
    实现：
        1. 获取开始执行的时刻
        2. 模拟运行时间（N秒）
        3. 计算结束时刻 = 开始+运行时间
    注意：
        1. 时刻与持续时间之间可以执行加减
        2. 持续时间之间也可以加减
        3. 时刻之间只可以相减，不可以相加

    需求4：每隔1s，在控制台输出一段文本
    实现：
        法1. ros::rate()
        法2. 定时器
    注意：
        创建：nh.createTimer();
        参数1：时间间隔
        参数2：回调函数（时间事件 TimerEvent）
        参数3：是否只执行一次
        参数4：是否自动启动（当设置为false时，需要手动调用 timer.start()）
        定时器启动后要ros::spin()
*/

//回调函数
void cb(const ros::TimerEvent& event)
{
    ROS_INFO("------------");
    ROS_INFO("函数被调用的时刻：%.2f",event.current_real.toSec());
}

int main(int argc, char *argv[])
{
    // 1. 准备（头文件、节点初始化、Nodehandle创建）`
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello_time");
    ros::NodeHandle nh; //注意：必须 否则会导致时间API调用失败（在Nodehandle中会初始化时间操作）
    // 2. 获取当前时刻
    // now()函数会将当前时刻封装并返回
    // 当前时刻：now()被调用执行的时刻
    // 参考系：1970年1月1日 00:00:00
    ros::Time right_now = ros::Time::now();
    ROS_INFO("当前时刻：%.2f",right_now.toSec());//函数 toSec将时间转换为秒
    ROS_INFO("当前时刻：%d",right_now.sec);//字段

    // 3. 设置指定时刻
    ros::Time t1(20, 13241412);
    ros::Time t2(100.13241412);
    ROS_INFO("t1=%.2f",t1.toSec());//toSec将时间转换为秒
    ROS_INFO("t2=%.2f",t2.toSec());//toSec将时间转换为秒
    
    // 需求2：程序执行中停顿
    // 实现：
    // 1. 创建持续时间对象
    ros::Duration du(4.5);
    // 2. 休眠
    du.sleep();

    // 需求3：已知程序可开始运行的时刻和运行时间，求运行结束的时刻
    // 实现：
    // 时刻与持续时间运算
        // 1. 获取开始执行的时刻
    ros::Time begin = ros::Time::now();
        // 2. 模拟运行时间（N秒）
    ros::Duration du1(5);
        // 3. 计算结束时刻 = 开始+运行时间
    ros::Time end = begin + du1;
    
    ROS_INFO("开始时刻：%.2f",begin.toSec());
    ROS_INFO("结束时刻：%.2f",end.toSec());

    //时刻与时刻运算
    //ros::Time sum = begin + end; //时刻不可以相加
    ros::Duration du2 = begin - end; //时刻可以相减，返回值为Duration类型
    ROS_INFO("时刻相减：%.2f",du2.toSec());
    
    //持续时间与持续时间之间的运算、
    ros::Duration du3 = du1 + du2;
    ros::Duration du4 = du1 - du2;
    ROS_INFO("du1 + du2：%.2f",du3.toSec());
    ROS_INFO("du1 - du2：%.2f",du4.toSec());
    
    ROS_INFO("---------------定时器----------------");
    /*
        ros::Timer createTimer(ros::Duration period,            //时间间隔
                            const ros::TimerCallback &callback, //回调函数
                            bool oneshot = false,               //是否一次性（ture：1s执行一次回调函数，只执行一次）（false：每隔1s执行一次，循环执行）
                            bool autostart = true)              //true为自动启动，false为手动启动
    */
    
    // ros::Timer timer = nh.createTimer(ros::Duration(1),cb);
    // ros::Timer timer = nh.createTimer(ros::Duration(1),cb,true);
    ros::Timer timer = nh.createTimer(ros::Duration(1),cb,false,false);
    timer.start();//手动启动
    ros::spin();//需要回旋
    
    return 0;        
}
