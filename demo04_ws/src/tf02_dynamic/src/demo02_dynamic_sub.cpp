#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
/*

    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点，NodeHandle（必须）
        3.创建 TF 订阅对象(订阅坐标系相对关系)
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spinOnce()
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_sub");
    ros::NodeHandle nh;

    // 3. 创建 TF 订阅对象(订阅坐标系相对关系)
    // 3.1 创建一个buffer缓存（缓存）
    tf2_ros::Buffer buffer; //动态坐标变换buffer里面有许多个值，包含了多个时间戳
    // 3.2 再创建监听对象(监听对象可以将订阅的数据存入buffer
    tf2_ros::TransformListener listener(buffer); //两个坐标间关系存在buffer中

    // 4.生成一个坐标点数据(相对于子级坐标系)
    geometry_msgs::PointStamped ps;
    ps.header.frame_id="turtle1";
    //时间戳
    ps.header.stamp=ros::Time(0.0);//时间不能用now(),动态坐标变换时buffer里面有许多个值，包含了多个时间戳，坐标转换时会比对时间戳
    ps.point.x=2.0;
    ps.point.y=3.0;
    ps.point.z=5.0;

    //解决1.添加休眠
    // ros::Duration(2).sleep();

    // 5.转换坐标点(相对于父级坐标系)
    ros::Rate rate(10);
    while (ros::ok())
    {
        //核心：将ps转换成相对于base_link的坐标点
        geometry_msgs::PointStamped ps_out;
        /*
            调用了buffer的转换函数transform
            参数1：被转换的座标点
            参数2：目标座标点
            返回值：输出座标点

            注意1：调用时必须包含头文件"tf2_geometry_msgs/tf2_geometry_msgs.h"
            注意2: 出现base_link不存在的问题
                原因：订阅数据是耗时操作，可能在调用transform转换函数时，坐标系的相对关系还没有订阅到，
                    因此出现异常
                解决：
                    1.在调用转换函数前执行休眠
                    2.进行异常处理
        */
        // 解决2.try捕获异常
        try
        {
            ps_out=buffer.transform(ps,"world"); //动态坐标变换时buffer里面有许多个值，包含了多个时间戳，坐标转换时会比对时间戳
            //最后输出
            ROS_INFO("转换后的坐标值：(%.2f,%.2f,%.2f),参考的坐标系：%s",
                    ps_out.point.x,
                    ps_out.point.y,
                    ps_out.point.z,
                    ps_out.header.frame_id.c_str());              
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            ROS_INFO("异常消息:%s",e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
