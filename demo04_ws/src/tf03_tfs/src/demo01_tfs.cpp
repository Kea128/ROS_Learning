#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"//创建订阅对象
#include "tf2_ros/buffer.h"//创建buffer
#include "geometry_msgs/PointStamped.h"//创建坐标点
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"//坐标点从一个坐标转换到另一个坐标要用到
#include "geometry_msgs/TransformStamped.h"//创建坐标变换关系

/*
    订阅方实现:1.计算son1与son2的相对关系；2.计算son1中的某个点在son2中的坐标值
    实现流程:
        1.包含头文件
        2.编码，初始化 ROS 节点，NodeHandle
        3.创建 TF 订阅对象(订阅坐标系相对关系)
        4.编写解析逻辑
        6.spinOnce()
*/

int main(int argc, char *argv[])
{
    // 2.编码，初始化 ROS 节点，NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tfs_sub");
    ros::NodeHandle nh;
    // 3.创建 TF 订阅对象(订阅坐标系相对关系)
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);
    // 4.编写解析逻辑 

    //在son1下 创建一个坐标点 
    geometry_msgs::PointStamped psAtSon1;
    psAtSon1.header.stamp = ros::Time::now();
    psAtSon1.header.frame_id = "son1";
    psAtSon1.point.x=1.0;
    psAtSon1.point.y=2.0;
    psAtSon1.point.z=3.0;

    ros::Rate rate(10);
    while (ros::ok())
    {
        //核心
        try
        {
            // 1.计算son1与son2的相对关系；(son1（源）相对于son2（目标），时间ros::Time(0)，表示查找时间差最小的相对关系)
            // 创建son1相对于son2点坐标变换关系
            geometry_msgs::TransformStamped son1ToSon2 = buffer.lookupTransform("son2","son1",ros::Time(0)); //ros::Time(0)查找时间最近的两个坐标系的关系
            ROS_INFO("son1相对于son2点坐标变换关系:父级:%s,子级:%s, 位移(%.2f,%.2f,%.2f)", 
                    son1ToSon2.header.frame_id.c_str(),
                    son1ToSon2.child_frame_id.c_str(),
                    son1ToSon2.transform.translation.x,
                    son1ToSon2.transform.translation.y,
                    son1ToSon2.transform.translation.z);


            // 2.计算son1中的某个点在son2中的坐标值
            geometry_msgs::PointStamped psAtson2 = buffer.transform(psAtSon1,"son2");
            ROS_INFO("坐标点在son2中的值(%.2f,%.2f,%.2f)",
                    psAtson2.point.x,
                    psAtson2.point.y,
                    psAtson2.point.z);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("错误提示：%s",e.what());
        }
        

        rate.sleep();
        ros::spinOnce();
    }
    
    // 6.spinOnce()
    return 0;
}

