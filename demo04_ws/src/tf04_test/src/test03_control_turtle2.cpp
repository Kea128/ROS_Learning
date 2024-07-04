#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"//创建订阅对象
#include "tf2_ros/buffer.h"//创建buffer
#include "geometry_msgs/PointStamped.h"//创建坐标点
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"//坐标点从一个坐标转换到另一个坐标要用到
#include "geometry_msgs/TransformStamped.h"//创建坐标变换关系
#include "geometry_msgs/Twist.h" //要发布的速度消息的类型

/*
    需求1：换算出turtle1相对于turtle2的关系
    需求2：计算角速度和线速度并发布
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
    
    //A.创建发布对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",100);
    
    // 4.编写解析逻辑 
    ros::Rate rate(10);
    while (ros::ok())
    {
        //核心
        try
        {
            // 1.计算son1与son2的相对关系；(son1（源）相对于son2（目标），时间ros::Time(0)，表示查找时间差最小的相对关系)
            // 创建son1相对于son2点坐标变换关系
            geometry_msgs::TransformStamped son1ToSon2 = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));
            // ROS_INFO("turtle1相对于turtle2坐标变换关系:父级:%s,子级:%s, 位移(%.2f,%.2f,%.2f)", 
            //         son1ToSon2.header.frame_id.c_str(), //turtle2
            //         son1ToSon2.child_frame_id.c_str(),  //turtle1
            //         son1ToSon2.transform.translation.x,
            //         son1ToSon2.transform.translation.y,
            //         son1ToSon2.transform.translation.z);

            //B.根据相对关系计算并组织速度消息
            geometry_msgs::Twist twist;
            twist.linear.x=0.5*sqrt(pow(son1ToSon2.transform.translation.x,2)+pow(son1ToSon2.transform.translation.y,2));
            twist.angular.z=4*atan2(son1ToSon2.transform.translation.y,son1ToSon2.transform.translation.x);
            //C.发布
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            ROS_INFO("错误提示：%s",e.what());
        }
        

        rate.sleep();
        ros::spinOnce();
    }
    


    // 6.spinOnce()
    return 0;
}

