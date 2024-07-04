#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

#define pi 3.1415926

/* 
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建静态坐标转换广播器
        4.创建坐标系信息
        5.广播器发布坐标系信息
        6.spin()
*/


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_pub");
    ros::NodeHandle nh;
    // 创建静态坐标转换广播器（创建发布对象）
    tf2_ros::StaticTransformBroadcaster pub; //依赖于静态坐标转换广播器"tf2_ros/static_transform_broadcaster.h"
    // 创建坐标系信息（组织发布消息）//消息类型依赖"geometry_msgs/TransformStamped.h"
    geometry_msgs::TransformStamped tfs; 
    tfs.header.stamp=ros::Time::now();
    tfs.header.frame_id="base_link";//相对坐标系关系中被参考的那个(基坐标系) 
    tfs.child_frame_id="laser";//雷达坐标系
    tfs.transform.translation.x=0.2;
    tfs.transform.translation.y=0.0;
    tfs.transform.translation.z=0.5;
    //根据欧拉角转换为四元数
    tf2::Quaternion qtn; //四元数对象 依赖于"tf2/LinearMath/Quaternion.h"
    //向该对象设置欧拉角，这个对象可以将欧拉角转换为四元数
    qtn.setRPY(0*pi/180,0,0);//单位rad

    tfs.transform.rotation.x=qtn.getX();
    tfs.transform.rotation.y=qtn.getY();
    tfs.transform.rotation.z=qtn.getZ();
    tfs.transform.rotation.w=qtn.getW();
    //发布数据
    pub.sendTransform(tfs); //这里不是publish，而是sendTransform
    //spin()
    ros::spin();
    return 0;
}
 