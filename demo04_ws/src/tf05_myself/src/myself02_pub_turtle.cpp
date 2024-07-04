#include "ros/ros.h"
#include "turtlesim/Pose.h" //turtle pose的消息类型
#include "tf2_ros/transform_broadcaster.h" //创建坐标转换动态广播器依赖
#include "geometry_msgs/TransformStamped.h" //坐标转换关系消息的依赖
#include "tf2/LinearMath/Quaternion.h"  //封装坐标转换关系时使用四元数

// 订阅乌龟位姿消息，发布坐标转换关系

std::string poseName;

void callBack(const turtlesim::Pose::ConstPtr &pose){
    // static tf2_ros::TransformBroadcaster pub;
    static tf2_ros::TransformBroadcaster tf2_pub;   //创建动态坐标转换器
    static geometry_msgs::TransformStamped tf2_msgs;   //坐标转换关系的消息
    
    tf2_msgs.header.frame_id = "world";
    tf2_msgs.header.stamp = ros::Time::now();
    tf2_msgs.child_frame_id = "body"+poseName;
    tf2_msgs.transform.translation.x = pose->x; 
    tf2_msgs.transform.translation.y = pose->y; 
    tf2_msgs.transform.translation.z = 0;

    tf2::Quaternion qtn; //创建qtn类型
    qtn.setEulerZYX(pose->theta,0,0); //将当前乌龟的pose设置到qtn中，采用zyx欧拉角
    tf2_msgs.transform.rotation.x = qtn.getX();
    tf2_msgs.transform.rotation.y = qtn.getY();
    tf2_msgs.transform.rotation.z = qtn.getZ();
    tf2_msgs.transform.rotation.w = qtn.getW();

    tf2_pub.sendTransform(tf2_msgs);
    // ROS_INFO("%s:已发布一次",poseName.c_str());
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "pubTFs");
    if(argc!=2){
        ROS_WARN("*** please enter ONE parameter ***");
        return 1;
    }
    else{
        poseName = argv[1];
    }
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(poseName,10,callBack);
    ROS_INFO("已启动");
    ros::spin();
    return 0;
}
