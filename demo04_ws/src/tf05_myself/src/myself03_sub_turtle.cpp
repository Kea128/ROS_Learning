#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/TransformStamped.h" //坐标转换消息类型
#include "geometry_msgs/Twist.h"    //速度消息类型


// 订阅两个坐标转换消息，并处理（turtle1/2相对于word的transform，转换为turtle1相对于turtle2的关系）

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "subTFs");
    ros::NodeHandle nh;

    // 创建TF订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf2_sub(buffer);

    // 创建第二只乌龟的速度发布对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/velocity_02", 10);

    ros::Rate rate(10);
    while (ros::ok())
    {
        try
        {
            //创建一个坐标转换消息
            static geometry_msgs::TransformStamped _1To2;
            _1To2 = buffer.lookupTransform("body/pose_02","body/pose_01",ros::Time(0));
            
            ROS_INFO("pose_01相对于pose_02的关系:位移(%.2f,%.2f,%.2f)", 
                    _1To2.transform.translation.x,
                    _1To2.transform.translation.y,
                    _1To2.transform.translation.z);

            float distance = sqrt(pow(_1To2.transform.translation.x,2) + pow(_1To2.transform.translation.y,2)); 
            float yawError = atan2(_1To2.transform.translation.y, _1To2.transform.translation.x);

            static geometry_msgs::Twist followTurtleVel;
            followTurtleVel.linear.x = 0.5 * distance;
            followTurtleVel.angular.z = 2 * yawError;
            pub.publish(followTurtleVel);  
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_WARN("错误信息：%s",e.what());
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
