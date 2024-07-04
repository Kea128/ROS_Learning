#include "ros/ros.h"
// 创建功能包时添加依赖geometry_msgs
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#define PI 3.1415926
/*
    需求：发布速度消息(使小乌龟走矩形)
        话题：/turtle1/cmd_vel
        消息：geometry_msgs/Twist
    1.包含头文件
    2.初始化ros节点
    3.创建节点句柄
    4.创建发布对象
    5.发布逻辑
    6.spinOnce();
*/

class Pose
{
public:
    // 构造函数
    Pose(float x, float y, float yaw) : m_posX(x), m_posY(y), m_yaw(){}
    
    // 设置成员变量
    void setParam(float x, float y, float yaw){
        m_posX = x;
        m_posY = y;
        m_yaw  = yaw;
    }
    // 访问成员变量
    float showX(){
        return m_posX;
    }
    float showY(){
        return m_posY;
    }
    float showYaw(){
        return m_yaw;
    }
    
    //计算两点间距离
    float cal_dis(Pose &p2){
        return sqrt((this->m_posX - p2.m_posX)*(this->m_posX - p2.m_posX) + (this->m_posY - p2.m_posY)*(this->m_posY - p2.m_posY));
    }
private:
    float m_posX;
    float m_posY;
    float m_yaw;
};

static Pose turtlePose(0,0,0);
void poseCallBack(const turtlesim::Pose::ConstPtr &msg){
    turtlePose.setParam(msg->x, msg->y, msg->theta); 
    // ROS_INFO("posX = %f, posY = %f, yaw = %f", posX, posY, yaw);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化ros节点
    ros::init(argc,argv,"my_circle_control");
    // 3.创建节点句柄
    ros::NodeHandle nh;
    // 4.创建发布对象，发布turtle速度
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
    // 5.发布逻辑10HZ
    ros::Rate rate(50);//设置发布频率
    // 6.创建订阅对象，订阅turtle姿态
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",10,poseCallBack);
    
    //组织发布消息
    geometry_msgs::Twist twist;
    //初始消息
    twist.linear.x=0.0;
    twist.linear.y=0.0;
    twist.linear.z=0,0;
    twist.angular.x=0.0;
    twist.angular.y=0.0;
    twist.angular.z=0.0;
    pub.publish(twist);
    ros::Duration(0.3).sleep();//在这里休眠1s，等待订阅消息到来
    ros::spinOnce();
    Pose lastPose = turtlePose;
    ROS_INFO("初始位姿态: x=%.2f, y=%.2f, yaw=%.2f",lastPose.showX(), lastPose.showY(), lastPose.showYaw());  
    // 乌龟状态
    int turtleState = 0;
    // 循环发布速度消息
    while(ros::ok())
    {
        ros::spinOnce();
        if(turtleState==0){
            pub.publish(twist);
            ros::Duration(2).sleep();
            turtleState = 1;
            lastPose = turtlePose;
        }
        else if(turtleState==1){
            twist.linear.x=0.5;
            pub.publish(twist);
            if(turtlePose.cal_dis(lastPose)>=2){
                twist.linear.x=0.0;
                pub.publish(twist);
                turtleState=2;
                lastPose = turtlePose;
            }
        }
        else if(turtleState==2){
            twist.angular.z=1;
            pub.publish(twist);
            if(abs(turtlePose.showYaw()-lastPose.showYaw())>=90*PI/180){
                twist.angular.z=0.0;
                pub.publish(twist);
                turtleState=1;
                lastPose = turtlePose;
            }
        }
        // pub.publish(twist);
        //休眠
        // rate.sleep();

        // ros::spin();
    }
    // 6.spinOnce();
    return 0;

}
