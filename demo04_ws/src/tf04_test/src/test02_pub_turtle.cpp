#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"//用来发布动态坐标关系 发布对象创建
#include "geometry_msgs/TransformStamped.h"//被发布的数据 坐标变换数据
#include "tf2/LinearMath/Quaternion.h"

/*  
    动态的坐标系相对姿态发布(一个坐标系相对于另一个坐标系的相对姿态是不断变动的)

    需求: 启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘
    控制乌龟运动，将两个坐标系的相对位置动态发布

    实现分析:
        1.乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
        2.订阅(话题：turtle1/pose),可以获取(消息：/turtlesim/Pose)乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
        3.将 pose 信息转换成 坐标系相对信息并发布

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建订阅对象
        5.回调函数处理订阅到的数据(实现TF广播)
            5-1.创建 TF 广播器
            5-2.创建 广播的数据(通过 pose 设置)
            5-3.广播器发布数据
        6.spin
*/

//
std::string turtle_name;

void doPose(const turtlesim::Pose::ConstPtr& pose)
{
    //获取位姿信息，转换成坐标系相对关系（核心），并发布
    //1.创建发布对象
    static tf2_ros::TransformBroadcaster pub;//static 每次都使用同一个发布对象
    //2.组织被发布的数据
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id="world";//父坐标系
    ts.header.stamp=ros::Time::now();
    // 关键点2：turtle1或turtle2动态传入
    ts.child_frame_id=turtle_name;
    //体坐标系相对基系坐标
    ts.transform.translation.x=pose->x;
    ts.transform.translation.y=pose->y;
    ts.transform.translation.z=0;
    //体坐标系相对基系姿态（四元数表示）
    /*
        位姿信息中没有四元数，但是有偏航，且无翻滚与俯仰
        欧拉角 0 0 theta
    */
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    ts.transform.rotation.x=qtn.getX();
    ts.transform.rotation.y=qtn.getY();
    ts.transform.rotation.z=qtn.getZ();
    ts.transform.rotation.w=qtn.getW();

    //3.发布
    pub.sendTransform(ts);
}

int main(int argc, char *argv[])
{ 
    setlocale(LC_ALL,"");
    // 2.设置编码，初始化，NodeHandle
 
    ros::init(argc,argv,"dynamic_pub");

    /*
        解析launch文件，通过args传入的参数
    */
    if(argc!=2)
    {
        ROS_INFO("请传入一个参数");
        return 1;
    }
    else
    {
        turtle_name=argv[1];
    }

    ros::NodeHandle nh;
    // 3.创建订阅对象，订阅 /turtle1&2/pose
    // 关键点1：订阅话题的名称，turtle1或turtle2动态传入
    ros::Subscriber sub = nh.subscribe(turtle_name+"/pose",100,doPose); 
    // 4.回调函数处理订阅的消息：将位姿信息转换成坐标相对关系并发布
    // 5.spin()
    ros::spin();
    return 0;
}
