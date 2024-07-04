#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello"); 

    // 核心：设置不同类型的话题
    // ros::NodeHandle nh;
    // 1. 全局话题的设置：话题名称需要以"/"开头（也可以设置自己的命名空间），与节点（命名空间）没有关系
    // ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter",100); // /chatter在更目录下
    // ros::Publisher pub = nh.advertise<std_msgs::String>("/yyy/chatter",100); // /yyy/chatter在更目录下
    
    // 2. 相对话题：非"/"开头，话题在节点的命名空间下，和节点名称平级
    // ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",100); // chatter在节点的命名空间下
    // ros::Publisher pub = nh.advertise<std_msgs::String>("yyy/chatter",100); // yyy/chatter在节点的命名空间下

    // 3. 私有话题：需要创建特定Nodehandle(见下)，话题在节点名称下
    // 注意：当使用~,而话题名称有时/开头时，那么话题是全局的
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",100); // chatter在节点名称下
    // ros::Publisher pub = nh.advertise<std_msgs::String>("yyy/chatter",100); // yyy/chatter在节点名称下

    
    while (ros::ok())
    {

    }

    return 0;
}