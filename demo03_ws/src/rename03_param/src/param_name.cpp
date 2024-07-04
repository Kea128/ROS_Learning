#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello"); 

    /* 
        使用 ros::param 来设置参数 
    */

    ros::NodeHandle nh;
    // 1. 全局参数的设置：需要以"/"开头（也可以设置自己的命名空间），与节点（命名空间）没有关系
    ros::param::set("/radiusA",100);
    // 2. 相对参数：非"/"开头，参数在节点的命名空间下，和节点名称平级
    ros::param::set("radiusA",100);
    // 3. 私有参数：参数在节点名称下
    ros::param::set("~radiusA",100);
    
    /* 
        使用 NodeHandle 来设置参数 
    */

    //全局
    nh.setParam("/radius_nh_B",100);
    //相对
    nh.setParam("radius_nh_B",100);
    //私有
    ros::NodeHandle nh_private("~");
    nh_private.setParam("radius_nh_B",100);
    
    return 0;
}

