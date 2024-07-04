#include "ros/ros.h"

int main(int argc, char *argv[])
{
    /* code */
    setlocale(LC_ALL,"");//解决乱码问题
    ros::init(argc,argv,"hello_c");//"hello_c"为节点名称
    ROS_INFO("hello,哈哈哈");
    return 0;
}
