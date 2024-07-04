#include "ros/ros.h"
#include "plumbing_head_src/hello.h"
using namespace hello_ns;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello_head_src");

    MyHello myHello;
    myHello.run();

    return 0;
}
