#include "ros/ros.h"
#include "turtlesim/Spawn.h"  //包含该产卵话题的消息

/*
    需求：向服务端发送请求，生成一只新乌龟
        话题：/spawn
        消息：turtlesim/Spawn

        1.包含头文件
        2.初始化ros节点
        3.创建节点句柄
        4.创建客户端对象
        5.组织数据并发送
        6.处理响应
*/

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");

  // 2.初始化ros节点
  ros::init(argc, argv, "service_call");
  // 3.创建节点句柄
  ros::NodeHandle nh;
  // 4.创建客户端对象
  // "/spawn"为话题， turtlesim::Spawn为消息类型
  ros::ServiceClient client =
      nh.serviceClient<turtlesim::Spawn>("/TURTLE/spawn");
  // 5.组织数据并发送
  // 5.1. 组织请求数据
  turtlesim::Spawn spawn;
  spawn.request.x = 2.0;
  spawn.request.y = 5.0;
  spawn.request.theta = 1.57;
  spawn.request.name = "turtle2";
  // 5.2. 发送请求
  // 先判断服务器状态（等待服务器启动）:以下两种方式
  // ros::service::waitForService("/spawn");
  client.waitForExistence();
  bool flag =
      client.call(spawn);  // flag接收响应状态，响应结果也会被设置进spawn对象

  // 6.处理响应
  if (flag) {
    ROS_INFO("乌龟生成成功，新乌龟叫%s", spawn.response.name.c_str());
  } else {
    ROS_INFO("请求失败");
  }
  return 0;
}
