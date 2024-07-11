#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"  //获取消息的集合
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "bag_read");
  ros::NodeHandle nh;

  //创建 bag 对象
  rosbag::Bag bag;
  //打开 bag 文件
  bag.open("hello.bag", rosbag::BagMode::Read);
  //读数据
  //取出话题，时间戳和消息
  //先获取消息的集合，再迭代取出消息的字段
  for (auto &&m : rosbag::View(bag)) {
    std::string topic = m.getTopic();
    ros::Time time = m.getTime();
    std_msgs::StringPtr p = m.instantiate<std_msgs::String>();
    ROS_INFO("解析内容：话题：%s,时间：%2.f,消息值：%s", topic.c_str(),
             time.toSec(), p->data.c_str());
  }

  // for (rosbag::MessageInstance const m : rosbag::View(bag))
  // {
  //     std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();
  //     if(p != nullptr){
  //         ROS_INFO("读取的数据:%s",p->data.c_str());
  //     }
  // }

  //关闭文件流
  bag.close();
  return 0;
}
