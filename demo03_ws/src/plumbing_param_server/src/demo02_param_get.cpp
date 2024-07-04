#include "ros/ros.h"

/*
    参数服务器操作之查询_C++实现:
    在 roscpp 中提供了两套 API 实现参数操作
    ros::NodeHandle
        param(键,默认值) 
            存在，返回对应结果，否则返回默认值

        getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamCached键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamNames(std::vector<std::string>)
            获取所有的键,并存储在参数 vector 中 

        hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

        searchParam(参数1，参数2)
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量

    ros::param ----- 与 NodeHandle 类似
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); 
    // 2.初始化ros节点
    ros::init(argc,argv,"get_param_c");//节点名称需要保持唯一
    // 3.创建节点句柄
    ros::NodeHandle nh;
   
    // 4.参数增 
    // 方案1 ros::NodeHandle
    // 1.param(键,默认值)
    double radius = nh.param("radius",0.5);
    ROS_INFO("radius=%.2f",radius);
    
    // 2.getParam
    double radius2=0.0;
    // bool result = nh.getParam("radius",radius2);
    
    // 3.getParamCached 与 getParam类似，只是性能上有提升
    bool result = nh.getParamCached("radius",radius2);
    if(result)
    {
        ROS_INFO("获取的半径是：%.2f",radius2);
    }
    else
    {
        ROS_INFO("被查询的变量不存在");
    }

    // 4.getParamNames(std::vector<std::string>)
    std::vector<std::string> names;
    nh.getParamNames(names);
    for (auto &&name : names)
    {
        ROS_INFO("遍历到的元素：%s",name.c_str());
    }

    // 5.hasParam
    bool flag1 = nh.hasParam("radius");
    bool flag2 = nh.hasParam("radiusxxx");
    ROS_INFO("radius 存在吗？%d",flag1);
    ROS_INFO("radiusxxx 存在吗？%d",flag2);


    // 6.searchParam(参数1，参数2)
    std::string key;
    nh.searchParam("radius",key);
    ROS_INFO("搜索结果：%s",key.c_str());

    // 方案2 ros::param
    double radius_param=ros::param::param("radius",100.5);
    ROS_INFO("radius_param:%.2f",radius_param);
    
    std::vector<std::string> names_param;
    ros::param::getParamNames(names_param);
    for (auto &&name : names_param)
    {
        ROS_INFO("键:%s",name.c_str());
    }
    return 0;
}
