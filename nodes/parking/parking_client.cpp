#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "map_file/LaneDistanceService.h"

double yaw_set = 0.0;
double x_set = 0.0;
double y_set = 0.0;
int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "parking_client");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    if (!nh_.getParam("yaw_set", yaw_set))
    {
        ROS_WARN("Failed to get yaw_set parameter, using default value 0.");
        yaw_set = 0;
    }
    if (!nh_.getParam("x_set", x_set))
    {
        ROS_WARN("Failed to get x_set parameter, using default value 0.");
        x_set = 0;
    }
    if (!nh_.getParam("y_set", y_set))
    {
        ROS_WARN("Failed to get y_set parameter, using default value 0.");
        y_set = 0;
    }

    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<map_file::LaneDistanceService>("parking_service");

    // 等待服务启动
    if (!client.waitForExistence(ros::Duration(5.0)))
    {
        ROS_ERROR("The parking_service is not available within 5 seconds.");
        return -1;
    }

    // 创建服务请求消息
    map_file::LaneDistanceService srv;
    srv.request.x = x_set;
    srv.request.y = y_set;
    srv.request.yaw = yaw_set; // deg
    // 设置循环频率为 1Hz，即每秒执行一次
    ros::Rate rate(1);
    // 进入循环，每秒发送一次服务请求
    while (ros::ok())
    {
        // 调用服务
        if (client.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Parking service call succeeded: %s", srv.response.message.c_str());
                ROS_INFO("\033[34mClient received--- Left distance: %f --- Right distance: %f", srv.response.left_distance, srv.response.right_distance);
            }
            else
            {
                ROS_WARN("Parking service call failed: %s", srv.response.message.c_str());
            }
        }
        else
        {
            ROS_ERROR("Failed to call the parking_service.");
        }

        // 按照设定频率休眠
        rate.sleep();
    }

    return 0;
}