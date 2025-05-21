#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>

#include "draw.h"
#include "distance2lane.h"

#include "map_file/LaneDistanceService.h"

// 全局变量
namespace DC = DistanceCalculation;

bool parking_triggered = false;
// static bool g_viz_lanelets_centerline = true;
// lanelet::LaneletMapPtr map_ptr;
geometry_msgs::PoseStamped current_pose;
ros::Subscriber bin_map_sub;
ros::Subscriber pose_sub;
ros::Publisher distance_pub;
// 新增全局变量，用于存储角点坐标
double front_right_x, front_right_y;
double rear_right_x, rear_right_y;
double rear_left_x, rear_left_y;
double front_left_x, front_left_y;
bool draw_flag = false;


// 地图回调函数（停车触发后使用）
std::array<double,2> processParkingTask(const autoware_lanelet2_msgs::MapBin &msg, const geometry_msgs::PoseStamped &current_pose)
{

    // lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);
    lanelet::LaneletMapPtr viz_lanelet_map = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(msg, viz_lanelet_map);
    ROS_INFO("Map loaded");

    // get lanelets etc to visualize
    lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
    lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

    int lane_num = 0;
    // 定义三个二维向量存储点集
    std::vector<std::array<double, 3>> left_points;
    std::vector<std::array<double, 3>> right_points;
    std::vector<std::array<double, 3>> center_points;
    // 提取x和y坐标
    auto extractXYZ = [](const auto &points)
    {
        std::vector<std::array<double, 3>> XYZpoint;
        for (const auto &point : points)
        {
            XYZpoint.push_back({point.x(), point.y(), point.z()});
        }
        return XYZpoint;
    };
    for (auto li = all_lanelets.begin(); li != all_lanelets.end(); li++)
    {
        lanelet::ConstLanelet lll = *li;
        // right_ls.constData_.get().points_.size()
        // right_ls.constData_.get().points_[0].constData_.get().point.m_storage.m_data.array
        // right_ls.constData_.get().points_
        lanelet::ConstLineString3d left_ls = lll.leftBound();
        lanelet::ConstLineString3d right_ls = lll.rightBound();
        lanelet::ConstLineString3d center_ls = lll.centerline();
        auto new_left_points = extractXYZ(left_ls);
        left_points.insert(left_points.end(), new_left_points.begin(), new_left_points.end());
        auto new_right_points = extractXYZ(right_ls);
        right_points.insert(right_points.end(), new_right_points.begin(), new_right_points.end());
        auto new_center_points = extractXYZ(center_ls);
        center_points.insert(center_points.end(), new_center_points.begin(), new_center_points.end());
        lane_num += 1;
    }
    ROS_INFO("left_points size: %lu", left_points.size());
    ROS_INFO("right_points size: %lu", right_points.size());
    ROS_INFO("center_points size: %lu", center_points.size());
    ROS_INFO("lane_num: %d", lane_num);

    // 提取x和y坐标
    auto extractXY = [](const auto &points)
    {
        std::vector<double> x, y;
        for (const auto &point : points)
        {
            x.push_back(point[0]);
            y.push_back(point[1]);
        }
        return std::make_pair(x, y);
    };
    auto [left_x, left_y] = extractXY(left_points);
    auto [right_x, right_y] = extractXY(right_points);
    auto [center_x, center_y] = extractXY(center_points);

    // 创建tf2四元数对象
    tf2::Quaternion q;
    tf2::fromMsg(current_pose.pose.orientation, q);

    // 获取偏航角（弧度）
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
    double yaw_deg = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    ROS_INFO("Current yaw_direct --- yaw (deg): %f", yaw_deg * 180.0 / M_PI);
    ROS_INFO("Current pose --- x: %f, y: %f, yaw (rad): %f",
             current_pose.pose.position.x, current_pose.pose.position.y, yaw);

    auto calculateGlobalPose = [&current_pose, &yaw](double local_x, double local_y)
    {
        std::array<double, 2> global_coords;
        global_coords[0] = current_pose.pose.position.x + local_x * cos(yaw) - local_y * sin(yaw);
        global_coords[1] = current_pose.pose.position.y + local_x * sin(yaw) + local_y * cos(yaw);
        return global_coords;
    };
    std::array<double, 2> front_right_coords = calculateGlobalPose(front_right_x, front_right_y);
    std::array<double, 2> rear_right_coords = calculateGlobalPose(rear_right_x, rear_right_y);
    std::array<double, 2> rear_left_coords = calculateGlobalPose(rear_left_x, rear_left_y);
    std::array<double, 2> front_left_coords = calculateGlobalPose(front_left_x, front_left_y);

    std::array<double, 5> min_distance =
        DC::calculateMinDistancesToLanes(front_right_coords, rear_right_coords, rear_left_coords, front_left_coords, left_x, left_y, right_x, right_y);
    ROS_INFO("\033[34mmin_distance: %f, %f", min_distance[0], min_distance[1]);
    if (draw_flag)
    {
        Draw::plotAndSaveLanes(left_x, left_y, right_x, right_y, center_x, center_y,
                               front_right_coords, rear_right_coords, rear_left_coords, front_left_coords, min_distance[2], min_distance[3], min_distance[4], false);
    }

    // 创建并填充 Float64MultiArray 消息
    std_msgs::Float64MultiArray distance_msg;
    distance_msg.data.clear();
    distance_msg.data.push_back(current_pose.pose.position.x);
    distance_msg.data.push_back(current_pose.pose.position.y);
    distance_msg.data.push_back(yaw);
    distance_msg.data.push_back(min_distance[0]);
    distance_msg.data.push_back(min_distance[1]);
    distance_pub.publish(distance_msg);

    return std::array<double, 2> {min_distance[0], min_distance[1]};
}

// 停车服务处理函数
bool handleParkRequest(map_file::LaneDistanceService::Request &req,
                       map_file::LaneDistanceService::Response &res)
{
    if (parking_triggered)
    {
        res.success = false;
        res.message = "The parking process is in progress.";
        return true;
    }

    // 触发停车流程
    parking_triggered = true;
    ROS_INFO("Received the parking signal and started subscribing to map information");

    // 修改订阅者的回调函数（如果需要动态修改）
    static ros::NodeHandle nh_("~"); // 使用私有命名空间的NodeHandle
    double wait_time = 1.0;          // 等待时间（秒）
    autoware_lanelet2_msgs::MapBin::ConstPtr map_msg =
        ros::topic::waitForMessage<autoware_lanelet2_msgs::MapBin>(
            "/lanelet_map_bin",
            nh_,
            ros::Duration(wait_time));
    if (map_msg)
    {
        // 处理消息
        ROS_INFO("\033[34mReceived a map message, timestamp: %f", map_msg->header.stamp.toSec());
    }
    else
    {
        ROS_WARN("Map Message waiting timed out (%f seconds)", wait_time);
    }

    if (map_msg)
    {
        geometry_msgs::PoseStamped default_pose_msg;
        default_pose_msg.header.stamp = ros::Time::now();
        default_pose_msg.header.frame_id = "map";
        default_pose_msg.pose.position.x = req.x;
        default_pose_msg.pose.position.y = req.y;
        default_pose_msg.pose.position.z = 0.1;
        double yaw = req.yaw / 180.0 * M_PI; //rad

        // 创建 tf2 四元数对象
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw); // 绕 z 轴旋转，roll 和 pitch 为 0
        q.normalize();

        // 将 tf2 四元数转换为 geometry_msgs::Quaternion
        default_pose_msg.pose.orientation = tf2::toMsg(q);

        std::array<double,2> MinLaneDistance = processParkingTask(*map_msg, default_pose_msg);
        res.left_distance = MinLaneDistance[0];
        res.right_distance = MinLaneDistance[1];
        res.success = true;
        res.message = "\033[34mThe parking computation process has been completed.";
    }
    else
    {
        ROS_ERROR("Failed to get map or pose message within the specified time.");
        res.left_distance = -1;
        res.right_distance = -1;
        res.success = false;
        res.message = "Failed to get map or pose message within the specified time.";
    }
    parking_triggered = false;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    // 获取角点坐标参数
    if (true)
    {
        if (!nh_.getParam("front_right_x", front_right_x))
        {
            ROS_WARN("Failed to get front_right_x parameter, using default value 0.");
            front_right_x = 0;
        }
        if (!nh_.getParam("front_right_y", front_right_y))
        {
            ROS_WARN("Failed to get front_right_y parameter, using default value 0.");
            front_right_y = 0;
        }
        if (!nh_.getParam("rear_right_x", rear_right_x))
        {
            ROS_WARN("Failed to get rear_right_x parameter, using default value 0.");
            rear_right_x = 0;
        }
        if (!nh_.getParam("rear_right_y", rear_right_y))
        {
            ROS_WARN("Failed to get rear_right_y parameter, using default value 0.");
            rear_right_y = 0;
        }
        if (!nh_.getParam("rear_left_x", rear_left_x))
        {
            ROS_WARN("Failed to get rear_left_x parameter, using default value 0.");
            rear_left_x = 0;
        }
        if (!nh_.getParam("rear_left_y", rear_left_y))
        {
            ROS_WARN("Failed to get rear_left_y parameter, using default value 0.");
            rear_left_y = 0;
        }
        if (!nh_.getParam("front_left_x", front_left_x))
        {
            ROS_WARN("Failed to get front_left_x parameter, using default value 0.");
            front_left_x = 0;
        }
        if (!nh_.getParam("front_left_y", front_left_y))
        {
            ROS_WARN("Failed to get front_left_y parameter, using default value 0.");
            front_left_y = 0;
        }
        if (!nh_.getParam("draw_flag", draw_flag))
        {
            ROS_WARN("Failed to get draw_flag parameter, using default value false.");
            draw_flag = false;
        }

        // 打印获取到的角点坐标
        ROS_INFO("Front Right: (%f, %f)", front_right_x, front_right_y);
        ROS_INFO("Rear Right: (%f, %f)", rear_right_x, rear_right_y);
        ROS_INFO("Rear Left: (%f, %f)", rear_left_x, rear_left_y);
        ROS_INFO("Front Left: (%f, %f)", front_left_x, front_left_y);
    }
    // 创建距离车道发布者
    distance_pub = nh.advertise<std_msgs::Float64MultiArray>("distance2lane", 1);
    //  创建停车服务
    ros::ServiceServer service = nh.advertiseService("parking_service", handleParkRequest);
    if (!service)
    {
        ROS_ERROR("Failed to advertise the parking_service.");
        return -1;
    }
    ROS_INFO("The parking controller has been started and is waiting for parking service requests....");
    ros::spin();
    return 0;
}