#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "draw.h"
#include "distance2lane.h"

// 定义全局变量
namespace DC = DistanceCalculation;
ros::Publisher corner_distance_pub;
ros::Subscriber map_sub;

// 角点坐标
double front_right_x, front_right_y;
double rear_right_x, rear_right_y;
double rear_left_x, rear_left_y;
double front_left_x, front_left_y;
double width, length;
double rotation_margin;
bool draw_flag = false;
// 全局智能指针
std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr;

// 地图回调函数
void Callback(const autoware_lanelet2_msgs::MapBin::ConstPtr& map_msg) {
    /* 获取车辆 base_link 相对于世界坐标系的位姿 */
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PoseStamped current_pose;
    if (!tf_buffer_ptr || !tf_listener_ptr) {
        ROS_FATAL("TF components not initialized!");
        return;
    }
    try {
        // 获取车辆 base_link 相对于世界坐标系的位姿
        transform = tf_buffer_ptr->lookupTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
        current_pose.header = transform.header;
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.position.z = transform.transform.translation.z;
        current_pose.pose.orientation = transform.transform.rotation;
        // ... 已有代码 ...
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed to get transform from world to base_link: %s", ex.what());
        return;
    }
    // 创建tf2四元数对象
    tf2::Quaternion q;
    tf2::fromMsg(current_pose.pose.orientation, q);
    // 获取偏航角（弧度）
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Current pose --- x: %f, y: %f, yaw (rad): %f",
             current_pose.pose.position.x, current_pose.pose.position.y, yaw);
    auto calculateGlobalPose =  [&current_pose, &yaw](double local_x, double local_y)
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
    
    /* 将接收到的地图消息转换为 LaneletMap,并最终转换为线段点集 */
    lanelet::LaneletMapPtr lanelet_map = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map);
    ROS_INFO("Map loaded in corner distance calculater...");
    lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
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

    /*开始计算距离,确定是否正向居中*/
    std::array<double, 4> min_corner_distance = //front_right rear_right rear_left front_left
        DC::DistanceCorner2MidLanes(front_right_coords, rear_right_coords, rear_left_coords, front_left_coords, center_x, center_y);

    ROS_INFO("front_right: %f, rear_right: %f, rear_left: %f, front_left: %f",
        min_corner_distance[0],min_corner_distance[1],min_corner_distance[2],min_corner_distance[3]);
    double yaw2line_flag = 1.0; // 1.0代表正
    if(min_corner_distance[0]+min_corner_distance[3] >= width or min_corner_distance[1]+min_corner_distance[2] >= width){
        ROS_INFO("Vehicle is not centered ...");
        yaw2line_flag = 0.0;
    }
    auto clamp = [](double x, double lower, double upper) {
        return x < lower ? lower : (x > upper ? upper : x);
    };
    double rotation_deg_0 = acos( clamp((min_corner_distance[0]+min_corner_distance[3])/width, -1.0, 1.0)) * 180 / M_PI;
    double rotation_deg_1 = acos( clamp((min_corner_distance[1]+min_corner_distance[2])/width, -1.0, 1.0)) * 180 / M_PI;
    double rotation_deg = std::max(rotation_deg_0, rotation_deg_1);
    if(rotation_deg >= rotation_margin){
        ROS_INFO("rotation_deg: %f, rotation_margin: %f", rotation_deg, rotation_margin);
        ROS_INFO("Vehicle is not parallel to the middle lane...");
        yaw2line_flag = 0.0;
    }
    /*可视化*/
    if (draw_flag)
    {
        Draw::plotAndSaveLanes(left_x, left_y, right_x, right_y, center_x, center_y,
                               front_right_coords, rear_right_coords, rear_left_coords, front_left_coords, false);
    }
    /*发布角点距离*/
    std_msgs::Float64MultiArray corner_distance_msg;
    corner_distance_msg.data.clear();
    corner_distance_msg.data.push_back(yaw2line_flag);
    for(size_t i = 0; i < min_corner_distance.size(); i++){
        corner_distance_msg.data.push_back(min_corner_distance[i]);
    }
    corner_distance_pub.publish(corner_distance_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "corner_distance_publisher");
    ROS_INFO("corner_distance_publisher node started.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    // 获取角点坐标参数
    if (true){
    if (!nh_.getParam("front_right_x", front_right_x)) {
        ROS_WARN("Failed to get front_right_x parameter, using default value 0.");
        front_right_x = 0;
    }
    if (!nh_.getParam("front_right_y", front_right_y)) {
        ROS_WARN("Failed to get front_right_y parameter, using default value 0.");
        front_right_y = 0;
    }
    if (!nh_.getParam("rear_right_x", rear_right_x)) {
        ROS_WARN("Failed to get rear_right_x parameter, using default value 0.");
        rear_right_x = 0;
    }
    if (!nh_.getParam("rear_right_y", rear_right_y)) {
        ROS_WARN("Failed to get rear_right_y parameter, using default value 0.");
        rear_right_y = 0;
    }
    if (!nh_.getParam("rear_left_x", rear_left_x)) {
        ROS_WARN("Failed to get rear_left_x parameter, using default value 0.");
        rear_left_x = 0;
    }
    if (!nh_.getParam("rear_left_y", rear_left_y)) {
        ROS_WARN("Failed to get rear_left_y parameter, using default value 0.");
        rear_left_y = 0;
    }
    if (!nh_.getParam("front_left_x", front_left_x)) {
        ROS_WARN("Failed to get front_left_x parameter, using default value 0.");
        front_left_x = 0;
    }
    if (!nh_.getParam("front_left_y", front_left_y)) {
        ROS_WARN("Failed to get front_left_y parameter, using default value 0.");
        front_left_y = 0;
    }
    if (!nh_.getParam("draw_flag", draw_flag)) {
        ROS_WARN("Failed to get draw_flag parameter, using default value false.");
        draw_flag = false;
    }
    if(!nh_.getParam("rotation_margin", rotation_margin)){
        ROS_WARN("Failed to get rotation_margin parameter, using default value 15.");
        rotation_margin = 15;
    }
    // 打印获取到的角点坐标
    ROS_INFO("Front Right: (%f, %f)", front_right_x, front_right_y);
    ROS_INFO("Rear Right: (%f, %f)", rear_right_x, rear_right_y);
    ROS_INFO("Rear Left: (%f, %f)", rear_left_x, rear_left_y);
    ROS_INFO("Front Left: (%f, %f)", front_left_x, front_left_y);
    ROS_INFO("Draw flag: %d", draw_flag);
    ROS_INFO("Rotation margin: %f", rotation_margin);
    width = sqrt(pow(front_right_x - front_left_x, 2) + pow(front_right_y - front_left_y, 2));
    length = sqrt(pow(front_right_x - rear_right_x, 2) + pow(front_right_y - rear_right_y, 2));
    ROS_INFO("Vehicle width: %f, length: %f", width, length);
    }
    
    // 创建 TF 监听器和缓冲区
    tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>(ros::Duration(0.5));
    tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);
    // 创建角点距离发布者
    corner_distance_pub = nh.advertise<std_msgs::Float64MultiArray>("/corner_distance", 1);

    // 创建地图订阅者
    map_sub = nh.subscribe<autoware_lanelet2_msgs::MapBin>("/lanelet_map_bin", 1, Callback);
    ros::spin();
    return 0;
}