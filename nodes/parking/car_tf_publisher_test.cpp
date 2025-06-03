#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "car_tf_publisher_test");
    ROS_INFO("car_tf_publisher_test node started.");
    ros::NodeHandle nh("~");

    double x_set, y_set, yaw_set;
    nh.param<double>("x_set", x_set, 47.4);
    nh.param<double>("y_set", y_set, 8.9);
    nh.param<double>("yaw_set", yaw_set, 190);

    // 将 yaw 角度转换为弧度
    double yaw_rad = yaw_set * 3.1415926 / 180.0;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    ros::Rate rate(10); // 10Hz 发布频率
    while (ros::ok()) {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = x_set;
        transformStamped.transform.translation.y = y_set;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
        rate.sleep();
    }

    return 0;
}