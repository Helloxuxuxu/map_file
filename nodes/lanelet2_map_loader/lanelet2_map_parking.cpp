/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <vector>

#include <matplotlibcpp.h>
#include <cmath>
namespace plt = matplotlibcpp;
static bool g_viz_lanelets_centerline = true;
static ros::Publisher g_map_pub;

void laneletsBoundary2line(autoware_lanelet2_msgs::MapBin msg)
{
  
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

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
  for (auto li = all_lanelets.begin(); li != all_lanelets.end(); li++)
  {
    lanelet::ConstLanelet lll = *li;
    // right_ls.constData_.get().points_.size()
    // right_ls.constData_.get().points_[0].constData_.get().point.m_storage.m_data.array
    // right_ls.constData_.get().points_
    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();

    // 提取左边界点集
    for (const auto& point : left_ls) {
      left_points.push_back({point.x(), point.y(), point.z()});
    }

    // 提取右边界点集
    for (const auto& point : right_ls) {
      right_points.push_back({point.x(), point.y(), point.z()});
    }

    // 提取中心线点集
    for (const auto& point : center_ls) {
      center_points.push_back({point.x(), point.y(), point.z()});
    }
    lane_num+=1;
  }
  ROS_INFO("left_points size: %lu", left_points.size());
  ROS_INFO("right_points size: %lu", right_points.size());
  ROS_INFO("center_points size: %lu", center_points.size());
  ROS_INFO("lane_num: %d", lane_num);

  std::vector<double> left_x,left_y;
  std::vector<double> right_x,right_y;
  std::vector<double> center_x,center_y;
  for (const auto& point : left_points){
    left_x.push_back(point[0]);
    left_y.push_back(point[1]);
  }
  for (const auto& point : right_points){
    right_x.push_back(point[0]);
    right_y.push_back(point[1]);

  }
  for (const auto& point : center_points){
    center_x.push_back(point[0]);
    center_y.push_back(point[1]);
  }


  
  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);

  plt::named_plot("mid lane", center_x, center_y,"r--");
  plt::named_plot("left lane", left_x, left_y,"g");
  plt::named_plot("right lane", right_x, right_y,"b");
  // Set x-axis to interval [0,1000000]
  // plt::xlim(0, 1000*1000);

  // Add graph title
  plt::title("map lanes");

  // Enable legend.
  plt::legend();
  plt::show();
  // save figure
  // const char* filename = "/home/nihao/workspace/map_ws/src/mapfile/map_lanes.png";
  // std::cout << "Saving result to " << filename << std::endl;
  // plt::save(filename);
  return ;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lanelet_map_visualizer");
  ros::NodeHandle rosnode;
  ros::Subscriber bin_map_sub;
  bin_map_sub = rosnode.subscribe("/lanelet_map_bin", 1, laneletsBoundary2line);
  ros::spin();
  return 0;
}
