1.  运行地图加载转换以及发布，在rviz中订阅
roslaunch map_file lanelet2_map_loader.launch file_name:=/home/nihao/workspace/map_ws/src/mapfile/lanelet2_maps_20250422.osm


2.  laneletsBoundaryAsMarkerArray 函数实现
visualization_msgs::MarkerArray visualization::laneletsBoundaryAsMarkerArray(const lanelet::ConstLanelets& lanelets,
                                                                             const std_msgs::ColorRGBA c,
                                                                             const bool viz_centerline)
{
  double lss = 0.2;  // line string size
  visualization_msgs::MarkerArray marker_array;
  for (auto li = lanelets.begin(); li != lanelets.end(); li++)
  {
    lanelet::ConstLanelet lll = *li;

    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();

    visualization_msgs::Marker left_line_strip, right_line_strip, center_line_strip;

    visualization::lineString2Marker(left_ls, &left_line_strip, "map", "left_lane_bound", c, lss);
    visualization::lineString2Marker(right_ls, &right_line_strip, "map", "right_lane_bound", c, lss);
    marker_array.markers.push_back(left_line_strip);
    marker_array.markers.push_back(right_line_strip);
    if (viz_centerline)
    {
      visualization::lineString2Marker(center_ls, &center_line_strip, "map", "center_lane_line", c, lss * 0.5);
      marker_array.markers.push_back(center_line_strip);
    }
  }
  return marker_array;
}

3.  运行地图加载转换,画出
roslaunch map_file_parking lanelet2_map_parking.launch file_name:=/home/nihao/workspace/map_ws/src/mapfile/lanelet2_maps_20250422.osm

4.  运行地图加载转换以及发布停车点,画出(client and service)
roslaunch map_file_parking parking_process.launch file_name:=/home/nihao/workspace/map_ws/src/mapfile/lanelet2_maps_20250422.osm

5.发布的话题
rostopic echo /distance2lane

6.编译
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=14  #Debug
catkin_make -DCMAKE_CXX_STANDARD=14  
catkin build map_file
