# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.14)  # 提升 CMake 最低版本要求，支持更多特性
project(map_file)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# add_compile_options(-g -O0 -std=c++17) # -O0 禁用优化，-g 添加调试符号



# 查找依赖项
find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  lanelet2_extension
  pcl_ros
  roscpp
  std_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  vector_map
  visualization_msgs
  nav_msgs
  message_generation 
)
# 服务消息
add_service_files(
  FILES
  LaneDistanceService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(PCL REQUIRED COMPONENTS io)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(CURL REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# 输出 Python 和 NumPy 信息
message(STATUS "Python3 Interpreter: ${Python3_EXECUTABLE}")
message(STATUS "Python3 Include Directory: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 Library: ${Python3_LIBRARIES}")
message(STATUS "NumPy Include Directory: ${Python3_NumPy_INCLUDE_DIRS}")
message(STATUS "NumPy Library: ${Python3_NumPy_LIBRARIES}")
if(Python3_NumPy_FOUND)
  message(STATUS "NumPy Include Directory: ${Python3_NumPy_INCLUDE_DIRS}")
  message(STATUS "NumPy Library: ${Python3_NumPy_LIBRARIES}")
else()
  message(FATAL_ERROR  "NumPy not found!")
endif()
# 处理 PCL 库问题
# See: https://github.com/ros-perception/perception_pcl/blob/lunar-devel/pcl_ros/CMakeLists.txt#L10-L22
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # FIXME: this causes duplicates and not found error in ubuntu:zesty
  list(REMOVE_ITEM PCL_LIBRARIES "/usr/lib/libmpi.so")

  # For debian: https://github.com/ros-perception/perception_pcl/issues/139
  list(REMOVE_ITEM PCL_IO_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt"
  )
endif()



# set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")


# Catkin 包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES get_file GraphicsLibrary DrawLibrary
  CATKIN_DEPENDS
    nav_msgs
    autoware_msgs
    geometry_msgs
    std_msgs
    tf2_geometry_msgs
    visualization_msgs
    message_runtime
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
#创建库
##
add_library(get_file lib/map_file/get_file.cpp)
target_link_libraries(get_file ${CURL_LIBRARIES})
##
add_library(GraphicsLibrary lib/distance2lane.cpp)  #图形计算
##
add_library(DrawLibrary lib/draw.cpp)  #绘图
target_compile_features(DrawLibrary PRIVATE cxx_std_17)
target_include_directories(DrawLibrary PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)
if(TARGET Python3::Numpy)
  target_link_libraries(DrawLibrary PRIVATE
    Python3::Python
    Python3::Module
    Python3::Numpy
  )
else()
  target_link_libraries(DrawLibrary PRIVATE
    Python3::Python
    Python3::Module
  )
  message(STATUS "Python3::Numpy target not found!")
endif()

#############
add_executable(points_map_loader nodes/points_map_loader/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} get_file ${CURL_LIBRARIES} ${PCL_IO_LIBRARIES})
add_dependencies(points_map_loader   ${catkin_EXPORTED_TARGETS})
#############
add_executable(vector_map_loader nodes/vector_map_loader/vector_map_loader.cpp)
target_link_libraries(vector_map_loader ${catkin_LIBRARIES} ${vector_map_LIBRARIES} get_file ${CURL_LIBRARIES})
add_dependencies(vector_map_loader ${catkin_EXPORTED_TARGETS})
#############
add_executable(lanelet2_map_loader nodes/lanelet2_map_loader/lanelet2_map_loader.cpp)
target_link_libraries(lanelet2_map_loader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lanelet2_map_loader ${catkin_EXPORTED_TARGETS})
#############
add_executable(lanelet2_map_visualization nodes/lanelet2_map_loader/lanelet2_map_visualization.cpp)
target_link_libraries(lanelet2_map_visualization ${catkin_LIBRARIES})
add_dependencies(lanelet2_map_visualization ${catkin_EXPORTED_TARGETS})
#############
add_executable(points_map_filter nodes/points_map_filter/points_map_filter_node.cpp nodes/points_map_filter/points_map_filter.cpp)
target_link_libraries(points_map_filter ${catkin_LIBRARIES})
add_dependencies(points_map_filter ${catkin_EXPORTED_TARGETS})

#############
add_executable(lanelet2_map_parking nodes/lanelet2_map_loader/lanelet2_map_parking.cpp)
target_include_directories(lanelet2_map_parking PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)
if(TARGET Python3::Numpy)
  target_link_libraries(lanelet2_map_parking PRIVATE
    ${catkin_LIBRARIES}
    Python3::Python
    Python3::Module
    Python3::Numpy
  )
else()
  target_link_libraries(lanelet2_map_parking PRIVATE
    ${catkin_LIBRARIES}
    Python3::Python
    Python3::Module
  )
  message(STATUS "Python3::Numpy target not found!")
endif()

set_target_properties(lanelet2_map_parking PROPERTIES # 添加链接器选项
    LINK_FLAGS "-export-dynamic"
)
add_dependencies(lanelet2_map_parking ${catkin_EXPORTED_TARGETS})

#############
add_executable(parking_service nodes/parking/parking_service.cpp)
target_include_directories(parking_service PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)
target_link_libraries(parking_service PRIVATE
    ${catkin_LIBRARIES}
    DrawLibrary
    GraphicsLibrary
)
set_target_properties(parking_service PROPERTIES # 添加链接器选项
    LINK_FLAGS "-export-dynamic"
)
add_dependencies(parking_service ${catkin_EXPORTED_TARGETS})

#############
add_executable(parking_client nodes/parking/parking_client.cpp)

target_link_libraries(parking_client PRIVATE ${catkin_LIBRARIES})

add_dependencies(parking_client ${catkin_EXPORTED_TARGETS})

#############
add_executable(car_tf_publisher_test nodes/parking/car_tf_publisher_test.cpp)
target_link_libraries(car_tf_publisher_test ${catkin_LIBRARIES})
add_dependencies(car_tf_publisher_test ${catkin_EXPORTED_TARGETS})

#############
add_executable(corner_distance_publisher nodes/parking/corner_distance_publisher.cpp)
target_include_directories(corner_distance_publisher PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)
target_link_libraries(corner_distance_publisher PRIVATE
    ${catkin_LIBRARIES}
    DrawLibrary
    GraphicsLibrary
)
set_target_properties(corner_distance_publisher PROPERTIES # 添加链接器选项
    LINK_FLAGS "-export-dynamic"
)
add_dependencies(corner_distance_publisher ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(
  TARGETS
    DrawLibrary
    GraphicsLibrary
    get_file
    points_map_loader
    vector_map_loader
    lanelet2_map_loader
    lanelet2_map_visualization
    lanelet2_map_parking
    points_map_filter
    parking_service
    parking_client
    car_tf_publisher_test
    corner_distance_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
