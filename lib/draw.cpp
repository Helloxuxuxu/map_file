#include "draw.h"
#include <iostream>
#include <cmath>
namespace plt = matplotlibcpp;

// 绘制并保存图像的函数
namespace Draw
{
    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                          const std::vector<double> &right_x, const std::vector<double> &right_y,
                          const std::vector<double> &center_x, const std::vector<double> &center_y,
                          bool save = false)
    {
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);

        plt::named_plot("mid lane", center_x, center_y, "r--");
        plt::named_plot("left lane", left_x, left_y, "g");
        plt::named_plot("right lane", right_x, right_y, "b");
        // Set x-axis to interval [0,1000000]
        // plt::xlim(0, 1000*1000);

        // Add graph title
        plt::title("map lanes");

        // Enable legend.
        plt::legend();
        plt::show();
        // plt::figure();
        // plt::pause(1);

        if (save)
        {
            const std::string filename = "/home/nihao/Pictures/map_lanes.png";
            std::cout << "Saving result to " << filename << std::endl;
            plt::save(filename);
        }
    }

    // 绘制并保存图像的函数
    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                          const std::vector<double> &right_x, const std::vector<double> &right_y,
                          bool save = false)
    {
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);
        plt::named_plot("left lane", left_x, left_y, "g");
        plt::named_plot("right lane", right_x, right_y, "b");
        // Set x-axis to interval [0,1000000]
        // plt::xlim(0, 1000*1000);

        // Add graph title
        plt::title("map lanes");

        // Enable legend.
        plt::legend();
        // plt::show();
        plt::figure();
        plt::pause(1);

        if (save)
        {
            const std::string filename = "/home/nihao/Pictures/map_lanes.png";
            std::cout << "Saving result to " << filename << std::endl;
            plt::save(filename);
        }
    }

    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                          const std::vector<double> &right_x, const std::vector<double> &right_y,
                          const std::vector<double> &center_x, const std::vector<double> &center_y,
                          std::array<double, 2>& front_right,
                          std::array<double, 2>& rear_right,
                          std::array<double, 2>& rear_left,
                          std::array<double, 2>& front_left,
                          bool save = false)
    {
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);

        plt::named_plot("mid lane", center_x, center_y, "r--");
        plt::named_plot("left lane", left_x, left_y, "g");
        plt::named_plot("right lane", right_x, right_y, "b");
        // 定义车辆角点的 x 和 y 坐标向量，按顺序连接四个角点
        std::vector<double> car_x = {front_right[0], rear_right[0], rear_left[0], front_left[0], front_right[0]};
        std::vector<double> car_y = {front_right[1], rear_right[1], rear_left[1], front_left[1], front_right[1]};
        // 绘制车辆轮廓
        plt::named_plot("car", car_x, car_y, "k-");
        // 绘制车辆角点
        // plt::plot({front_right[0]}, {front_right[1]}, "g^");
        // plt::plot({rear_right[0]}, {rear_right[1]}, "ro");
        // plt::plot({rear_left[0]}, {rear_left[1]}, "ro");
        // plt::plot({front_left[0]}, {front_left[1]}, "ro");
        // 绘制前右角点，用红色圆形，添加名称
        plt::named_plot("Front Right", std::vector<double> ({front_right[0]}), std::vector<double> ({front_right[1]}), "ro");
        // 绘制后右角点，用蓝色方形，添加名称
        plt::named_plot("Rear Right", std::vector<double> ({rear_right[0]}), std::vector<double> ({rear_right[1]}), "bs");
        // 绘制后左角点，用绿色三角形，添加名称
        plt::named_plot("Rear Left", std::vector<double> ({rear_left[0]}), std::vector<double> ({rear_left[1]}), "g^");
        // 绘制前左角点，用黄色菱形，添加名称
        plt::named_plot("Front Left", std::vector<double> ({front_left[0]}), std::vector<double> ({front_left[1]}), "yd");
        // Add graph title
        plt::title("map lanes and vehicle");
        // 设置 x 轴和 y 轴比例为 1:1
        plt::axis("equal");
        // Enable legend.
        plt::legend();
        if (save)
        {
            const std::string filename = "/home/nihao/Pictures/map_car_lanes.png";
            std::cout << "Saving result to " << filename << std::endl;
            plt::save(filename);
        }
        plt::show();
    }


    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                          const std::vector<double> &right_x, const std::vector<double> &right_y,
                          const std::vector<double> &center_x, const std::vector<double> &center_y,
                          std::array<double, 2>& front_right,
                          std::array<double, 2>& rear_right,
                          std::array<double, 2>& rear_left,
                          std::array<double, 2>& front_left,
                          double lane_situation,  //0;         // 左侧为A车道
                          double left_min_index,
                          double right_min_index,
                          bool save = false)
    {
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);

        plt::named_plot("mid lane", center_x, center_y, "r--");
        plt::named_plot("left lane", left_x, left_y, "g");
        plt::named_plot("right lane", right_x, right_y, "b");
        // 定义车辆角点的 x 和 y 坐标向量，按顺序连接四个角点
        std::vector<double> car_x = {front_right[0], rear_right[0], rear_left[0], front_left[0], front_right[0]};
        std::vector<double> car_y = {front_right[1], rear_right[1], rear_left[1], front_left[1], front_right[1]};
        // 绘制车辆轮廓
        plt::named_plot("car", car_x, car_y, "k-");
        // 绘制车辆角点
        // plt::plot({front_right[0]}, {front_right[1]}, "g^");
        // plt::plot({rear_right[0]}, {rear_right[1]}, "ro");
        // plt::plot({rear_left[0]}, {rear_left[1]}, "ro");
        // plt::plot({front_left[0]}, {front_left[1]}, "ro");
        // 绘制前右角点，用红色圆形，添加名称
        plt::named_plot("Front Right", std::vector<double> ({front_right[0]}), std::vector<double> ({front_right[1]}), "ro");
        // 绘制后右角点，用蓝色方形，添加名称
        plt::named_plot("Rear Right", std::vector<double> ({rear_right[0]}), std::vector<double> ({rear_right[1]}), "bs");
        // 绘制后左角点，用绿色三角形，添加名称
        plt::named_plot("Rear Left", std::vector<double> ({rear_left[0]}), std::vector<double> ({rear_left[1]}), "g^");
        // 绘制前左角点，用黄色菱形，添加名称
        plt::named_plot("Front Left", std::vector<double> ({front_left[0]}), std::vector<double> ({front_left[1]}), "yd");
        // Add graph title
        plt::title("map lanes and vehicle");
        // 设置 x 轴和 y 轴比例为 1:1
        // plt::axis("equal");
        // Enable legend.

        double EPSILON = 0.0001;
        int left_min_index_int = static_cast<int>(std::round(left_min_index));
        int right_min_index_int = static_cast<int>(std::round(right_min_index));
        //区分车道情况，哪侧是左侧
        if(lane_situation - 0.0 < EPSILON && lane_situation - 0.0 > -EPSILON){
            std::vector<double> left_min_x = {left_x[left_min_index_int], left_x[left_min_index_int+1]};
            std::vector<double> left_min_y = {left_y[left_min_index_int], left_y[left_min_index_int+1]};
            plt::named_plot("left min lane", left_min_x, left_min_y, "y--");
            std::vector<double> right_min_x = {right_x[right_min_index_int], right_x[right_min_index_int+1]};
            std::vector<double> right_min_y = {right_y[right_min_index_int], right_y[right_min_index_int+1]};
            plt::named_plot("right min lane", right_min_x, right_min_y, "m--");
        } else if (lane_situation - 1.0 < EPSILON && lane_situation - 1.0 > -EPSILON){
            std::vector<double> left_min_x = {right_x[left_min_index_int], right_x[left_min_index_int+1]};
            std::vector<double> left_min_y = {right_y[left_min_index_int], right_y[left_min_index_int+1]};
            plt::named_plot("left min lane", left_min_x, left_min_y, "y--");
            std::vector<double> right_min_x = {left_x[right_min_index_int], left_x[right_min_index_int+1]};
            std::vector<double> right_min_y = {left_y[right_min_index_int], left_y[right_min_index_int+1]};
            plt::named_plot("right min lane", right_min_x, right_min_y, "m--");

        }   else{
            std::cout << "lane situation error" << std::endl;
        }

        plt::legend();
        if (save)
        {
            const std::string filename = "/home/nihao/Pictures/map_car_lanes_min_lane.png";
            std::cout << "Saving result to " << filename << std::endl;
            plt::save(filename);
        }
        plt::show();
    }
}
