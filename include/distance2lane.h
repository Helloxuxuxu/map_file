#ifndef DISTANCE2LANE_H
#define DISTANCE2LANE_H
#include <cmath>
#include <array>
#include <vector>
#include <iostream>
#include <algorithm>  // 必须包含此头文件
// 定义命名空间
namespace DistanceCalculation {
    struct Point {
        double x, y;
        Point(double x = 0, double y = 0) : x(x), y(y) {}
    };
    double ccw(const Point& A, const Point& B, const Point& C);
    bool isPointOnSegment(const Point& P, const Point& A, const Point& B);
    bool segmentsIntersect(const Point& A, const Point& B, const Point& C, const Point& D);
    double Point2LineSegment(double p_x, double p_y, double L_x1, double L_y1, double L_x2, double L_y2);
    double LineSegment2LineSegment(double L1_x1, double L1_y1, double L1_x2, double L1_y2, 
                                double L2_x1, double L2_y1, double L2_x2, double L2_y2);
    std::array<double, 5> calculateMinDistancesToLanes(
            const std::array<double, 2>& front_right_coords,
            const std::array<double, 2>& rear_right_coords,
            const std::array<double, 2>& rear_left_coords,
            const std::array<double, 2>& front_left_coords,
            const std::vector<double>& left_x,
            const std::vector<double>& left_y,
            const std::vector<double>& right_x,
            const std::vector<double>& right_y
        );
    std::array<double, 4> DistanceCorner2MidLanes(const std::array<double, 2>& front_right_coords,
        const std::array<double, 2>& rear_right_coords,
        const std::array<double, 2>& rear_left_coords,
        const std::array<double, 2>& front_left_coords,
        const std::vector<double>& center_x,
        const std::vector<double>& center_y);
    }
    
#endif // DISTANCE2LANE_H