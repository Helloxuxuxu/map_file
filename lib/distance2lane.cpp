#include "distance2lane.h"
// 定义一个很小的阈值，用于浮点数比较
const double EPSILON = 1e-5;
// 在命名空间中实现 Point2LineSegment 函数
namespace DistanceCalculation
{

    /**
     * @brief 计算点到线段的最短距离
     *
     * 该函数接收一个点的坐标和一条线段两个端点的坐标，计算该点到线段的最短距离。
     * 若线段长度趋近于 0，则将其视为一个点，计算点到该点的距离。
     * 否则，计算点在线段上的投影点，根据投影点位置计算最短距离。
     *
     * @param p_x 点的 x 坐标
     * @param p_y 点的 y 坐标
     * @param L_x1 线段第一个端点的 x 坐标
     * @param L_y1 线段第一个端点的 y 坐标
     * @param L_x2 线段第二个端点的 x 坐标
     * @param L_y2 线段第二个端点的 y 坐标
     * @return double 点到线段的最短距离
     */

    double Point2LineSegment(double p_x, double p_y, double L_x1, double L_y1, double L_x2, double L_y2)
    {
        double line_length_sq = (L_x2 - L_x1) * (L_x2 - L_x1) + (L_y2 - L_y1) * (L_y2 - L_y1);
        if (line_length_sq < EPSILON)
            return std::hypot(p_x - L_x1, p_y - L_y1);

        double t = ((p_x - L_x1) * (L_x2 - L_x1) + (p_y - L_y1) * (L_y2 - L_y1)) / line_length_sq; // 求导得到的直线距离最优值

        t = std::max(0.0, std::min(1.0, t)); // 限制在线段内[0-1]

        double proj_x = L_x1 + t * (L_x2 - L_x1);
        double proj_y = L_y1 + t * (L_y2 - L_y1);

        return std::hypot(p_x - proj_x, p_y - proj_y);
    }

    /**
     * 计算向量叉积 (B-A) × (C-A)
     */
    double ccw(const Point &A, const Point &B, const Point &C)
    {
        return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
    }

    /**
     * @brief 判断点P是否在线段AB上
     */
    bool isPointOnSegment(const Point &P, const Point &A, const Point &B)
    {
        return (std::min(A.x, B.x) <= P.x + 1e-9 && P.x <= std::max(A.x, B.x) + 1e-9) &&
               (std::min(A.y, B.y) <= P.y + 1e-9 && P.y <= std::max(A.y, B.y) + 1e-9);
    }

    // 判断线段AB和CD是否相交
    bool segmentsIntersect(const Point &A, const Point &B, const Point &C, const Point &D)
    {
        // 快速排斥试验
        if (std::max(A.x, B.x) < std::min(C.x, D.x) - 1e-9 ||
            std::max(C.x, D.x) < std::min(A.x, B.x) - 1e-9 ||
            std::max(A.y, B.y) < std::min(C.y, D.y) - 1e-9 ||
            std::max(C.y, D.y) < std::min(A.y, B.y) - 1e-9)
        {
            return false;
        }

        // 跨立试验
        double ccw1 = ccw(A, B, C);
        double ccw2 = ccw(A, B, D);
        double ccw3 = ccw(C, D, A);
        double ccw4 = ccw(C, D, B);

        // 判断叉积符号
        if ((ccw1 * ccw2 <= 1e-9) && (ccw3 * ccw4 <= 1e-9))
        {
            // 处理端点在另一条线段上的情况
            if ((std::abs(ccw1) < 1e-9 && isPointOnSegment(C, A, B)) ||
                (std::abs(ccw2) < 1e-9 && isPointOnSegment(D, A, B)) ||
                (std::abs(ccw3) < 1e-9 && isPointOnSegment(A, C, D)) ||
                (std::abs(ccw4) < 1e-9 && isPointOnSegment(B, C, D)))
            {
                return true;
            }
            return true;
        }
        return false;
    }

    void test()
    {
        if (true)
        { // 测试线段相交的基本情况
            Point A(0, 0), B(1, 1), C(1, 0), D(0, 1);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试线段不相交的情况
            Point A(0, 0), B(1, 1), C(2, 0), D(3, 1);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试共线但不相交的情况
            Point A(0, 0), B(1, 0), C(2, 0), D(3, 0);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试共线且部分重叠的情况
            Point A(0, 0), B(2, 0), C(1, 0), D(3, 0);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试端点相交共线的情况
            Point A(0, 0), B(1, 1), C(1, 1), D(2, 2);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试平行但不相交的情况
            Point A(0, 0), B(1, 1), C(0, 1), D(1, 2);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试垂直线段相交
            Point A(1, 0), B(1, 2), C(0, 1), D(2, 1);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试一个线段完全包含在另一个线段中的情况
            Point A(0, 0), B(3, 0), C(1, 0), D(2, 0);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        if (true)
        { // 测试线段共享一个端点但不共线的情况
            Point A(0, 0), B(1, 1), C(1, 1), D(1, 2);
            std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl; // 输出: True
        }
        return;
    }

    /**
     * @brief 计算两条线段之间的最短距离
     *
     * 该函数通过计算第一条线段的两个端点到第二条线段的最短距离，
     * 以及第二条线段的两个端点到第一条线段的最短距离，
     * 然后取这四个最短距离中的最小值作为两条线段之间的最短距离。
     *
     * @param L1_x1 第一条线段第一个端点的 x 坐标
     * @param L1_y1 第一条线段第一个端点的 y 坐标
     * @param L1_x2 第一条线段第二个端点的 x 坐标
     * @param L1_y2 第一条线段第二个端点的 y 坐标
     * @param L2_x1 第二条线段第一个端点的 x 坐标
     * @param L2_y1 第二条线段第一个端点的 y 坐标
     * @param L2_x2 第二条线段第二个端点的 x 坐标
     * @param L2_y2 第二条线段第二个端点的 y 坐标
     * @return double 两条线段之间的最短距离
     */
    double LineSegment2LineSegment(double L1_x1, double L1_y1, double L1_x2, double L1_y2,
                                   double L2_x1, double L2_y1, double L2_x2, double L2_y2)
    {

        Point A1(L1_x1, L1_y1), A2(L1_x2, L1_y2), B1(L2_x1, L2_y1), B2(L2_x2, L2_y2);
        // 先判断两条线段是否相交
        if (segmentsIntersect(A1, A2, B1, B2))
        {
            return 0; // 如果相交，直接返回 0
        }

        // 计算线段1的两个端点到线段2的最短距离
        double dist1 = Point2LineSegment(L1_x1, L1_y1, L2_x1, L2_y1, L2_x2, L2_y2);
        double dist2 = Point2LineSegment(L1_x2, L1_y2, L2_x1, L2_y1, L2_x2, L2_y2);

        // 计算线段2的两个端点到线段1的最短距离
        double dist3 = Point2LineSegment(L2_x1, L2_y1, L1_x1, L1_y1, L1_x2, L1_y2);
        double dist4 = Point2LineSegment(L2_x2, L2_y2, L1_x1, L1_y1, L1_x2, L1_y2);

        // 取四个距离中的最小值
        return std::min({dist1, dist2, dist3, dist4});
    }

    //  计算车两侧到车道所有线段的最小距离
    std::array<double, 5> calculateMinDistancesToLanes(
        const std::array<double, 2> &front_right_coords,
        const std::array<double, 2> &rear_right_coords,
        const std::array<double, 2> &rear_left_coords,
        const std::array<double, 2> &front_left_coords,
        const std::vector<double> &left_x,
        const std::vector<double> &left_y,
        const std::vector<double> &right_x,
        const std::vector<double> &right_y)
    {
        std::array<double, 5> min_distances = {
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            0, // 0：代表左侧为A车道，即 left_x，left_y  1：代表左侧为B车道
            0, // 左侧最短索引
            0  // 右侧最短索引
        };
        double L2A = std::numeric_limits<double>::max(); // 车辆左侧到A车道线的最短距离
        double L2B = std::numeric_limits<double>::max();
        double R2A = std::numeric_limits<double>::max();
        double R2B = std::numeric_limits<double>::max();
        double temp_distance = std::numeric_limits<double>::max();
        double L2A_index = 0;
        double L2B_index = 0;
        double R2A_index = 0;
        double R2B_index = 0;
        // 遍历左车道的所有线段
        for (size_t i = 0; i < left_x.size() - 1; ++i)
        {
            double L1_x1 = left_x[i];
            double L1_y1 = left_y[i];
            double L1_x2 = left_x[i + 1];
            double L1_y2 = left_y[i + 1];

            // 计算 车辆左侧 到 左（A）车道线 的距离
            temp_distance = LineSegment2LineSegment(
                rear_left_coords[0], rear_left_coords[1], front_left_coords[0], front_left_coords[1],
                L1_x1, L1_y1, L1_x2, L1_y2);
            if (temp_distance <= L2A)
            {
                L2A = temp_distance;
                L2A_index = i;
            }

            // 计算 车辆右侧 到 左（A）车道线 的距离
            temp_distance = LineSegment2LineSegment(
                rear_right_coords[0], rear_right_coords[1], front_right_coords[0], front_right_coords[1],
                L1_x1, L1_y1, L1_x2, L1_y2);
            if (temp_distance <= R2A)
            {
                R2A = temp_distance;
                R2A_index = i;
            }
        }

        for (size_t i = 0; i < right_x.size() - 1; ++i)
        {
            double L2_x1 = right_x[i];
            double L2_y1 = right_y[i];
            double L2_x2 = right_x[i + 1];
            double L2_y2 = right_y[i + 1];

            // 计算 车辆左侧 到 右（B）车道线 的距离
            temp_distance = LineSegment2LineSegment(
                rear_left_coords[0], rear_left_coords[1], front_left_coords[0], front_left_coords[1],
                L2_x1, L2_y1, L2_x2, L2_y2);
            if (temp_distance <= L2B)
            {
                L2B = temp_distance;
                L2B_index = i;
            }
            // 计算 车辆右侧 到 右（B）车道线 的距离
            temp_distance = LineSegment2LineSegment(
                rear_right_coords[0], rear_right_coords[1], front_right_coords[0], front_right_coords[1],
                L2_x1, L2_y1, L2_x2, L2_y2);
            if (temp_distance <= R2B)
            {
                R2B = temp_distance;
                R2B_index = i;
            }
        }

        // A车道线距离车辆 哪（左右）侧更近，则A到 哪（左右）侧 的距离为 哪（左右侧）侧 的最短距离
        // （例如A车道线，距离车辆两侧，相对车辆右侧更近，则A车道，在车辆右侧。则车辆右侧最短距离为 A车道线，到车辆右侧的距离）

        if (L2A <= R2A)
        {                                 // 车道A离车辆左侧更近，此时必定 L2B >= R2B，即车道B在车辆右侧
            min_distances[0] = L2A;       // 左侧最短
            min_distances[1] = R2B;       // 右侧最短
            min_distances[2] = 0;         // 左侧为A车道
            min_distances[3] = L2A_index; // 左侧最短索引
            min_distances[4] = R2B_index; // 右侧最短索引
        }
        else
        {                                 // 车道A离车辆右侧更近，此时必定 L2B < R2B，即车道B在车辆左侧
            min_distances[0] = L2B;       // 左侧最短
            min_distances[1] = R2A;       // 右侧最短
            min_distances[2] = 1;         // 左侧为B车道
            min_distances[3] = L2B_index; // 左侧最短索引
            min_distances[4] = R2A_index; // 右侧最短索引
        }
        return min_distances;
    }

    // 计算车角点到车道中线所有线段的最小距离
    std::array<double, 4> DistanceCorner2MidLanes(
        const std::array<double, 2> &front_right_coords,
        const std::array<double, 2> &rear_right_coords,
        const std::array<double, 2> &rear_left_coords,
        const std::array<double, 2> &front_left_coords,
        const std::vector<double> &center_x,
        const std::vector<double> &center_y)
    {
        std::array<double, 4> min_distances = {
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
        };

        for (size_t i = 0; i < center_x.size() - 1; ++i){
            double x1 = center_x[i];
            double y1 = center_y[i];
            double x2 = center_x[i + 1];
            double y2 = center_y[i + 1];
            min_distances[0] = std::min(min_distances[0], Point2LineSegment(front_right_coords[0], front_right_coords[1], x1, y1, x2, y2));
            min_distances[1] = std::min(min_distances[1], Point2LineSegment(rear_right_coords[0], rear_right_coords[1], x1, y1, x2, y2));
            min_distances[2] = std::min(min_distances[2], Point2LineSegment(rear_left_coords[0], rear_left_coords[1], x1, y1, x2, y2));
            min_distances[3] = std::min(min_distances[3], Point2LineSegment(front_left_coords[0], front_left_coords[1], x1, y1, x2, y2));
        }
        return min_distances;
    }
}