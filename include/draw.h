#ifndef DRAW_H
#define DRAW_H
#include <matplotlibcpp.h>
#include <cmath>

namespace plt = matplotlibcpp;
namespace Draw {
    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                        const std::vector<double> &right_x, const std::vector<double> &right_y,
                        const std::vector<double> &center_x, const std::vector<double> &center_y,
                        bool save);

    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                        const std::vector<double> &right_x, const std::vector<double> &right_y,
                        bool save);

    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                          const std::vector<double> &right_x, const std::vector<double> &right_y,
                          const std::vector<double> &center_x, const std::vector<double> &center_y,
                          std::array<double, 2>& front_right,
                          std::array<double, 2>& rear_right,
                          std::array<double, 2>& rear_left,
                          std::array<double, 2>& front_left,
                          bool save);
    void plotAndSaveLanes(const std::vector<double> &left_x, const std::vector<double> &left_y,
                          const std::vector<double> &right_x, const std::vector<double> &right_y,
                          const std::vector<double> &center_x, const std::vector<double> &center_y,
                          std::array<double, 2>& front_right,
                          std::array<double, 2>& rear_right,
                          std::array<double, 2>& rear_left,
                          std::array<double, 2>& front_left,
                          double lane_situation,
                          double left_min_index,
                          double right_min_index,
                          bool save);
}
#endif // DRAW_H