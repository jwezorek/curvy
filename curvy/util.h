#pragma once

#include <Eigen/Dense>
#include <tuple>

namespace curvy {

    using matrix = Eigen::Matrix<double, 3, 3>;
    using point = std::tuple<double, double>;

    double pi();
    double normalize(const double value, const double start, const double end);
    double normalize_angle(const double theta);
    double euclidean_distance(double x1, double y1, double x2, double y2);
    double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2);
    point operator-(const point& a);
    point apply_matrix(const matrix& mat, const point& pt);
    matrix rotation_matrix(double cos_theta, double sin_theta);
    matrix rotation_matrix(double theta);
    matrix translation_matrix(double x, double y);
    matrix scale_matrix(double x_scale, double y_scale);
    matrix scale_matrix(double scale);
    matrix translation_matrix(const point& pt);
    matrix identity_matrix();
    std::tuple<int, int> to_scr_coords(double x, double y, double logical_sz, int pixel_sz);
    std::tuple<int, int> to_scr_coords(const point& p, double logical_sz, int pixel_sz);
    std::tuple<double, double> from_scr_coords(int x, int y, double logical_sz, int pixel_sz);
    std::tuple<double, double> from_scr_coords(const std::tuple<int, int>& pt, double logical_sz, int pixel_sz);
    std::tuple<int, int, int, int> to_scr_coords(double x1, double y1, double x2, double y2, double logical_sz, int pixel_sz);
    std::tuple<int, int, int, int> to_scr_coords(const std::tuple<double, double, double, double>& rect, double logical_sz, int pixel_sz);
    double get_angle_to_pt(const point& from_pt, const point& to_pt);
    bool is_to_the_right_of(double direction, const point& from_pt, const point& to_pt);
    double direction_on_circle(double theta, bool orientation);

}