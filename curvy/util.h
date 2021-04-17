#pragma once

#include <Eigen/Dense>
#include <tuple>
#include <string>

namespace curvy {

    using matrix = Eigen::Matrix<double, 3, 3>;
    using point = std::tuple<double, double>;

    double eps();
    double pi();
    double pi_over_two();
    double three_pi_over_two();
    double two_pi();
    double normalize(const double value, const double start, const double end);
    double normalize_angle(const double theta);
    point normalize_pt(const point& pt);
    double euclidean_distance(double x1, double y1, double x2, double y2);
    double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2);
    point operator-(const point& a);
    point operator+(const point& a, const point& b);
    point operator-(const point& a, const point& b);
    point operator*(double scale, const point& a);
    point operator*(const point& a, double scale);
    std::string to_string(const point& pt);
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
    double angle_to_pt(const point& from_pt, const point& to_pt);
    double angle_to_point_relative_to_direction(const point& from_pt, double direction_at_from, const point& to_pt);
    bool is_to_the_right_of(double direction, const point& from_pt, const point& to_pt);
    double direction_on_circle(double theta, bool orientation);
    double atan_of_pt(const point& pt);
    double hypot_of_point(const point& pt);
    void output_debug_message(const std::string& msg);
    point pt_on_unit_circle(double theta);
    bool pt_in_triangle(const point& pt, const point& v1, const point& v2, const point& v3);
}