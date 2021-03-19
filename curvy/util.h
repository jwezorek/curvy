#pragma once

#include <Eigen/Dense>
using matrix = Eigen::Matrix<double, 3, 3>;
using point = std::tuple<double, double>;

#include <tuple>
#include "circle.h"

point operator-(const point& a);
point apply_matrix(const matrix& mat, const point& pt);
void apply_matrix(const matrix& mat, const point& pts, int n);
curvy::circle apply_matrix(const matrix& mat, const curvy::circle& c);
matrix rotation_matrix(double cos_theta, double sin_theta);
matrix rotation_matrix(double theta);
matrix translation_matrix(double x, double y);
matrix scale_matrix(double x_scale, double y_scale);
matrix scale_matrix(double scale);
matrix translation_matrix(const point& pt);
matrix identity_matrix();
double euclidean_distance(double x1, double y1, double x2, double y2);
double euclidean_distance(const point& p1, const point& p2);
std::tuple<int, int> to_scr_coords(double x, double y, double logical_sz, int pixel_sz);
std::tuple<int, int> to_scr_coords(const point& p, double logical_sz, int pixel_sz);
std::tuple<double, double> from_scr_coords(int x, int y, double logical_sz, int pixel_sz);
std::tuple<double, double> from_scr_coords(const std::tuple<int,int>& pt, double logical_sz, int pixel_sz);
std::tuple<int, int, int, int> to_scr_coords(double x1, double y1, double x2, double y2, double logical_sz, int pixel_sz);
std::tuple<int, int, int, int> to_scr_coords(const std::tuple<double, double, double, double>& rect , double logical_sz, int pixel_sz);
curvy::circle circle_through_point(const point& pt);
curvy::circle circle_through_point(const point& from_pt, const point& to_pt, double from_direction);
bool is_pt_on_circle(const curvy::circle& c, const point& pt, double eps);
std::tuple<double, double> closest_pt_on_circle(const curvy::circle& c, const point& pt);
double get_angle_to_pt(const point& from_pt, const point& to_pt);
std::tuple<double, double> rotate_about_pt(const point& center_of_rot, double theta, const point& pt);