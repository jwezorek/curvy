#include "util.h"
#include <cmath>

namespace {
    using vec = Eigen::Matrix<double, 3, 1>;
    const long double g_pi = std::acos(-1.L);
}

double curvy::pi() {
    return g_pi;
}

double curvy::pi_over_two()
{
    static double pi_over_two = g_pi / 2.0;
    return pi_over_two;
}

double curvy::three_pi_over_two()
{
    static double three_pi_over_two = 3.0 * g_pi / 2.0;
    return three_pi_over_two;
}

double curvy::two_pi()
{
    static double two_pi = 2 * g_pi;
    return two_pi;
}

double curvy::normalize(const double value, const double start, const double end) {
    const double width = end - start;
    const double offsetValue = value - start;
    return (offsetValue - (floor(offsetValue / width) * width)) + start;
}

double curvy::normalize_angle(const double theta) {
    return normalize(theta, 0, 2 * pi());
}

double curvy::euclidean_distance(double x1, double y1, double x2, double y2) {
    auto diff_x = x2 - x1;
    auto diff_y = y2 - y1;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

double curvy::euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2) {
    auto [x1, y1] = p1;
    auto [x2, y2] = p2;
    return euclidean_distance(x1, y1, x2, y2);
}

curvy::point curvy::operator-(const curvy::point& a) {
    auto [x, y] = a;
    return { -x, -y };
}

curvy::point curvy::operator+(const point& a, const point& b)
{
    auto [x1, y1] = a;
    auto [x2, y2] = b;
    return { x1 + x2 , y1 + y2 };
}

curvy::point curvy::apply_matrix(const curvy::matrix& mat, const curvy::point& pt)
{
    vec v;
    v << std::get<0>(pt), std::get<1>(pt), 1.0;
    v = mat * v;
    return { v[0], v[1] };
}

curvy::matrix curvy::rotation_matrix(double cos_theta, double sin_theta) {
    curvy::matrix rotation;
    rotation <<
        cos_theta, -sin_theta, 0,
        sin_theta, cos_theta, 0,
        0, 0, 1;
    return rotation;
}

curvy::matrix curvy::rotation_matrix(double theta) {
    return rotation_matrix(std::cos(theta), std::sin(theta));
}

curvy::matrix curvy::translation_matrix(double x, double y) {
    matrix translation;
    translation <<
        1, 0, x,
        0, 1, y,
        0, 0, 1;
    return translation;
}

curvy::matrix curvy::scale_matrix(double x_scale, double y_scale) {
    matrix scale;
    scale <<
        x_scale, 0, 0,
        0, y_scale, 0,
        0, 0, 1;
    return scale;
}

curvy::matrix curvy::scale_matrix(double scale) {
    matrix mat;
    mat <<
        scale, 0, 0,
        0, scale, 0,
        0, 0, 1;
    return mat;
}

curvy::matrix curvy::identity_matrix() {
    return scale_matrix(1.0);
}

curvy::matrix curvy::translation_matrix(const curvy::point& pt) {
    return translation_matrix( std::get<0>(pt), std::get<1>(pt) );
}

std::tuple<int, int> curvy::to_scr_coords(double x, double y, double logical_sz, int pixel_sz)
{
    y *= -1;

    x += logical_sz / 2;
    y += logical_sz / 2;

    double log_to_scr = pixel_sz / logical_sz;

    x *= log_to_scr;
    y *= log_to_scr;

    return std::tuple<int, int>(
        static_cast<int>(std::round(x)),
        static_cast<int>(std::round(y))
    );
}

std::tuple<int, int> curvy::to_scr_coords(const curvy::point& p, double logical_sz, int pixel_sz)
{
    auto [x, y] = p; 
    return to_scr_coords(x, y, logical_sz, pixel_sz);
}

std::tuple<double, double> curvy::from_scr_coords(int xx, int yy, double logical_sz, int pixel_sz)
{
    double scr_to_log = logical_sz / pixel_sz;
    double x = xx * scr_to_log;
    double y = yy * scr_to_log;

    x -= logical_sz / 2.0;
    y -= logical_sz / 2.0;

    y *= -1;

    return { x,y };
}

std::tuple<double, double> curvy::from_scr_coords(const std::tuple<int, int>& pt, double logical_sz, int pixel_sz)
{
    auto [x, y] = pt;
    return from_scr_coords(x, y, logical_sz, pixel_sz);
}

double curvy::get_angle_to_pt(const curvy::point& from_pt, const curvy::point& to_pt)
{
    auto [fx, fy] = from_pt;
    auto [tx, ty] = to_pt;
    return std::atan2(ty - fy, tx - fx);
}

bool curvy::is_to_the_right_of(double from_direction, const point& from_pt, const point& to_pt)
{
    matrix mat1 = rotation_matrix( -from_direction) * translation_matrix(-from_pt);
    auto rotated_pt = apply_matrix(mat1, to_pt);
    return std::get<1>(to_pt) < 0;
}

std::tuple<int, int, int, int> curvy::to_scr_coords(double x1, double y1, double x2, double y2, double logical_sz, int pixel_sz)
{
    y1 *= -1;
    y2 *= -1;

    x1 += logical_sz / 2;
    y1 += logical_sz / 2;
    x2 += logical_sz / 2;
    y2 += logical_sz / 2;

    double log_to_scr = pixel_sz / logical_sz;

    x1 *= log_to_scr;
    y1 *= log_to_scr;
    x2 *= log_to_scr;
    y2 *= log_to_scr;

    return std::tuple<int, int, int, int>(
        static_cast<int>(std::round(x1)),
        static_cast<int>(std::round(y1)),
        static_cast<int>(std::round(x2)),
        static_cast<int>(std::round(y2))
    );
}

std::tuple<int, int, int, int> curvy::to_scr_coords(const std::tuple<double, double, double, double>& rect, double logical_sz, int pixel_sz) {
    auto [x1, y1, x2, y2] = rect;
    return to_scr_coords(x1, y1, x2, y2, logical_sz, pixel_sz);
}

double curvy::direction_on_circle( double angle, bool orientation)
{
    auto direction_angle = orientation ?
        angle + 0.5 * pi() :
        angle + 1.5 * pi();
    return normalize_angle(direction_angle);
}


