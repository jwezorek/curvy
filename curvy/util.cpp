#include "util.h"
#include <cmath>
using vec = Eigen::Matrix<double, 3, 1>;

point operator-(const point& a) {
    auto [x, y] = a;
    return { -x, -y };
}

point apply_matrix(const matrix& mat, const point& pt)
{
    vec v;
    v << std::get<0>(pt), std::get<1>(pt), 1.0;
    v = mat * v;
    return { v[0], v[1] };
}



curvy::circle apply_matrix(const matrix& mat, const curvy::circle& c)
{
    return curvy::circle(
        apply_matrix(mat, c.center()),
        c.r
    );
}

matrix rotation_matrix(double cos_theta, double sin_theta) {
    matrix rotation;
    rotation <<
        cos_theta, -sin_theta, 0,
        sin_theta, cos_theta, 0,
        0, 0, 1;
    return rotation;
}

matrix rotation_matrix(double theta) {
    return rotation_matrix(std::cos(theta), std::sin(theta));
}

matrix translation_matrix(double x, double y) {
    matrix translation;
    translation <<
        1, 0, x,
        0, 1, y,
        0, 0, 1;
    return translation;
}

matrix scale_matrix(double x_scale, double y_scale) {
    matrix scale;
    scale <<
        x_scale, 0, 0,
        0, y_scale, 0,
        0, 0, 1;
    return scale;
}

matrix scale_matrix(double scale) {
   matrix mat;
    mat <<
        scale, 0, 0,
        0, scale, 0,
        0, 0, 1;
    return mat;
}

matrix identity_matrix() {
    return scale_matrix(1.0);
}

matrix translation_matrix(const point& pt) {
    return translation_matrix( std::get<0>(pt), std::get<1>(pt) );
}

double euclidean_distance(double x1, double y1, double x2, double y2) {
    auto diff_x = x2 - x1;
    auto diff_y = y2 - y1;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

double euclidean_distance(const point& p1, const point& p2) {
    auto [x1, y1] = p1;
    auto [x2, y2] = p2;
    return euclidean_distance(x1, y1, x2, y2);
}

std::tuple<int, int> to_scr_coords(double x, double y, double logical_sz, int pixel_sz)
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

std::tuple<int, int> to_scr_coords(const point& p, double logical_sz, int pixel_sz)
{
    auto [x, y] = p; 
    return to_scr_coords(x, y, logical_sz, pixel_sz);
}

std::tuple<double, double> from_scr_coords(int xx, int yy, double logical_sz, int pixel_sz)
{
    double scr_to_log = logical_sz / pixel_sz;
    double x = xx * scr_to_log;
    double y = yy * scr_to_log;

    x -= logical_sz / 2.0;
    y -= logical_sz / 2.0;

    y *= -1;

    return { x,y };
}

std::tuple<double, double> from_scr_coords(const std::tuple<int, int>& pt, double logical_sz, int pixel_sz)
{
    auto [x, y] = pt;
    return from_scr_coords(x, y, logical_sz, pixel_sz);
}



curvy::circle circle_through_point(const point& pt)
{
    auto [x, y] = pt;
    auto r = (x * x + y * y) / (2 * x);
    return curvy::circle(r, 0, r);
}

curvy::circle circle_through_point(const point& from_pt, const point& to_pt, double from_direction)
{
    matrix mat1 = rotation_matrix(pi() / 2.0 - from_direction) * translation_matrix(-from_pt);
    matrix mat2 = translation_matrix(from_pt) * rotation_matrix(from_direction - pi() / 2.0 );
    point transform_pt = apply_matrix(mat1, to_pt);
    auto c = circle_through_point(transform_pt);
    c = apply_matrix(mat2, c);
    return c;
}

bool is_pt_on_circle(const curvy::circle& c, const point& pt, double eps)
{
    auto distance = euclidean_distance(c.center(), pt);
    return std::abs(distance - c.r) <= eps;
}

std::tuple<double, double> closest_pt_on_circle(const curvy::circle& c, const point& pt)
{
    // https://math.stackexchange.com/a/127615/63016

    auto [cx, cy] = c.center();
    auto [px, py] = pt;
    auto distance_to_center = euclidean_distance(cx, cy, px, py);
    return {
        cx + c.r * (px - cx) / distance_to_center,
        cy + c.r * (py - cy) / distance_to_center
    };
}

double get_angle_to_pt(const point& from_pt, const point& to_pt)
{
    auto [fx, fy] = from_pt;
    auto [tx, ty] = to_pt;
    return std::atan2(ty - fy, tx - fx);
}

std::tuple<double, double> rotate_about_pt(const point& center_of_rot, double theta, const std::tuple<double, double>& pt)
{
    matrix mat = translation_matrix(-pt) * rotation_matrix(theta) * translation_matrix(pt);
    return apply_matrix(mat, pt);
}

std::tuple<int, int, int, int> to_scr_coords(double x1, double y1, double x2, double y2, double logical_sz, int pixel_sz) 
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

std::tuple<int, int, int, int> to_scr_coords(const std::tuple<double, double, double, double>& rect, double logical_sz, int pixel_sz) {
    auto [x1, y1, x2, y2] = rect;
    return to_scr_coords(x1, y1, x2, y2, logical_sz, pixel_sz);
}


