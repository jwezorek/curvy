#include "util.h"
#include <Windows.h>
#include <cmath>
#include <array>
#include <boost/math/tools/roots.hpp>

namespace {
    using vec = Eigen::Matrix<double, 3, 1>;
    const long double g_pi = std::acos(-1.L);

    double get_triangle_orientation(const curvy::point& p1, const curvy::point& p2, const curvy::point& p3) {
        auto [x1, y1] = p1;
        auto [x2, y2] = p2;
        auto [x3, y3] = p3;
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    }

    // std::fmod just returns the mod of the absolute value with the sign of the divisor, I think.
    // we want the Donald Knuth et. al. preferred float mod.

    double floored_division_modulo(double a, double n) {
        return a - std::floor(a / n) * n;
    }

    // return the difference of theta1 and theta2 with the sign such that the returned difference 
    // plus theta2 equals theta1...

    double angle_delta(double theta1, double theta2) {
        auto delta = theta1 - theta2;
        return floored_division_modulo(delta + curvy::pi(), curvy::two_pi()) - curvy::pi();
    }

}

double curvy::eps()
{
    return 1e-10;
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
    return normalize(theta, -pi(), pi());
}

curvy::point curvy::normalize_pt(const point& pt)
{
    auto [x, y] = pt;
    auto hypot = std::hypot(x, y);
    return { x / hypot, y / hypot };
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

curvy::point curvy::operator-(const point& a, const point& b)
{
    return a + (-b);
}

curvy::point curvy::operator*(double scale, const point& a)
{
    auto [x, y] = a;
    return point{ scale * x, scale * y };
}

curvy::point curvy::operator*(const point& a, double scale)
{
    return scale * a;
}

double curvy::dot_product(const point& a, const point& b)
{
    auto [x1, y1] = a;
    auto [x2, y2] = b;
    return x1 * x2 + y1 * y2;
}

std::string curvy::to_string(const point& pt)
{
    auto [x, y] = pt;
    return "( " + std::to_string(x) + " , " + std::to_string(y) + " )";
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

double curvy::angle_to_pt(const curvy::point& from_pt, const curvy::point& to_pt)
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

double curvy::atan_of_pt(const point& pt)
{
    auto [dx, dy] = pt;
    return std::atan2(dy, dx);
}

double curvy::hypot_of_point(const point& pt)
{
    auto [x, y] = pt;
    return std::hypot(x, y);
}

double curvy::angle_to_point_relative_to_direction(const point& from_pt, double direction_at_from, const point& to_pt)
{
    return normalize_angle(curvy::angle_to_pt(from_pt, to_pt) - direction_at_from);
}

void curvy::output_debug_message(const std::string& msg)
{
    std::string str = msg + "\n";
    OutputDebugStringA(str.c_str());
}

void curvy::output_debug_message(const std::string& label, double value)
{
    std::string msg = label + " => " + std::to_string(value);
    output_debug_message(msg);
}

curvy::point curvy::pt_on_unit_circle(double theta)
{
    return {
        std::cos(theta),
        std::sin(theta)
    };
}

bool curvy::pt_in_triangle(const curvy::point& pt, const curvy::point& v1, const curvy::point& v2, const curvy::point& v3)
{
    auto d1 = get_triangle_orientation(pt, v1, v2);
    auto d2 = get_triangle_orientation(pt, v2, v3);
    auto d3 = get_triangle_orientation(pt, v3, v1);

    auto has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    auto has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

std::optional<curvy::point> curvy::line_intersection(const std::tuple<point, point>& line1, const std::tuple<point, point>& line2)
{
    auto [line1_1, line1_2] = line1;
    auto [line2_1, line2_2] = line2;
    auto [x1, y1] = line1_1;
    auto [x2, y2] = line1_2;
    auto [x3, y3] = line2_1;
    auto [x4, y4] = line2_2;
    auto D = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

    if (!D)
        return std::nullopt;

    return {{
        ((x1*y2 - y1*x2)*(x3-x4) - (x1 - x2)*(x3*y4 - y3*x4)) / D,
        ((x1*y2 - y1*x2)*(y3-y4) - (y1 - y2)*(x3*y4 - y3*x4)) / D
    }};
}

double curvy::to_degrees(double radians)
{
    return radians * 180.0 / curvy::pi();
}

double curvy::from_degrees(double degrees)
{
    return degrees * curvy::pi() / 180.0;
}

struct circles_traveling_in_circles {
    circles_traveling_in_circles(double r1, double theta1, double a1, double cx, double cy, double r2, double theta2, double a2, double d) :
        r1(r1), theta1(theta1), a1(a1), cx(cx), cy(cy), r2(r2), theta2(theta2), a2(a2), d(d) {}

    std::tuple<double, double, double> operator()(double const& t)
    {
        // Return f(x) and f'(x) and f''(x).
        // f(x)...
        double cosine_term = r1 * std::cos(theta1 + t * a1) - (cx + r2 * std::cos(theta2 + t * a2));
        double sine_term = r1 * std::sin(theta1 + t * a1) - (cy + r2 * std::sin(theta2 + t * a2));
        double fx = cosine_term * cosine_term + sine_term * sine_term - d * d;

        // f'(x)
        double dx = 2.0 * (a1 * r1 * std::cos(a1 * t + theta1) - a2 * r2 * std::cos(a2 * t + theta2)) *
            (-cy + r1 * std::sin(a1 * t + theta1) - r2 * std::sin(a2 * t + theta2)) +
            2.0 * (-cx + r1 * std::cos(a1 * t + theta1) - r2 * std::cos(a2 * t + theta2)) *
            (-a1 * r1 * std::sin(a1 * t + theta1) + a2 * r2 * std::sin(a2 * t + theta2));

        // f''(x)
        double cosine_term_2 = a1 * r1 * std::cos(a1 * t + theta1) - a2 * r2 * std::cos(a2 * t + theta2);
        double sine_term_2 = -a1 * r1 * std::sin(a1 * t + theta1) + a2 * r2 * std::sin(a2 * t + theta2);
        double d2x = 2.0 * cosine_term_2 * cosine_term_2 +
            2.0 * (-cx + r1 * std::cos(a1 * t + theta1) - r2 * std::cos(a2 * t + theta2)) *
            (-(a1 * a1) * r1 * std::cos(a1 * t + theta1) + a2 * a2 * r2 * std::cos(a2 * t + theta2)) +
            2.0 * sine_term_2 * sine_term_2 +
            2.0 * (-cy + r1 * std::sin(a1 * t + theta1) - r2 * std::sin(a2 * t + theta2)) *
            (-(a1 * a1) * r1 * std::sin(a1 * t + theta1) + a2 * a2 * r2 * std::sin(a2 * t + theta2));

        return { fx, dx, d2x };  // 'return' fx, dx and d2x.
    }

private:
    double r1, theta1, a1, cx, cy, r2, theta2, a2, d;
};

double solve_circles_traveling_in_circles(double r1, double theta1, double a1, double cx, double cy, double r2, double theta2, double a2, double d, double t2)
{
    using namespace boost::math::tools;
    const int digits = std::numeric_limits<double>::digits;  
    int get_digits = static_cast<int>(digits - 4);    
                                                       
    boost::uintmax_t maxit = 20;
    double result = halley_iterate(
        circles_traveling_in_circles(r1, theta1, a1, cx, cy, r2, theta2, a2, d),
        t2 / 2.0, 0.0, t2, get_digits, maxit);

    return result;
}

double curvy::circles_traveling_in_circles_collision_time(double r1, double theta1, double a1, double cx, double cy, double r2, double theta2, double a2, double d, double t2)
{
    return solve_circles_traveling_in_circles(r1, theta1, a1, cx, cy, r2, theta2, a2, d, t2);
}
/*
std::optional<double> curvy::circle_traveling_in_circle_collision_time_with_circular_border(double R, double r, double cx, double cy, double a, double theta, double d, double t2)
{
    using namespace std;
    auto arc_cosine = (-(r * (4 * pow(cx, 3) * cos(theta) + 4 * cx * pow(cy, 2) * cos(theta) - 4 * cx * pow(d, 2) * cos(theta) + 4 * cx * pow(r, 2) * cos(theta) + 8 * cx * d * R * cos(theta) - 4 * cx * pow(R, 2) * cos(theta) +
        4 * pow(cx, 2) * cy * sin(theta) + 4 * pow(cy, 3) * sin(theta) - 4 * cy * pow(d, 2) * sin(theta) + 4 * cy * pow(r, 2) * sin(theta) + 8 * cy * d * R * sin(theta) - 4 * cy * pow(R, 2) * sin(theta))) +
        sqrt(pow(r, 2) * pow(4 * pow(cx, 3) * cos(theta) + 4 * cx * pow(cy, 2) * cos(theta) - 4 * cx * pow(d, 2) * cos(theta) + 4 * cx * pow(r, 2) * cos(theta) + 8 * cx * d * R * cos(theta) - 4 * cx * pow(R, 2) * cos(theta) +
            4 * pow(cx, 2) * cy * sin(theta) + 4 * pow(cy, 3) * sin(theta) - 4 * cy * pow(d, 2) * sin(theta) + 4 * cy * pow(r, 2) * sin(theta) + 8 * cy * d * R * sin(theta) - 4 * cy * pow(R, 2) * sin(theta), 2) -
            4 * pow(r, 2) * (4 * pow(cx, 2) * pow(cos(theta), 2) + 4 * pow(cy, 2) * pow(cos(theta), 2) + 4 * pow(cx, 2) * pow(sin(theta), 2) + 4 * pow(cy, 2) * pow(sin(theta), 2)) *
            (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) + 2 * pow(cx, 2) * pow(r, 2) + 2 * pow(cy, 2) * pow(r, 2) -
                2 * pow(d, 2) * pow(r, 2) + pow(r, 4) + 4 * pow(cx, 2) * d * R + 4 * pow(cy, 2) * d * R - 4 * pow(d, 3) * R + 4 * d * pow(r, 2) * R - 2 * pow(cx, 2) * pow(R, 2) - 2 * pow(cy, 2) * pow(R, 2) + 6 * pow(d, 2) * pow(R, 2) -
                2 * pow(r, 2) * pow(R, 2) - 4 * d * pow(R, 3) + pow(R, 4) - 4 * pow(cy, 2) * pow(r, 2) * pow(cos(theta), 2) + 8 * cx * cy * pow(r, 2) * cos(theta) * sin(theta) - 4 * pow(cx, 2) * pow(r, 2) * pow(sin(theta), 2)))) /
        (2. * pow(r, 2) * (4 * pow(cx, 2) * pow(cos(theta), 2) + 4 * pow(cy, 2) * pow(cos(theta), 2) + 4 * pow(cx, 2) * pow(sin(theta), 2) + 4 * pow(cy, 2) * pow(sin(theta), 2)));
    
    if (arc_cosine > 1.0) // sometimes this can come out to being some tiny epsilon above 1.0 due to floating point bullshit...
        return 0.0;

    auto t = acos(arc_cosine) / a;
    if (t >= 0 && t <= t2)
        return t;

    t *= -1.0;
    if (t >= 0 && t <= t2)
        return t;

    return std::nullopt;
}
*/


std::optional<double> curvy::circle_traveling_in_circle_collision_time_with_circular_border(double R, double r, double cx, double cy, double a, double theta, double d, double t2)
{
    return circle_traveling_in_circle_collision_time_with_stationary_circle(r, cx, cy, a, theta, 0, 0, R - d, t2);
}

std::optional<double> curvy::circle_traveling_in_circle_collision_time_with_stationary_circle(double r, double cx, double cy, double a, double theta, double px, double py, double d, double t2)
{
    using namespace std;

    if (a == 0)
        return nullopt;

    //mathematica reverses the params of two param arctangent...
    auto arctan = [](double x, double y) {return std::atan2(y, x); };

    std::array<double, 2> candidates = {
        (-theta + arctan((-4 * pow(cx, 3) * r - 4 * cx * pow(cy, 2) * r + 4 * cx * pow(d, 2) * r + 12 * pow(cx, 2) * px * r + 4 * pow(cy, 2) * px * r - 4 * pow(d, 2) * px * r - 12 * cx * pow(px, 2) * r + 4 * pow(px, 3) * r + 8 * cx * cy * py * r - 8 * cy * px * py * r - 4 * cx * pow(py, 2) * r + 4 * px * pow(py, 2) * r -
        4 * cx * pow(r, 3) + 4 * px * pow(r, 3) - sqrt(pow(4 * pow(cx, 3) * r + 4 * cx * pow(cy, 2) * r - 4 * cx * pow(d, 2) * r - 12 * pow(cx, 2) * px * r - 4 * pow(cy, 2) * px * r + 4 * pow(d, 2) * px * r + 12 * cx * pow(px, 2) * r - 4 * pow(px, 3) * r - 8 * cx * cy * py * r + 8 * cy * px * py * r + 4 * cx * pow(py, 2) * r -
            4 * px * pow(py, 2) * r + 4 * cx * pow(r, 3) - 4 * px * pow(r, 3), 2) - 4 * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) *
            (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) - 4 * pow(cx, 3) * px - 4 * cx * pow(cy, 2) * px + 4 * cx * pow(d, 2) * px + 6 * pow(cx, 2) * pow(px, 2) + 2 * pow(cy, 2) * pow(px, 2) -
                2 * pow(d, 2) * pow(px, 2) - 4 * cx * pow(px, 3) + pow(px, 4) - 4 * pow(cx, 2) * cy * py - 4 * pow(cy, 3) * py + 4 * cy * pow(d, 2) * py + 8 * cx * cy * px * py - 4 * cy * pow(px, 2) * py + 2 * pow(cx, 2) * pow(py, 2) + 6 * pow(cy, 2) * pow(py, 2) - 2 * pow(d, 2) * pow(py, 2) -
                4 * cx * px * pow(py, 2) + 2 * pow(px, 2) * pow(py, 2) - 4 * cy * pow(py, 3) + pow(py, 4) + 2 * pow(cx, 2) * pow(r, 2) - 2 * pow(cy, 2) * pow(r, 2) - 2 * pow(d, 2) * pow(r, 2) - 4 * cx * px * pow(r, 2) + 2 * pow(px, 2) * pow(r, 2) + 4 * cy * py * pow(r, 2) -
                2 * pow(py, 2) * pow(r, 2) + pow(r, 4)))) / (2. * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2))),
        (-pow(cx, 2) - pow(cy, 2) + pow(d, 2) + 2 * cx * px - pow(px, 2) + 2 * cy * py - pow(py, 2) - pow(r, 2) +
            (4 * pow(cx, 4) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cx, 2) * pow(cy, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (4 * pow(cx, 2) * pow(d, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (16 * pow(cx, 3) * px * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cx * pow(cy, 2) * px * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (8 * cx * pow(d, 2) * px * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (24 * pow(cx, 2) * pow(px, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cy, 2) * pow(px, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (4 * pow(d, 2) * pow(px, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (16 * cx * pow(px, 3) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(px, 4) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * pow(cx, 2) * cy * py * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (16 * cx * cy * px * py * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cy * pow(px, 2) * py * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cx, 2) * pow(py, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cx * px * pow(py, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(px, 2) * pow(py, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cx, 2) * pow(r, 4)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cx * px * pow(r, 4)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(px, 2) * pow(r, 4)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (cx * r * sqrt(pow(4 * pow(cx, 3) * r + 4 * cx * pow(cy, 2) * r - 4 * cx * pow(d, 2) * r - 12 * pow(cx, 2) * px * r - 4 * pow(cy, 2) * px * r + 4 * pow(d, 2) * px * r + 12 * cx * pow(px, 2) * r - 4 * pow(px, 3) * r - 8 * cx * cy * py * r + 8 * cy * px * py * r + 4 * cx * pow(py, 2) * r - 4 * px * pow(py, 2) * r +
                4 * cx * pow(r, 3) - 4 * px * pow(r, 3), 2) - 4 * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) *
                (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) - 4 * pow(cx, 3) * px - 4 * cx * pow(cy, 2) * px + 4 * cx * pow(d, 2) * px + 6 * pow(cx, 2) * pow(px, 2) + 2 * pow(cy, 2) * pow(px, 2) -
                    2 * pow(d, 2) * pow(px, 2) - 4 * cx * pow(px, 3) + pow(px, 4) - 4 * pow(cx, 2) * cy * py - 4 * pow(cy, 3) * py + 4 * cy * pow(d, 2) * py + 8 * cx * cy * px * py - 4 * cy * pow(px, 2) * py + 2 * pow(cx, 2) * pow(py, 2) + 6 * pow(cy, 2) * pow(py, 2) - 2 * pow(d, 2) * pow(py, 2) -
                    4 * cx * px * pow(py, 2) + 2 * pow(px, 2) * pow(py, 2) - 4 * cy * pow(py, 3) + pow(py, 4) + 2 * pow(cx, 2) * pow(r, 2) - 2 * pow(cy, 2) * pow(r, 2) - 2 * pow(d, 2) * pow(r, 2) - 4 * cx * px * pow(r, 2) + 2 * pow(px, 2) * pow(r, 2) + 4 * cy * py * pow(r, 2) -
                    2 * pow(py, 2) * pow(r, 2) + pow(r, 4)))) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (px * r * sqrt(pow(4 * pow(cx, 3) * r + 4 * cx * pow(cy, 2) * r - 4 * cx * pow(d, 2) * r - 12 * pow(cx, 2) * px * r - 4 * pow(cy, 2) * px * r + 4 * pow(d, 2) * px * r + 12 * cx * pow(px, 2) * r - 4 * pow(px, 3) * r - 8 * cx * cy * py * r + 8 * cy * px * py * r + 4 * cx * pow(py, 2) * r - 4 * px * pow(py, 2) * r +
                4 * cx * pow(r, 3) - 4 * px * pow(r, 3), 2) - 4 * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) *
                (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) - 4 * pow(cx, 3) * px - 4 * cx * pow(cy, 2) * px + 4 * cx * pow(d, 2) * px + 6 * pow(cx, 2) * pow(px, 2) + 2 * pow(cy, 2) * pow(px, 2) -
                    2 * pow(d, 2) * pow(px, 2) - 4 * cx * pow(px, 3) + pow(px, 4) - 4 * pow(cx, 2) * cy * py - 4 * pow(cy, 3) * py + 4 * cy * pow(d, 2) * py + 8 * cx * cy * px * py - 4 * cy * pow(px, 2) * py + 2 * pow(cx, 2) * pow(py, 2) + 6 * pow(cy, 2) * pow(py, 2) - 2 * pow(d, 2) * pow(py, 2) -
                    4 * cx * px * pow(py, 2) + 2 * pow(px, 2) * pow(py, 2) - 4 * cy * pow(py, 3) + pow(py, 4) + 2 * pow(cx, 2) * pow(r, 2) - 2 * pow(cy, 2) * pow(r, 2) - 2 * pow(d, 2) * pow(r, 2) - 4 * cx * px * pow(r, 2) + 2 * pow(px, 2) * pow(r, 2) + 4 * cy * py * pow(r, 2) -
                    2 * pow(py, 2) * pow(r, 2) + pow(r, 4)))) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2))) / (2 * cy * r - 2 * py * r))) / a,

        (-theta + arctan((-4 * pow(cx, 3) * r - 4 * cx * pow(cy, 2) * r + 4 * cx * pow(d, 2) * r + 12 * pow(cx, 2) * px * r + 4 * pow(cy, 2) * px * r - 4 * pow(d, 2) * px * r - 12 * cx * pow(px, 2) * r + 4 * pow(px, 3) * r + 8 * cx * cy * py * r - 8 * cy * px * py * r - 4 * cx * pow(py, 2) * r + 4 * px * pow(py, 2) * r -
        4 * cx * pow(r, 3) + 4 * px * pow(r, 3) + sqrt(pow(4 * pow(cx, 3) * r + 4 * cx * pow(cy, 2) * r - 4 * cx * pow(d, 2) * r - 12 * pow(cx, 2) * px * r - 4 * pow(cy, 2) * px * r + 4 * pow(d, 2) * px * r + 12 * cx * pow(px, 2) * r - 4 * pow(px, 3) * r - 8 * cx * cy * py * r + 8 * cy * px * py * r + 4 * cx * pow(py, 2) * r -
            4 * px * pow(py, 2) * r + 4 * cx * pow(r, 3) - 4 * px * pow(r, 3), 2) - 4 * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) *
            (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) - 4 * pow(cx, 3) * px - 4 * cx * pow(cy, 2) * px + 4 * cx * pow(d, 2) * px + 6 * pow(cx, 2) * pow(px, 2) + 2 * pow(cy, 2) * pow(px, 2) -
                2 * pow(d, 2) * pow(px, 2) - 4 * cx * pow(px, 3) + pow(px, 4) - 4 * pow(cx, 2) * cy * py - 4 * pow(cy, 3) * py + 4 * cy * pow(d, 2) * py + 8 * cx * cy * px * py - 4 * cy * pow(px, 2) * py + 2 * pow(cx, 2) * pow(py, 2) + 6 * pow(cy, 2) * pow(py, 2) - 2 * pow(d, 2) * pow(py, 2) -
                4 * cx * px * pow(py, 2) + 2 * pow(px, 2) * pow(py, 2) - 4 * cy * pow(py, 3) + pow(py, 4) + 2 * pow(cx, 2) * pow(r, 2) - 2 * pow(cy, 2) * pow(r, 2) - 2 * pow(d, 2) * pow(r, 2) - 4 * cx * px * pow(r, 2) + 2 * pow(px, 2) * pow(r, 2) + 4 * cy * py * pow(r, 2) -
                2 * pow(py, 2) * pow(r, 2) + pow(r, 4)))) / (2. * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2))),
        (-pow(cx, 2) - pow(cy, 2) + pow(d, 2) + 2 * cx * px - pow(px, 2) + 2 * cy * py - pow(py, 2) - pow(r, 2) +
            (4 * pow(cx, 4) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cx, 2) * pow(cy, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (4 * pow(cx, 2) * pow(d, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (16 * pow(cx, 3) * px * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cx * pow(cy, 2) * px * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (8 * cx * pow(d, 2) * px * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (24 * pow(cx, 2) * pow(px, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cy, 2) * pow(px, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (4 * pow(d, 2) * pow(px, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (16 * cx * pow(px, 3) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(px, 4) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * pow(cx, 2) * cy * py * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (16 * cx * cy * px * py * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cy * pow(px, 2) * py * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cx, 2) * pow(py, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cx * px * pow(py, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(px, 2) * pow(py, 2) * pow(r, 2)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(cx, 2) * pow(r, 4)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (8 * cx * px * pow(r, 4)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (4 * pow(px, 2) * pow(r, 4)) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) -
            (cx * r * sqrt(pow(4 * pow(cx, 3) * r + 4 * cx * pow(cy, 2) * r - 4 * cx * pow(d, 2) * r - 12 * pow(cx, 2) * px * r - 4 * pow(cy, 2) * px * r + 4 * pow(d, 2) * px * r + 12 * cx * pow(px, 2) * r - 4 * pow(px, 3) * r - 8 * cx * cy * py * r + 8 * cy * px * py * r + 4 * cx * pow(py, 2) * r - 4 * px * pow(py, 2) * r +
                4 * cx * pow(r, 3) - 4 * px * pow(r, 3), 2) - 4 * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) *
                (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) - 4 * pow(cx, 3) * px - 4 * cx * pow(cy, 2) * px + 4 * cx * pow(d, 2) * px + 6 * pow(cx, 2) * pow(px, 2) + 2 * pow(cy, 2) * pow(px, 2) -
                    2 * pow(d, 2) * pow(px, 2) - 4 * cx * pow(px, 3) + pow(px, 4) - 4 * pow(cx, 2) * cy * py - 4 * pow(cy, 3) * py + 4 * cy * pow(d, 2) * py + 8 * cx * cy * px * py - 4 * cy * pow(px, 2) * py + 2 * pow(cx, 2) * pow(py, 2) + 6 * pow(cy, 2) * pow(py, 2) - 2 * pow(d, 2) * pow(py, 2) -
                    4 * cx * px * pow(py, 2) + 2 * pow(px, 2) * pow(py, 2) - 4 * cy * pow(py, 3) + pow(py, 4) + 2 * pow(cx, 2) * pow(r, 2) - 2 * pow(cy, 2) * pow(r, 2) - 2 * pow(d, 2) * pow(r, 2) - 4 * cx * px * pow(r, 2) + 2 * pow(px, 2) * pow(r, 2) + 4 * cy * py * pow(r, 2) -
                    2 * pow(py, 2) * pow(r, 2) + pow(r, 4)))) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) +
            (px * r * sqrt(pow(4 * pow(cx, 3) * r + 4 * cx * pow(cy, 2) * r - 4 * cx * pow(d, 2) * r - 12 * pow(cx, 2) * px * r - 4 * pow(cy, 2) * px * r + 4 * pow(d, 2) * px * r + 12 * cx * pow(px, 2) * r - 4 * pow(px, 3) * r - 8 * cx * cy * py * r + 8 * cy * px * py * r + 4 * cx * pow(py, 2) * r - 4 * px * pow(py, 2) * r +
                4 * cx * pow(r, 3) - 4 * px * pow(r, 3), 2) - 4 * (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2)) *
                (pow(cx, 4) + 2 * pow(cx, 2) * pow(cy, 2) + pow(cy, 4) - 2 * pow(cx, 2) * pow(d, 2) - 2 * pow(cy, 2) * pow(d, 2) + pow(d, 4) - 4 * pow(cx, 3) * px - 4 * cx * pow(cy, 2) * px + 4 * cx * pow(d, 2) * px + 6 * pow(cx, 2) * pow(px, 2) + 2 * pow(cy, 2) * pow(px, 2) -
                    2 * pow(d, 2) * pow(px, 2) - 4 * cx * pow(px, 3) + pow(px, 4) - 4 * pow(cx, 2) * cy * py - 4 * pow(cy, 3) * py + 4 * cy * pow(d, 2) * py + 8 * cx * cy * px * py - 4 * cy * pow(px, 2) * py + 2 * pow(cx, 2) * pow(py, 2) + 6 * pow(cy, 2) * pow(py, 2) - 2 * pow(d, 2) * pow(py, 2) -
                    4 * cx * px * pow(py, 2) + 2 * pow(px, 2) * pow(py, 2) - 4 * cy * pow(py, 3) + pow(py, 4) + 2 * pow(cx, 2) * pow(r, 2) - 2 * pow(cy, 2) * pow(r, 2) - 2 * pow(d, 2) * pow(r, 2) - 4 * cx * px * pow(r, 2) + 2 * pow(px, 2) * pow(r, 2) + 4 * cy * py * pow(r, 2) -
                    2 * pow(py, 2) * pow(r, 2) + pow(r, 4)))) / (4 * pow(cx, 2) * pow(r, 2) + 4 * pow(cy, 2) * pow(r, 2) - 8 * cx * px * pow(r, 2) + 4 * pow(px, 2) * pow(r, 2) - 8 * cy * py * pow(r, 2) + 4 * pow(py, 2) * pow(r, 2))) / (2 * cy * r - 2 * py * r))) / a
    };

    for (auto candidate : candidates) {
        if (!std::isnan(candidate) && !std::isinf(candidate) && candidate >= 0 && candidate <= t2)
            return candidate;
    }

    return nullopt;
}


double curvy::lerp_angles(double theta1, double theta2, double t)
{
    auto delta = angle_delta(theta2, theta1);
    return delta * t + theta1;
}

// return theta2 such that lerp_angles(theta1, theta2, t) equals interpolated_angle...
double curvy::lerp_angles_inverse(double interpolated_angle, double theta1, double t)
{
    auto inverse = (interpolated_angle - theta1) / t + theta1;

    if (inverse > pi())
        inverse -= two_pi();
    else if (inverse < -pi())
        inverse += two_pi();

    return inverse;
}

double curvy::lerp_angles_inverse_2(double interpolated_angle, double theta2, double t)
{
    auto inverse = (-interpolated_angle + t * theta2) / (-1.0 + t);

    if (inverse > pi())
        inverse -= two_pi();
    else if (inverse < -pi())
        inverse += two_pi();

    return inverse;
}

double curvy::magnitude(const vec3& v)
{
    auto [x, y, z] = v;
    return std::sqrt(x * x + y * y + z * z);
}

curvy::vec3 curvy::operator*(const vec3& v, double k)
{
    return k * v;
}

curvy::vec3 curvy::operator*(double k, const vec3& v)
{
    auto [x, y, z] = v;
    return { k * x, k * y, k * z };
}

curvy::vec3 curvy::operator+(const vec3& v1, const vec3& v2)
{
    const auto& [x1, y1, z1] = v1;
    const auto& [x2, y2, z2] = v2;
    return {
        x1 + x2,
        y1 + y2,
        z1 + z2
    };
}

curvy::vec3 curvy::operator-(const curvy::vec3& a, const curvy::vec3& b)
{
    return a + (-1.0 * b);
}

curvy::vec3 curvy::normalize(const curvy::vec3& v)
{
    auto m = magnitude(v);
    if (m == 0)
        return { 0,0,0 };
    return (1.0 / m) * v;
}



