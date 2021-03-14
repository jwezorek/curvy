#include "util.h"
#include <cmath>

const long double g_pi = std::acos(-1.L);

double euclidean_distance(double x1, double y1, double x2, double y2) {
    auto diff_x = x2 - x1;
    auto diff_y = y2 - y1;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2) {
    auto [x1, y1] = p1;
    auto [x2, y2] = p2;
    return euclidean_distance(x1, y1, x2, y2);
}

double normalize(const double value, const double start, const double end) {
    const double width = end - start;
    const double offsetValue = value - start;
    return (offsetValue - (floor(offsetValue / width) * width)) + start;
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

double pi() {
    return g_pi;
}

curvy::circle circle_through_point(const std::tuple<double, double>& pt)
{
    auto [x, y] = pt;
    auto r = (x * x + y * y) / (2 * x);
    return curvy::circle(r, 0, r);
}

bool is_pt_on_circle(const curvy::circle& c, std::tuple<double, double> pt, double eps)
{
    auto distance = euclidean_distance(c.center(), pt);
    return std::abs(distance - c.r) <= eps;
}

std::tuple<double, double> closest_pt_on_circle(const curvy::circle& c, std::tuple<double, double> pt)
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

double get_angle_to_pt(const std::tuple<double, double>& from_pt, const std::tuple<double, double>& to_pt)
{
    auto [fx, fy] = from_pt;
    auto [tx, ty] = to_pt;
    return std::atan2(ty - fy, tx - fx);
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
