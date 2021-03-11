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

double pi() {
    return g_pi;
}