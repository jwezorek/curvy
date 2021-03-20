#include "circle.h"
#include <cmath>

const long double g_pi = std::acos(-1.L);

double pi() {
    return g_pi;
}

double normalize(const double value, const double start, const double end) {
    const double width = end - start;
    const double offsetValue = value - start;
    return (offsetValue - (floor(offsetValue / width) * width)) + start;
}

double normalize_angle(const double theta) {
    return normalize(theta, 0, 2 * pi());
}

/*-----------------------------------------------------------------------------------------------------------------------------*/

curvy::circle::circle(double cx, double cy, double r) :
    x(cx), y(cy), r( std::abs(r) )
{
}

curvy::circle::circle(const std::tuple<double, double>& pt, double r) :
    circle( std::get<0>(pt), std::get<1>(pt), r)
{
}

std::tuple<double, double, double, double> curvy::circle::bounding_box() const {
    return {
        x - r,
        y - r,
        x + r,
        y + r
    };
}

std::tuple<double, double> curvy::circle::center() const{
    return { x,y };
}

/*-----------------------------------------------------------------------------------------------------------------------------*/

std::tuple<double, double> curvy::circular_vector::position() const {
    return {
        circle.x + circle.r * std::cos(theta),
        circle.y + circle.r * std::sin(theta)
    };
}

double curvy::circular_vector::get_direction_angle() const {
    return get_direction_angle(theta, magnitude_angle);
}

double curvy::circular_vector::get_direction_angle(double angle, double s) const {
    auto direction_angle = (s > 0) ?
        angle + 0.5 * pi() :
        angle + 1.5 * pi();
    return normalize_angle(direction_angle);
}
