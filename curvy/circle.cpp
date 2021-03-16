#include "circle.h"
#include <cmath>

const long double g_pi = std::acos(-1.L);

double pi() {
    return g_pi;
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

std::tuple<double, double> curvy::circle::center() const
{
    return { x,y };
}

/*-----------------------------------------------------------------------------------------------------------------------------*/

std::tuple<double, double> curvy::circle_rotation_state::position() const {
    return {
        circle.x + circle.r * std::cos(theta),
        circle.y + circle.r * std::sin(theta)
    };
}

double curvy::circle_rotation_state::get_direction_angle() const
{
    if (speed > 0)
        return theta + 0.5 * pi();
    else
        return theta + 1.5 * pi();
}
