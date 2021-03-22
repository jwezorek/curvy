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

double euclidean_distance(double x1, double y1, double x2, double y2) {
    auto diff_x = x2 - x1;
    auto diff_y = y2 - y1;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

double euclidean_distance(const std::tuple<double,double>& p1, const std::tuple<double, double>& p2) {
    auto [x1, y1] = p1;
    auto [x2, y2] = p2;
    return euclidean_distance(x1, y1, x2, y2);
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

double curvy::circle::circumference() const
{
    return 2.0 * pi() * r;
}

std::tuple<double, double> curvy::circle::invert(const std::tuple<double, double>& pt) const
{
    auto [cx, cy] = center();
    auto R = euclidean_distance( center(), pt );
    auto [px, py] = pt;
    auto cosine_theta = (px - x) / R;
    auto sine_theta = (py - y) / R;
    auto inverted_dist = r * r / R;
     return {
        cx + inverted_dist * cosine_theta,
        cy + inverted_dist * sine_theta
    };
}

double curvy::circle::diameter() const
{
    return 2.0 * r;
}

/*-----------------------------------------------------------------------------------------------------------------------------*/

std::tuple<double, double> curvy::circular_vector::position() const {
    return {
        circle.x + circle.r * std::cos(theta),
        circle.y + circle.r * std::sin(theta)
    };
}

double curvy::circular_vector::direction_angle() const {
    return direction_angle(theta, angular_magnitude);
}

double curvy::circular_vector::direction_angle(double angle, double s) const {
    auto direction_angle = (s > 0) ?
        angle + 0.5 * pi() :
        angle + 1.5 * pi();
    return normalize_angle(direction_angle);
}

double curvy::circular_vector::linear_magnitude() const
{
    return angular_magnitude * circle.r;
}

double curvy::circular_vector::circumference() const
{
    return circle.circumference();
}

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const circle& circ, double theta, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.r;
    return circular_vector(circ, theta, angular_magnitude);
}

curvy::circular_vector curvy::operator*(double scalar, const circular_vector& cv)
{
    auto scaled = curvy::circular_vector(cv);
    scaled.angular_magnitude *= scalar;
    return scaled;
}
