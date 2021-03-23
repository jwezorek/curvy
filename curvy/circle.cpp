#include "circle.h"
#include <cmath>



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

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double theta, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.r;
    return curvy::circular_vector(circ, theta, angular_magnitude);
}

curvy::circular_vector curvy::operator*(double scalar, const curvy::circular_vector& cv)
{
    auto scaled = curvy::circular_vector(cv);
    scaled.angular_magnitude *= scalar;
    return scaled;
}

curvy::circle curvy::apply_matrix(const curvy::matrix& mat, const curvy::circle& c)
{
    return curvy::circle(
        curvy::apply_matrix(mat, c.center()),
        c.r
    );
}

curvy::circular_vector curvy::apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& cv)
{
    auto center = curvy::apply_matrix(mat, cv.circle.center());
    auto pt = curvy::apply_matrix(mat, cv.position());
    auto circle = curvy::circle(center, curvy::euclidean_distance(center, pt));
    return curvy::circular_vector(
        circle,
        curvy::get_angle_to_pt(center, pt),
        cv.angular_magnitude
    );
}

bool is_pt_on_circle(const curvy::circle& c, const curvy::point& pt, double eps)
{
    auto distance = curvy::euclidean_distance(c.center(), pt);
    return std::abs(distance - c.r) <= eps;
}

bool is_pt_in_circle(const curvy::circle& c, const curvy::point& pt, double eps)
{
    if (eps > 0 && is_pt_on_circle(c, pt, eps))
        return true;
    else
        return (curvy::euclidean_distance(c.center(), pt) < c.r);
}

std::tuple<double, double> closest_pt_on_circle(const curvy::circle& c, const curvy::point& pt)
{
    // https://math.stackexchange.com/a/127615/63016

    auto [cx, cy] = c.center();
    auto [px, py] = pt;
    auto distance_to_center = curvy::euclidean_distance(cx, cy, px, py);
    return {
        cx + c.r * (px - cx) / distance_to_center,
        cy + c.r * (py - cy) / distance_to_center
    };
}