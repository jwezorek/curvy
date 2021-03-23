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

curvy::circle curvy::apply_matrix(const curvy::matrix& mat, const curvy::circle& c)
{
    return curvy::circle(
        curvy::apply_matrix(mat, c.center()),
        c.r
    );
}

bool curvy::is_pt_on_circle(const curvy::circle& c, const curvy::point& pt, double eps)
{
    auto distance = curvy::euclidean_distance(c.center(), pt);
    return std::abs(distance - c.r) <= eps;
}

bool curvy::is_pt_in_circle(const curvy::circle& c, const curvy::point& pt, double eps)
{
    if (eps > 0 && is_pt_on_circle(c, pt, eps))
        return true;
    else
        return (curvy::euclidean_distance(c.center(), pt) < c.r);
}

std::tuple<double, double> curvy::closest_pt_on_circle(const curvy::circle& c, const curvy::point& pt)
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