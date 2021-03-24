#include "circle.h"
#include <cmath>



/*-----------------------------------------------------------------------------------------------------------------------------*/

curvy::circle::circle(double cx, double cy, double r) :
    center_(cx,cy), radius_( std::abs(r) )
{
}

curvy::circle::circle(const curvy::point& pt, double r) :
    center_(pt), radius_(std::abs(r))
{
}

void curvy::circle::set_center(const point& pt)
{
    center_ = pt;
}

void curvy::circle::set_radius(double r)
{
    radius_ = r;
}

std::tuple<double, double, double, double> curvy::circle::bounding_box() const {
    auto [x, y] = center_;
    return {
        x - radius_,
        y - radius_,
        x + radius_,
        y + radius_
    };
}

curvy::point curvy::circle::center() const{
    return center_;
}

bool curvy::circle::contains(const curvy::point& pt) const
{
    return curvy::euclidean_distance(center_, pt) <= radius_;
}

bool curvy::circle::perimeter_contains(const curvy::point& pt, double eps) const
{
    auto distance = curvy::euclidean_distance( center_, pt);
    return std::abs(distance - radius_) <= eps;
}

double curvy::circle::circumference() const
{
    return 2.0 * pi() * radius_;
}

curvy::point curvy::circle::invert(const curvy::point& pt) const
{
    auto [cx, cy] = center_;
    auto R = euclidean_distance( center(), pt );
    auto [px, py] = pt;
    auto cosine_theta = (px - cx) / R;
    auto sine_theta = (py - cy) / R;
    auto inverted_dist = radius_ * radius_ / R;
     return {
        cx + inverted_dist * cosine_theta,
        cy + inverted_dist * sine_theta
    };
}

double curvy::circle::diameter() const
{
    return 2.0 * radius_;
}

double curvy::circle::x() const
{
    return std::get<0>(center_);
}

double curvy::circle::y() const
{
    return std::get<1>(center_);
}

double curvy::circle::radius() const
{
    return radius_;
}

/*-----------------------------------------------------------------------------------------------------------------------------*/

curvy::circle curvy::apply_matrix(const curvy::matrix& mat, const curvy::circle& c)
{
    return curvy::circle(
        curvy::apply_matrix(mat, c.center()),
        c.radius()
    );
}


/*
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
*/