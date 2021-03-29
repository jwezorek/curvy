#include "circle.h"
#include <cmath>
#include <complex>
#include <optional>

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

curvy::point curvy::circle::get_point(double theta) const
{
    return center_ + point(radius_ *std::cos(theta), radius_*std::sin(theta));
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

curvy::circle curvy::circle::invert(const circle& c) const
{
    auto east = invert(c.get_point(0));
    auto north = invert(c.get_point(pi_over_two()));
    auto west = invert(c.get_point(pi()));
    return *circle_through_three_points(east, north, west);
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



curvy::point curvy::closest_pt_on_circle(const curvy::circle& c, const curvy::point& pt) {
    // https://math.stackexchange.com/a/127615/63016

    auto [cx, cy] = c.center();
    auto [px, py] = pt;
    auto distance_to_center = curvy::euclidean_distance(cx, cy, px, py);
    return {
        cx + c.radius() * (px - cx) / distance_to_center,
        cy + c.radius() * (py - cy) / distance_to_center
    };
}

std::complex<double> pt_to_z(const curvy::point& pt) {
    auto [x, y] = pt;
    return { x,y };
}

std::optional<curvy::circle> curvy::circle_through_three_points(const point& pt1, const point& pt2, const point& pt3) {

    using namespace std::complex_literals;
    auto z1 = pt_to_z(pt1), z2 = pt_to_z(pt2), z3 = pt_to_z(pt3);
    auto w = (z3 - z1) / (z2 - z1);

    if (w.imag() == 0)
        return std::nullopt; // the three points are colinear

    auto magnitude_w = std::abs(w);
    auto c = (z2 - z1) * (w - magnitude_w * magnitude_w) / (2i * w.imag()) + z1;
    auto r = std::abs(z1 - c);

    return curvy::circle(c.real(), c.imag(), r);
}