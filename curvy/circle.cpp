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
    return center_ + point(radius_ * std::cos(theta), radius_* std::sin(theta));
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

std::string curvy::circle::to_string() const
{
    return "{ " + curvy::to_string(center_) + " , " + std::to_string(radius_) + " }";
}



/*-----------------------------------------------------------------------------------------------------------------------------*/

std::tuple<std::tuple<curvy::point, curvy::point>, std::tuple<curvy::point, curvy::point>> curvy::mutual_tangents(const curvy::circle& c1, const curvy::circle& c2)
{
    // https://en.wikipedia.org/wiki/Tangent_lines_to_circles#Analytic_geometry

    auto [x1, y1] = c1.center();
    auto [x2, y2] = c2.center();
    auto r = c1.radius();
    auto R = c2.radius();
    auto gamma = -std::atan2((y2 - y1) , (x2 - x1));
    auto beta = -std::asin((R - r) / std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)));
    auto alpha1 = gamma + beta;
    auto alpha2 = gamma - beta;

    return {
        {{x1 + r * std::sin(alpha1), y1 + r * std::cos(alpha1) }, {x2 + R * std::sin(alpha1), y2 + R * std::cos(alpha1)}},
        {{x1 - r * std::sin(alpha2), y1 - r * std::cos(alpha2) }, {x2 - R * std::sin(alpha2), y2 - R * std::cos(alpha2)}}
    };
}

std::optional<std::tuple<curvy::point, curvy::point>> curvy::intersections(const circle& c1, const circle& c2)
{
    auto [x1, y1] = c1.center();
    auto [x2, y2] = c2.center();
    auto r1 = c1.radius();
    auto r2 = c2.radius();

    auto centerdx = x1 - x2;
    auto centerdy = y1 - y2;
    auto R = std::sqrt(centerdx * centerdx + centerdy * centerdy);

    if (!(std::abs(r1 - r2) <= R && R <= r1 + r2)) { // no intersection
        return std::nullopt;
    }
    // intersection(s) should exist

    auto R2 = R * R;
    auto R4 = R2 * R2;
    auto a = (r1 * r1 - r2 * r2) / (2 * R2);
    auto r2r2 = (r1 * r1 - r2 * r2);
    auto c = std::sqrt(2 * (r1 * r1 + r2 * r2) / R2 - (r2r2 * r2r2) / R4 - 1);

    auto fx = (x1 + x2) / 2 + a * (x2 - x1);
    auto gx = c * (y2 - y1) / 2;
    auto ix1 = fx + gx;
    auto ix2 = fx - gx;

    auto fy = (y1 + y2) / 2 + a * (y2 - y1);
    auto gy = c * (x1 - x2) / 2;
    auto iy1 = fy + gy;
    auto iy2 = fy - gy;

    // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
    // but that one solution will just be duplicated as the code is currently written
    return { { {ix1, iy1},{ix2, iy2} } };
}

curvy::circle curvy::apply_matrix(const curvy::matrix& mat, const curvy::circle& c)
{
    auto east = apply_matrix(mat, c.get_point(0));
    auto north = apply_matrix(mat, c.get_point(pi_over_two()));
    auto west = apply_matrix(mat, c.get_point(pi()));
    return *circle_through_three_points(east, north, west);
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