#include "puck.h"
#include "util.h"
#include "circle.h"
#include "curvy_vector.h"
#include <cmath>
#include <string>

namespace {

    std::tuple<int, int, int, int> bounding_box_in_pixels(const curvy::puck& p, double log_sz, int pix_sz)  {
        return curvy::to_scr_coords(
            p.puck_circle().bounding_box(),
            log_sz, pix_sz
        );
    }

    double distance_from_intersection(const curvy::puck& p1, const curvy::puck& p2)
    {
        auto dist = curvy::euclidean_distance(p1.position(), p2.position());
        auto distance_when_touching = p1.puck_circle().radius() + p2.puck_circle().radius();
        return dist - distance_when_touching;
    }

    bool is_intersecting(const curvy::puck& p1, const curvy::puck& p2)
    {
        return distance_from_intersection(p1, p2) <= 0;
    }

    bool is_in_contact(const curvy::puck& p1, const curvy::puck& p2, double eps) {
        return std::abs(distance_from_intersection(p1, p2)) <= eps;
    }

    std::optional<double> get_collision_time(const curvy::puck& p1, const curvy::puck& p2, double t1, double t2, double eps) {
        auto p1_at_t2 = p1.update(t2);
        auto p2_at_t2 = p2.update(t2);

        if (!is_intersecting(p1_at_t2 , p2_at_t2))
            return std::nullopt;

        if (is_in_contact(p1_at_t2, p2_at_t2, eps))
            return t2;

        auto half_time = (t1 + t2) / 2.0;
        bool intersects_at_halftime = is_intersecting(p1.update(half_time), p2.update(half_time));
        if (intersects_at_halftime)
            return get_collision_time(p1, p2, t1, half_time, eps);
        else
            return get_collision_time(p1, p2, half_time, t2, eps);
    }

    double distance_from_intersection_with_boundary(const curvy::puck& p1, const curvy::circle& border)
    {
        auto pt_on_boundary = curvy::closest_pt_on_circle(border, p1.position());
        auto dist = curvy::euclidean_distance(p1.position(), pt_on_boundary);
        auto distance_when_touching = p1.puck_circle().radius();
        return dist - distance_when_touching;
    }

    bool is_intersecting_with_border(const curvy::puck& p1, const curvy::circle& border)
    {
        return distance_from_intersection_with_boundary(p1, border) <= 0;
    }

    bool is_in_contact_with_border(const curvy::puck& p1, const curvy::circle& border, double eps) {
        return std::abs(distance_from_intersection_with_boundary(p1, border)) <= eps;
    }

    std::optional<double> get_boundary_collision_time(const curvy::puck& p1, const curvy::circle& border, double t1, double t2, double eps) {
        auto p1_at_t2 = p1.update(t2);

        if (!is_intersecting_with_border(p1_at_t2, border))
            return std::nullopt;

        if (is_in_contact_with_border(p1_at_t2, border, eps))
            return t2;

        auto half_time = (t1 + t2) / 2.0;
        bool intersects_at_halftime = is_intersecting_with_border(p1.update(half_time), border);
        if (intersects_at_halftime)
            return get_boundary_collision_time(p1, border, t1, half_time, eps);
        else
            return get_boundary_collision_time(p1, border, half_time, t2, eps);
    }

}

/*-----------------------------------------------------------------------------------------------------------------------------*/

curvy::puck::puck(const curvy_vector& crs, double theta, gdi::Color color, double puck_radius, double mass) :
    state_(crs),
    theta_(theta),
    color_(color),
    puck_radius_(puck_radius),
    mass_(mass)
{ }

void curvy::puck::update(double dt) {
    theta_ += dt * state_.signed_angular_magnitude();
    theta_ = normalize_angle(theta_);
}

curvy::puck curvy::puck::update(double dt) const {
    puck clone(*this);
    clone.update(dt);
    return clone;
}

curvy::curvy_vector curvy::puck::state() const
{
    return state_;
}

curvy::curvy_vector& curvy::puck::state()
{
    return state_;
}

curvy::curvy_vector curvy::puck::momentum_vector() const
{
    return mass_ * state_;
}

std::optional<double> curvy::puck::get_collision_time(const puck& p, double dt, double eps) const
{
    return ::get_collision_time(*this, p, 0, dt, eps);
}

std::optional<double> curvy::puck::get_boundary_collision_time(const curvy::circle& border, double dt, double eps) const
{
    return ::get_boundary_collision_time(*this, border, 0, dt, eps);
}

gdi::Color curvy::puck::color() const
{
    return color_;
}

void curvy::puck::set_color(gdi::Color color)
{
    color_ = color;
}

void curvy::puck::set_theta(double theta)
{
    theta_ = normalize_angle(theta);
}

void curvy::puck::set_speed(double speed)
{
    state_.set_magnitude(speed);
}

void curvy::puck::set_puck_radius(double r)
{
    puck_radius_ = r;
}

void curvy::puck::set_radius_of_revolution(double r)
{
    state_.set_radius(r);
}

void curvy::puck::set_circle_rotation_position(double theta, double cx, double cy, double r)
{
    set_theta( theta );
    state_.set_circle(curvy::circle(cx, cy, r));
}

void curvy::puck::set_center_of_revolution(const point& pt)
{
    state_.circle().set_center(pt);
}

void curvy::puck::set_position(const point& pt)
{
    auto offset = state_.circle().center() - position();
    state_.circle().set_center(pt + offset);
}

void curvy::puck::set_vector(const curvy_vector& v)
{
    auto pt = position();
    state_ = v;
    theta_ = angle_to_pt(v.circle().center(), pt);
}

curvy::point curvy::puck::position() const
{
    return state_.circle().get_point(theta_);
}

double curvy::puck::theta() const
{
    return theta_;
}

double curvy::puck::direction() const
{
    return normalize_angle( state_.direction_at(position()) );
}

curvy::circle curvy::puck::puck_circle() const
{
    return circle( position(), puck_radius_);
}

void curvy::puck::paint(gdi::Graphics& g, double log_sz, int pix_sz) const
{
    gdi::SolidBrush brush( color_ );
    auto [x1, y1, x2, y2] = bounding_box_in_pixels(*this, log_sz, pix_sz);
    g.FillEllipse(&brush, x1, y1, x2 - x1, y2 - y1);
}

void curvy::puck::add_to_contact_list(const puck& p)
{
    contact_list_.insert(&p);
}

bool curvy::puck::is_in_contact_list(const puck& p) const
{
    return contact_list_.find(&p) != contact_list_.end();
}

void curvy::puck::update_contact_list()
{
    std::unordered_set<const puck*> contact_list;
    std::copy_if(contact_list_.begin(), contact_list_.end(), std::inserter(contact_list, contact_list.end()),
        [&](const puck* p) { return is_in_contact_or_intersecting(*this, *p); }
    );
    contact_list_ = contact_list;
}

bool curvy::is_in_contact_or_intersecting(const puck& p1, const puck& p2)
{
    return is_in_contact(p1, p2, eps()) || is_intersecting(p1, p2);
}
