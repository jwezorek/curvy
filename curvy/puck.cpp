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

        if (!is_intersecting(p1_at_t2, p2_at_t2))
            return std::nullopt;

        if (is_in_contact(p1_at_t2, p2_at_t2, eps))
            return t2;

        auto v1 = p1.state();
        auto v2 = p2.state();

        if (v1.angular_magnitude() && v2.angular_magnitude()) {
            return curvy::circles_traveling_in_circles_collision_time(
                v1.circle().radius(),
                p1.theta(),
                *v1.signed_angular_magnitude(),
                v2.circle().x() - v1.circle().x(),
                v2.circle().y() - v1.circle().y(),
                v2.circle().radius(),
                p2.theta(),
                *v2.signed_angular_magnitude(),
                p1.puck_circle().radius() + p1.puck_circle().radius(),
                t2
            );
        }

        throw std::runtime_error("TODO: handle collisions on degenerate circles");

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

        if (p1.state().angular_magnitude()) {
            return curvy::circle_traveling_in_circle_collision_time_with_circular_border(
                border.radius(),
                p1.state().circle().radius(),
                p1.state().circle().x(),
                p1.state().circle().y(),
                *p1.state().signed_angular_magnitude(),
                p1.theta(),
                p1.puck_circle().radius(),
                t2
            );
        } else {
            throw std::runtime_error("TODO: handle border collisions on degenerate circles");
        }

        return std::nullopt;
    }

}

/*-----------------------------------------------------------------------------------------------------------------------------*/

curvy::puck::puck(const curvy_vector& crs, double theta, gdi::Color color, double puck_radius, double mass) :
    curvy_vector_(crs),
    theta_(theta),
    color_(color),
    puck_radius_(puck_radius),
    mass_(mass)
{ }

curvy::puck::puck(double x, double y, gdi::Color color, double puck_radius, double mass) :
    curvy_vector_(make_curvy_vector(circle(x, y, 0), true, 0)), theta_(0), color_(color),  mass_(mass), puck_radius_(puck_radius)
{
}

void curvy::puck::update(double dt) {
    auto omega = curvy_vector_.signed_angular_magnitude();
    if (!omega)
        return;
    theta_ += dt * omega.value();
    theta_ = normalize_angle(theta_);
}

curvy::puck curvy::puck::update(double dt) const {
    puck clone(*this);
    clone.update(dt);
    return clone;
}

curvy::curvy_vector curvy::puck::state() const
{
    return curvy_vector_;
}

curvy::curvy_vector& curvy::puck::state()
{
    return curvy_vector_;
}

curvy::curvy_vector curvy::puck::momentum_vector() const
{
    return mass_ * curvy_vector_;
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
    curvy_vector_.set_magnitude(speed);
}

void curvy::puck::set_puck_radius(double r)
{
    puck_radius_ = r;
}

void curvy::puck::set_radius_of_revolution(double r)
{
    curvy_vector_.set_radius(r);
}

void curvy::puck::set_circle_rotation_position(double theta, double cx, double cy, double r)
{
    set_theta( theta );
    curvy_vector_.set_circle(curvy::circle(cx, cy, r));
}

void curvy::puck::set_center_of_revolution(const point& pt)
{
    curvy_vector_.circle().set_center(pt);
}

void curvy::puck::set_position(const point& pt)
{
    auto offset = curvy_vector_.circle().center() - position();
    curvy_vector_.circle().set_center(pt + offset);
}

void curvy::puck::set_vector(const curvy_vector& v)
{
    auto pt = position();
    curvy_vector_ = v;
    theta_ = angle_to_pt(v.circle().center(), pt);
}

curvy::point curvy::puck::position() const
{
    return curvy_vector_.circle().get_point(theta_);
}

double curvy::puck::theta() const
{
    return theta_;
}

double curvy::puck::direction() const
{
    return normalize_angle( curvy_vector_.direction_at(position()) );
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

void curvy::puck::apply_friction(double dt)
{
    auto mag = state().linear_magnitude();
    if (mag == 0)
        return;

    auto friction = 0.00015 * mag * mag;
    if (friction < 0.01)
        friction = 0.01;

    mag -= friction;
    if (mag < 0)
        mag = 0;
    state().set_magnitude( state().sign() * mag);
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
