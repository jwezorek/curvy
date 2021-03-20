#include "puck.h"
#include "util.h"
#include <cmath>

namespace {
    std::optional<double> get_collision_time(const curvy::puck& p1, const curvy::puck& p2, double t1, double t2, double eps) {
        auto p1_at_t2 = p1.update(t2);
        auto p2_at_t2 = p2.update(t2);

        auto distance_from_touching = p1_at_t2.distance_from_intersection(p2_at_t2);
        if (distance_from_touching > 0)
            return std::nullopt;

        if (std::abs(distance_from_touching) <= eps)
            return t2;

        auto half_time = (t1 + t2) / 2.0;
        bool intersects_at_halftime = p1.update(half_time).intersects(p2.update(half_time));
        if (intersects_at_halftime)
            return get_collision_time(p1, p2, t1, half_time, eps);
        else
            return get_collision_time(p1, p2, half_time, t2, eps);
    }
}



/*-----------------------------------------------------------------------------------------------------------------------------*/

curvy::puck::puck(const circular_vector& crs, gdi::Color color, double puck_radius, double mass) :
    crs_(crs),
    color_(color),
    puck_radius_(puck_radius),
    mass_(mass)
{ }

void curvy::puck::update(double dt) {
    crs_.theta = normalize_angle(crs_.theta + dt * crs_.angular_magnitude);
}

curvy::puck curvy::puck::update(double dt) const {
    puck clone(*this);
    clone.update(dt);
    return clone;
}

double curvy::puck::theta() const {
    return crs_.theta;
}

std::tuple<double, double> curvy::puck::center_of_revolution() const {
    return { crs_.circle.x, crs_.circle.y };
}

curvy::circle curvy::puck::circle_of_revolution() const
{
    return crs_.circle;
}

double curvy::puck::radius_of_revolution() const {
    return crs_.circle.r;
}

double curvy::puck::angular_speed() const
{
    return crs_.angular_magnitude;
}

double curvy::puck::radius() const
{
    return puck_radius_;
}

std::tuple<double, double> curvy::puck::position() const
{
    return crs_.position();
}

bool curvy::puck::contains_point(double x, double y) const
{
    return contains_point( { x,y } );
}

bool curvy::puck::contains_point(const std::tuple<double, double>& pt) const
{
    return euclidean_distance(position(), pt) <= puck_radius_;
}

bool curvy::puck::intersects(const puck& p) const
{
    return distance_from_intersection(p) <= 0;
}

double curvy::puck::distance_from_center(const puck& p) const
{
    return euclidean_distance(position(), p.position());
}

double curvy::puck::distance_from_intersection(const puck& p) const
{
    auto dist = distance_from_center(p);
    auto distance_when_touching = puck_radius_ + p.radius();
    return dist - distance_when_touching;
}

double curvy::puck::direction() const
{
    return crs_.direction_angle();
}

curvy::circular_vector curvy::puck::circle_rot_state() const
{
    return crs_;
}

std::tuple<curvy::circular_vector, curvy::circular_vector> curvy::puck::momentum_vector_through_point(const point& pt)
{
    //TODO:
    return std::tuple<circular_vector, circular_vector>();
}

std::optional<double> curvy::puck::get_collision_time(const puck& p, double dt, double eps) const
{
    return ::get_collision_time(*this, p, 0, dt, eps);
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
    crs_.theta = theta;
}

void curvy::puck::set_speed(double speed)
{
    crs_.angular_magnitude = speed;
}

void curvy::puck::set_radius_of_revolution(double r)
{
    crs_.circle.r = r;
}

void curvy::puck::set_circle_rotation_position(double theta, double cx, double cy, double r)
{
    crs_.theta = theta;
    crs_.circle.x = cx;
    crs_.circle.y = cy;
    crs_.circle.r = r;
}

void curvy::puck::set_center_of_revolution(const std::tuple<double, double>& pt)
{
    crs_.circle.x = std::get<0>(pt);
    crs_.circle.y = std::get<1>(pt);
}

curvy::circle curvy::puck::get_puck_circle() const
{
    return circle(crs_.position(), puck_radius_);
}

std::tuple<int, int, int, int> curvy::puck::get_bounding_box_in_pixels(double log_sz, int pix_sz) const {
    return to_scr_coords(
        get_puck_circle().bounding_box(),
        log_sz, pix_sz
    );
}

/*
std::tuple<int, int, int, int> curvy::puck::get_bounding_box_in_pixels(double log_sz, int pix_sz) const {
    auto [x, y] = crs_.position();
    return to_scr_coords(
        x - puck_radius_,  y - puck_radius_,
        x + puck_radius_, y + puck_radius_, 
        log_sz, pix_sz
    );
}
*/

void curvy::puck::paint(gdi::Graphics& g, double log_sz, int pix_sz) const
{
    gdi::SolidBrush brush( color_ );
    auto [x1, y1, x2, y2] = get_bounding_box_in_pixels(log_sz, pix_sz);
    g.FillEllipse(&brush, x1, y1, x2 - x1, y2 - y1);
}


