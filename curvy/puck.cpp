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

curvy::puck::puck(const circle_rotation_state& crs, gdi::Color color, double puck_radius, double mass) :
    crs_(crs),
    color_(color),
    puck_radius_(puck_radius),
    mass_(mass)
{ }

void curvy::puck::update(double dt) {
    crs_.theta_ = normalize(crs_.theta_ + dt * crs_.speed_, 0, 2 * pi());
}

curvy::puck curvy::puck::update(double dt) const {
    puck clone(*this);
    clone.update(dt);
    return clone;
}

double curvy::puck::theta() const {
    return crs_.theta_;
}

std::tuple<double, double> curvy::puck::center_of_revolution() const {
    return { crs_.cx_, crs_.cy_ };
}

double curvy::puck::radius_of_revolution() const {
    return crs_.r_;
}

double curvy::puck::angular_speed() const
{
    return crs_.speed_;
}

double curvy::puck::radius() const
{
    return puck_radius_;
}

std::tuple<double, double> curvy::puck::position() const
{
    return {
        crs_.cx_ + crs_.r_ * std::cos(crs_.theta_),
        crs_.cy_ + crs_.r_ * std::sin(crs_.theta_)
    };
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

std::optional<double> curvy::puck::get_collision_time(const puck& p, double dt, double eps) const
{
    return ::get_collision_time(*this, p, 0, dt, eps);
}

gdi::Color curvy::puck::color() const
{
    return color_;
}

void curvy::puck::set_speed(double speed)
{
    crs_.speed_ = speed;
}

std::tuple<int, int, int, int> curvy::puck::get_location_in_pixels(float log_sz, int pix_sz) const
{
    auto [cx, cy] = center_of_revolution();
    auto x = cx + radius_of_revolution() * std::cos(theta());
    auto y = cy + radius_of_revolution() * std::sin(theta());

    auto x1 = x - radius();
    auto y1 = y - radius();
    auto x2 = x + radius();
    auto y2 = y + radius();

    return to_scr_coords(x1, y1, x2, y2, log_sz, pix_sz);
}