#include "puck.h"
#include "util.h"
#include "circle.h"
#include "circular_vector.h"
#include <cmath>
#include <string>

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

    curvy::circle get_circle_of_impulse(const curvy::point& cannonicalized_pt) {
        auto [px, py] = cannonicalized_pt;
        double circle_of_impulse_y = (px * px + py * py) / (2 * py);
        return curvy::circle(0, circle_of_impulse_y, std::abs(circle_of_impulse_y));
    }

    double get_momentum_transfer_factor(double radius_of_revolution, const curvy::point& cannonicalized_pt) {
        auto circle_of_inversion = curvy::circle(0, 2 * radius_of_revolution, 2 * radius_of_revolution);
        auto [inverted_x, inverted_y] = circle_of_inversion.invert(cannonicalized_pt);
        auto cosine = inverted_x / std::hypot(inverted_x, inverted_y);
        return cosine * cosine;
    }

    bool is_pt_in_front_of_puck(const curvy::point& cannonicalized_pt) {
        const auto pi = curvy::pi();
        auto [dx, dy] = cannonicalized_pt;
        auto angle = std::atan2(dy, dx);
        return (angle > -pi / 2.0 && angle < pi / 2.0);
    }

    curvy::circular_vector momentum_vec_through_point(const curvy::point& cannonicalized_pt, double radius_of_revolution, double total_linear_momentum)
    {
        auto circle_of_impulse = get_circle_of_impulse(cannonicalized_pt);

        if (!is_pt_in_front_of_puck(cannonicalized_pt)) {
            return curvy::circular_vector(circle_of_impulse, 0, 0);
        }

        auto amount_of_impulse_momentum = get_momentum_transfer_factor(radius_of_revolution, cannonicalized_pt);
        auto sign_of_impulse = (circle_of_impulse.y > 0) ? 1.0 : -1.0;
        auto linear_magnitude = sign_of_impulse * amount_of_impulse_momentum * std::abs(total_linear_momentum);

        return circular_vector_from_linear_magnitude(circle_of_impulse, curvy::get_angle_to_pt(circle_of_impulse.center(), cannonicalized_pt), linear_magnitude);
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

double curvy::puck::puck_radius() const
{
    return puck_radius_;
}

curvy::circular_vector curvy::puck::circle_rot_state() const
{
    return crs_;
}

curvy::circular_vector curvy::puck::momentum_vector() const
{
    return mass_ * crs_;
}

curvy::circular_vector curvy::puck::momentum_vector_through_point(const std::tuple<double, double>& pt)
{
    auto to_canonical_coords = rotation_matrix( -direction() ) * translation_matrix( -position() );
    auto from_canonical_coords = translation_matrix(position()) * rotation_matrix( direction() );
    auto linear_momentum = crs_.linear_magnitude() * mass_;

    return apply_matrix(from_canonical_coords,
        momentum_vec_through_point(
            apply_matrix(to_canonical_coords, pt),
            radius_of_revolution(),
            linear_momentum
        )
    );
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

void curvy::puck::set_puck_radius(double r)
{
    puck_radius_ = r;
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


