#include "post.h"
#include "util.h"
#include "curvy_vector.h"

curvy::post::post(const circle& c, gdi::Color color) : circle_(c), color_(color)
{
}

curvy::point curvy::post::position() const
{
    return circle_.center();
}

const curvy::circle& curvy::post::post_circle() const
{
    return circle_;
}

std::optional<double> curvy::post::get_collision_time(const puck& p, double dt, double eps) const
{
    if (!p.state().angular_magnitude())
        return std::nullopt; // TODO: handle degenerate circles

    if (! is_in_front_of(p.position(), p.direction(), circle_.center()))
        return std::nullopt;

    const auto& v = p.state();
    auto d = p.puck_circle().radius() + circle_.radius();
    return circle_traveling_in_circle_collision_time_with_stationary_circle(
        v.circle().radius(), 
        v.circle().x(), v.circle().y(), 
        *v.signed_angular_magnitude(), 
        p.theta(), 
        circle_.x(), circle_.y(), 
        d, 
        dt
    );
}

void curvy::post::paint(gdi::Graphics& g, double log_sz, int pix_sz) const
{
    gdi::SolidBrush brush(color_);
    auto [x1, y1, x2, y2] = bounding_box_in_pixels(circle_, log_sz, pix_sz);
    g.FillEllipse(&brush, x1, y1, x2 - x1, y2 - y1);
}
