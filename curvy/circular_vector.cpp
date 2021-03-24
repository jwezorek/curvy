#include "circular_vector.h"

namespace {
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
        auto sign_of_impulse = (circle_of_impulse.y() > 0) ? 1.0 : -1.0;
        auto linear_magnitude = sign_of_impulse * amount_of_impulse_momentum * total_linear_momentum;

        return circular_vector_from_linear_magnitude(circle_of_impulse, curvy::get_angle_to_pt(circle_of_impulse.center(), cannonicalized_pt), linear_magnitude);
    }
}

curvy::circular_vector::circular_vector(double theta, double cx, double cy, double r, double m) : 
    orientation_(m >= 0), theta_(theta), circle_(cx, cy, r), angular_magnitude_(std::abs(m))
{}

curvy::circular_vector::circular_vector(const curvy::circle & c, double t, double m) :
    orientation_(m >= 0), circle_(c), theta_(t), angular_magnitude_(std::abs(m))
{}

void curvy::circular_vector::set_magnitude(double m)
{
    orientation_ = m > 0;
    angular_magnitude_ = std::abs(m);
}

void curvy::circular_vector::set_theta(double theta)
{
    theta_ = theta;
    normalize_angle(theta_);
}

void curvy::circular_vector::increment_theta(double amt)
{
    theta_ += amt;
    normalize_angle(theta_);
}

void curvy::circular_vector::set_radius(double r)
{
    circle_.set_radius( r );
}

void curvy::circular_vector::set_circle(const curvy::circle& c)
{
    circle_ = c;
}

curvy::circle& curvy::circular_vector::circle()
{
    return circle_;
}

bool curvy::circular_vector::orientation() const
{
    return orientation_;
}

double curvy::circular_vector::theta() const
{
    return theta_;
}

double curvy::circular_vector::angular_magnitude() const
{
    return angular_magnitude_;
}

curvy::circle curvy::circular_vector::circle() const
{
    return circle_;
}

curvy::point curvy::circular_vector::position() const {
    return {
        circle_.x() + circle_.radius() * std::cos(theta_),
        circle_.y() + circle_.radius() * std::sin(theta_)
    };
}

double curvy::circular_vector::direction_angle() const {
    return direction_on_circle(theta_, orientation_);
}

double curvy::circular_vector::signed_angular_magnitude() const
{
    return sign() * angular_magnitude_;
}

double curvy::circular_vector::linear_magnitude() const
{
    return angular_magnitude_ * circle_.radius();
}

double curvy::circular_vector::circumference() const
{
    return circle_.circumference();
}

double curvy::circular_vector::sign() const
{
    return orientation_ ? 1.0 : -1.0;
}

std::tuple<curvy::circular_vector, curvy::circular_vector> curvy::circular_vector::split_into_components(const point& pt) const
{
    auto to_canonical_coords = rotation_matrix(-direction_angle()) * translation_matrix(-position());
    auto from_canonical_coords = translation_matrix(position()) * rotation_matrix(direction_angle());
    auto linear_momentum = linear_magnitude();

    auto impulse_vector = apply_matrix(from_canonical_coords,
        momentum_vec_through_point(
            apply_matrix(to_canonical_coords, pt),
            circle().radius(),
            linear_momentum
        )
    );

    return { impulse_vector, curvy::circular_vector() };
}

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double theta, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.radius();
    return curvy::circular_vector(circ, theta, angular_magnitude);
}

curvy::circular_vector curvy::operator*(double scalar, const curvy::circular_vector& cv)
{
    auto scaled = curvy::circular_vector(cv);
    scaled.set_magnitude( scaled.angular_magnitude() * scalar );
    return scaled;
}

curvy::circular_vector curvy::apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& cv)
{
    auto center = curvy::apply_matrix(mat, cv.circle().center());
    auto pt = curvy::apply_matrix(mat, cv.position());
    auto circle = curvy::circle(center, curvy::euclidean_distance(center, pt));
    return curvy::circular_vector(
        circle,
        curvy::get_angle_to_pt(center, pt),
        cv.sign() * cv.angular_magnitude()
    );
}