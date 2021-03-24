#include "circular_vector.h"

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
    circle_.r = r;
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
        circle_.x + circle_.r * std::cos(theta_),
        circle_.y + circle_.r * std::sin(theta_)
    };
}

double curvy::circular_vector::direction_angle() const {
    return direction_on_circle(theta_, orientation_);
}

double curvy::circular_vector::signed_magnitude() const
{
    return sign() * angular_magnitude_;
}

double curvy::circular_vector::linear_magnitude() const
{
    return angular_magnitude_ * circle_.r;
}

double curvy::circular_vector::circumference() const
{
    return circle_.circumference();
}

double curvy::circular_vector::sign() const
{
    return orientation_ ? 1.0 : -1.0;
}

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double theta, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.r;
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