#include "circular_vector.h"

curvy::circular_vector::circular_vector(double theta, double cx, double cy, double r, double m) : 
    orientation(m >= 0), theta(theta), circle(cx, cy, r), angular_magnitude_(std::abs(m))
{}

curvy::circular_vector::circular_vector(const curvy::circle & c, double t, double m) :
    orientation(m >= 0), circle(c), theta(t), angular_magnitude_(std::abs(m))
{}

void curvy::circular_vector::set_magnitude(double m)
{
    orientation = m > 0;
    angular_magnitude_ = std::abs(m);
}

std::tuple<double, double> curvy::circular_vector::position() const {
    return {
        circle.x + circle.r * std::cos(theta),
        circle.y + circle.r * std::sin(theta)
    };
}

double curvy::circular_vector::direction_angle() const {
    return direction_on_circle(theta, orientation);
}

double curvy::circular_vector::signed_magnitude() const
{
    return sign() * angular_magnitude_;
}

double curvy::circular_vector::linear_magnitude() const
{
    return angular_magnitude_ * circle.r;
}

double curvy::circular_vector::circumference() const
{
    return circle.circumference();
}

double curvy::circular_vector::sign() const
{
    return orientation ? 1.0 : -1.0;
}

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double theta, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.r;
    return curvy::circular_vector(circ, theta, angular_magnitude);
}

curvy::circular_vector curvy::operator*(double scalar, const curvy::circular_vector& cv)
{
    auto scaled = curvy::circular_vector(cv);
    scaled.angular_magnitude_ *= scalar;
    return scaled;
}

curvy::circular_vector curvy::apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& cv)
{
    auto center = curvy::apply_matrix(mat, cv.circle.center());
    auto pt = curvy::apply_matrix(mat, cv.position());
    auto circle = curvy::circle(center, curvy::euclidean_distance(center, pt));
    return curvy::circular_vector(
        circle,
        curvy::get_angle_to_pt(center, pt),
        cv.sign() * cv.angular_magnitude_
    );
}