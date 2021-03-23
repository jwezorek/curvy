#include "circular_vector.h"


std::tuple<double, double> curvy::circular_vector::position() const {
    return {
        circle.x + circle.r * std::cos(theta),
        circle.y + circle.r * std::sin(theta)
    };
}

double curvy::circular_vector::direction_angle() const {
    return direction_angle(theta, angular_magnitude);
}

double curvy::circular_vector::direction_angle(double angle, double s) const {
    auto direction_angle = (s > 0) ?
        angle + 0.5 * pi() :
        angle + 1.5 * pi();
    return normalize_angle(direction_angle);
}

double curvy::circular_vector::linear_magnitude() const
{
    return angular_magnitude * circle.r;
}

double curvy::circular_vector::circumference() const
{
    return circle.circumference();
}

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double theta, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.r;
    return curvy::circular_vector(circ, theta, angular_magnitude);
}

curvy::circular_vector curvy::operator*(double scalar, const curvy::circular_vector& cv)
{
    auto scaled = curvy::circular_vector(cv);
    scaled.angular_magnitude *= scalar;
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
        cv.angular_magnitude
    );
}