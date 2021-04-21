#define NOMINMAX
#include "curvy_vector.h"
#include <Windows.h>

namespace {

    bool is_pt_in_front_of_puck(const curvy::point& cannonicalized_pt) {
        const auto pi = curvy::pi();
        auto [dx, dy] = cannonicalized_pt;
        auto angle = std::atan2(dy, dx);
        return (angle > -pi / 2.0 && angle < pi / 2.0);
    }

    double get_curvy_energy(const curvy::curvy_vector& cv) {
        return cv.signed_angular_magnitude() / cv.circle().radius();
    }

    curvy::curvy_vector unpack_curvy_energy(double curvy_energy, double linear_magnitude) {

        auto orientation = (curvy_energy > 0) ? 1.0 : -1.0;
        curvy_energy = std::abs(curvy_energy);

        auto angular_magnitude = std::sqrt( linear_magnitude * curvy_energy);
        auto radius = std::sqrt(linear_magnitude) / std::sqrt(curvy_energy);
        auto center_y = orientation * radius;
        return curvy::curvy_vector(curvy::circle(0, center_y, radius), orientation * angular_magnitude);
    }

    curvy::curvy_vector circular_vector_arithmetic_aux(const curvy::curvy_vector& v1, const curvy::curvy_vector& v2, const curvy::point& where, std::function<double(double,double)> op)
    {
        if (v1.angular_magnitude() == 0)
            return v2;

        if (v2.angular_magnitude() == 0)
            return v1;

        using namespace curvy;
        auto linear_magnitude_of_sum = op(v1.linear_magnitude() , v2.linear_magnitude());
        auto direction_of_sum = atan_of_pt(v1.newtonian_vector_at_point(where) + v2.newtonian_vector_at_point(where));

        matrix from_canonical_coords = translation_matrix(where) * rotation_matrix(direction_of_sum);
        auto curvy_energy = op( get_curvy_energy(v1) , get_curvy_energy(v2));
        auto cv_sum = unpack_curvy_energy(curvy_energy, linear_magnitude_of_sum);

        return apply_matrix(from_canonical_coords, cv_sum);
    }

}

curvy::curvy_vector::curvy_vector(double cx, double cy, double r, double m) : 
    orientation_(m >= 0), circle_(cx, cy, r), angular_magnitude_(std::abs(m))
{}

curvy::curvy_vector::curvy_vector(const curvy::circle & c, double m) :
    orientation_(m >= 0), circle_(c), angular_magnitude_(std::abs(m))
{}

curvy::curvy_vector::curvy_vector(const curvy::circle & c, bool o, double m) :
    orientation_(o), circle_(c), angular_magnitude_(std::abs(m))
{
}

void curvy::curvy_vector::set_magnitude(double m)
{
    orientation_ = m > 0;
    angular_magnitude_ = std::abs(m);
}

void curvy::curvy_vector::set_radius(double r)
{
    circle_.set_radius( r );
}

void curvy::curvy_vector::set_circle(const curvy::circle& c)
{
    circle_ = c;
}

curvy::circle& curvy::curvy_vector::circle()
{
    return circle_;
}

bool curvy::curvy_vector::orientation() const
{
    return orientation_;
}


double curvy::curvy_vector::angular_magnitude() const
{
    return angular_magnitude_;
}

curvy::circle curvy::curvy_vector::circle() const
{
    return circle_;
}

/*
curvy::point curvy::circular_vector::position() const {
    return {
        circle_.x() + circle_.radius() * std::cos(theta_),
        circle_.y() + circle_.radius() * std::sin(theta_)
    };
}

double curvy::circular_vector::direction_angle() const {
    return direction_on_circle(theta_, orientation_);
}
*/

double curvy::curvy_vector::signed_angular_magnitude() const
{
    return sign() * angular_magnitude_;
}

double curvy::curvy_vector::linear_magnitude() const
{
    return angular_magnitude_ * circle_.radius();
}

double curvy::curvy_vector::signed_linear_magnitude() const
{
    return sign() * linear_magnitude();
}

double curvy::curvy_vector::circumference() const
{
    return circle_.circumference();
}

double curvy::curvy_vector::sign() const
{
    return orientation_ ? 1.0 : -1.0;
}

curvy::point curvy::curvy_vector::newtonian_vector_at_point(const point& pt) const
{
    auto angle = direction_at(pt);
    auto magnitude = linear_magnitude();
    auto vec = point{ std::cos(angle), std::sin(angle) };
    return magnitude * vec;
}

curvy::curvy_vector curvy::curvy_vector::add(const curvy_vector& cv, const point& where) const
{
    return circular_vector_arithmetic_aux(*this, cv, where, [](double v1, double v2) {return v1 + v2; });
}

curvy::curvy_vector curvy::curvy_vector::subtract(const curvy_vector& cv, const point& where) const
{
    return circular_vector_arithmetic_aux(*this, cv, where, [](double v1, double v2) {return v1 - v2; });
}

std::string curvy::curvy_vector::to_string() const
{
    return "[ " + circle_.to_string() + " , " + std::to_string(angular_magnitude_) + " ]";
}

double curvy::curvy_vector::direction_at(const point& pt) const
{
    return direction_on_circle(angle_to_pt(circle_.center(), pt), orientation_);
}

curvy::curvy_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.radius();
    return curvy::curvy_vector(circ, angular_magnitude);
}

curvy::curvy_vector curvy::operator*(double scale, const curvy_vector& cv)
{
    return curvy_vector(cv.circle(), scale * cv.signed_angular_magnitude());
}

curvy::curvy_vector curvy::operator*(const curvy_vector& cv, double scale)
{
    return scale * cv;
}

curvy::curvy_vector curvy::apply_matrix(const curvy::matrix& mat, const curvy::curvy_vector& cv)
{
    auto circle = curvy::apply_matrix(mat, cv.circle());
    return curvy::curvy_vector(
        circle,
        cv.sign() * cv.angular_magnitude()
    );
}

bool curvy::is_in_front_of(const curvy::point& pt1, double direction_at_pt1, const curvy::point& pt2) {
    matrix to_canonical_coords = rotation_matrix(-direction_at_pt1) * translation_matrix(-pt1);
    auto [px, py] = curvy::apply_matrix(to_canonical_coords, pt2);
    auto theta = std::atan2(py, px);
    return  theta >= -pi() / 2.0 && theta <= pi() / 2.0;
}

std::tuple<curvy::circle, bool> curvy::circular_direction_through_two_points(const curvy::point& pt1, double direction_at_pt1, const curvy::point& pt2)
{
    matrix to_canonical_coords = rotation_matrix(-direction_at_pt1) * translation_matrix(-pt1);
    matrix from_canonical_coords = translation_matrix(pt1) * rotation_matrix(direction_at_pt1);

    auto [px, py] = apply_matrix(to_canonical_coords, pt2);
    double circle_of_impulse_y = (px * px + py * py) / (2 * py);
    auto c = apply_matrix(from_canonical_coords, curvy::circle(0, circle_of_impulse_y, std::abs(circle_of_impulse_y)));
    return { c, py > 0 };
}

double curvy::momentum_transfer_factor(const curvy::point& pt1, double pt1_direction, bool orientation, const curvy::point& pt2, double r, double d) {

    if (!curvy::is_in_front_of(pt1, pt1_direction, pt2))
        return 0.0;

    auto min_radius = d / 2.0;
    auto theta = (orientation ? 1.0 : -1.0) * curvy::angle_to_point_relative_to_direction(pt1, pt1_direction, pt2);
    auto ir = std::abs((d * d) / (2.0 * d * std::sin(theta)));

    double val;
    if (r < min_radius) {
        val = r / ir;
    } else {

        auto peak = std::atan(d / std::sqrt(-d * d + 4.0 * r * r));
        if (theta < peak) {
            auto pcnt = (theta - (-curvy::pi_over_two())) / (peak - (-curvy::pi_over_two()));
            auto t2 = -curvy::pi_over_two() + pcnt * (curvy::pi_over_two() - peak);
            ir = std::abs((d * d) / (2.0 * d * std::sin(t2)));
        }

        val = (ir - min_radius) / std::abs(r - min_radius);
    }

    return std::sqrt(val);
}
