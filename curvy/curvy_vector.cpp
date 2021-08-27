#define NOMINMAX
#include "curvy_vector.h"
#include <Windows.h>

namespace {

    const double k_momentmum_transfer_constant = 0.5;
    const double k_arithmetic_weighting_constant = 0.5;
    const double k_curvature_weighting_constant = 1.0;

    double weight_from_magnitude(double v) {
        return std::pow(v, k_arithmetic_weighting_constant);
    }

    double to_angle_of_curvature(const curvy::curvy_vector& cv)
    {
        return curvy::angle_of_curvature(cv.circle(), cv.orientation() );
    }

    double curvature_to_pitch(double curve) {
        auto sgn = curve >= 0 ? 1.0 : -1.0;
        auto v = std::abs(curve) / curvy::pi();
        v = std::pow(v, k_curvature_weighting_constant);

        return sgn * v * curvy::pi_over_two();
    }

    double pitch_to_curvature(double pitch) {
        auto sgn = pitch >= 0 ? 1.0 : -1.0;
        auto v = std::abs(pitch) / curvy::pi_over_two();
        v = std::pow(v, 1.0 / k_curvature_weighting_constant);
        return sgn * v * curvy::pi();
    }

    curvy::vec3 to_vector(double weight, double direction, double curvature) {
        auto yaw = direction;
        auto pitch = curvature_to_pitch(curvature);
        return {
            weight * std::cos(yaw) * std::cos(pitch),
            weight * std::sin(yaw) * std::cos(pitch),
            weight * std::sin(pitch)
        };
    }

    std::tuple<double, double, double> from_vector(const curvy::vec3& v) {
        using namespace curvy;

        auto weight = magnitude(v);
        auto [x, y, z] = (1.0 / weight) * v;
        auto pitch = std::asin(z);
        auto sin_of_yaw = y / std::cos(pitch);
        auto cos_of_yaw = x / std::cos(pitch);
        auto yaw = std::atan2(sin_of_yaw, cos_of_yaw);

        return { weight, yaw, pitch_to_curvature(pitch) };
    }

    curvy::vec3 to_vector(const curvy::curvy_vector& v, const curvy::point& pt) {
        return to_vector(
            v.weight(),
            v.direction_at(pt),
            to_angle_of_curvature(v)
        );
    }

    curvy::curvy_vector from_vector(const curvy::vec3& v, double magnitude, const curvy::point& pt) {
        auto [weight, direction, curvature] = from_vector(v);

        auto orientation_of_sum = curvature >= 0;
        curvature = std::abs(curvature);
        curvy::matrix from_canonical_coords = curvy::translation_matrix(pt) * curvy::rotation_matrix( direction );
        curvy::circle vector_circle(0, 0, 0);

        if (curvature > curvy::eps()) {
            auto radius = curvy::pi() / curvature;
            auto center_y = (orientation_of_sum ? 1.0 : -1.0) * radius;
            vector_circle = curvy::circle(0, center_y, radius);
        }
        else {
            vector_circle = curvy::circle(true, { 0,0 }, 0);
        }

        return curvy::apply_matrix(
            from_canonical_coords,
            curvy::curvy_vector(vector_circle, orientation_of_sum, magnitude, weight)
        );
    }
}


curvy::curvy_vector::curvy_vector(const curvy::circle& c, bool o, double m, double w) :
    orientation_(o), circle_(c), linear_magnitude_(m), weight_((w >= 0) ? w : weight_from_magnitude(m))
{
}

void curvy::curvy_vector::set_magnitude(double m)
{
    orientation_ = m > 0;
    linear_magnitude_ = std::abs(m);
}

void curvy::curvy_vector::set_circle(const curvy::circle& c)
{
    circle_ = c;
}

void curvy::curvy_vector::refresh_weight()
{
    weight_ = weight_from_magnitude(linear_magnitude());
}

bool curvy::curvy_vector::orientation() const
{
    return orientation_;
}


std::optional<double> curvy::curvy_vector::angular_magnitude() const
{
    if (!circle_.is_degenerate() && circle_.radius() > 0) {
        return linear_magnitude_ / circle_.radius();
    }
    if (linear_magnitude_ == 0) {
        return 0.0;
    }
    return std::nullopt;
}

const curvy::circle& curvy::curvy_vector::circle() const
{
    return circle_;
}


std::optional<double> curvy::curvy_vector::signed_angular_magnitude() const
{
    auto omega = angular_magnitude();
    if (!omega)
        return std::nullopt;
    return sign() * omega.value();
}

double curvy::curvy_vector::linear_magnitude() const
{
    return linear_magnitude_;
}

double curvy::curvy_vector::signed_linear_magnitude() const
{
    return sign() * linear_magnitude();
}


double curvy::curvy_vector::sign() const
{
    return orientation_ ? 1.0 : -1.0;
}

std::string curvy::curvy_vector::to_string() const
{
    return "[ " + circle_.to_string() + " , " + std::to_string(linear_magnitude_) + " ]";
}

double curvy::curvy_vector::direction_at(const point& pt) const
{
    return direction_on_circle(angle_to_pt(circle_.center(), pt), orientation_);
}

double curvy::curvy_vector::weight() const
{
    return weight_;
}

curvy::curvy_vector curvy::operator*(double scale, const curvy_vector& cv)
{
    return curvy_vector(cv.circle(), cv.orientation(), scale * cv.signed_linear_magnitude(), cv.weight());
}

curvy::curvy_vector curvy::make_curvy_vector(const curvy::circle& c, bool o, double m)
{
    return curvy_vector(c, o, m, weight_from_magnitude(m));
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
        cv.orientation(),
        cv.linear_magnitude(),
        cv.weight()
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

    if (!curvy::is_in_front_of(pt1, pt1_direction, pt2) || !r)
        return 0.0;

    auto min_radius = d / 2.0;
    auto theta = (orientation ? 1.0 : -1.0) * curvy::angle_to_point_relative_to_direction(pt1, pt1_direction, pt2);
    auto ir = 0.5 * std::abs(d / std::sin(theta));

    if (r < min_radius) {
        throw std::runtime_error("a circle is too small");
    }

    auto peak = std::atan(d / std::sqrt(-d * d + 4.0 * r * r));
    if (theta < peak) {
        auto pcnt = (theta - (-curvy::pi_over_two())) / (peak - (-curvy::pi_over_two()));
        auto t2 = -curvy::pi_over_two() + pcnt * (curvy::pi_over_two() - peak);
        ir = 0.5 * std::abs(d / std::sin(t2));
    }
    double val = (ir - min_radius) / std::abs(r - min_radius);

    return std::pow(val, k_momentmum_transfer_constant);
}

double curvy::momentum_transfer_factor(const curvy::circle& base_circle, bool base_orientation, const curvy::circle& impulse_circle, bool impulse_orientation)
{
    auto base_curvature = curvy::angle_of_curvature(base_circle, base_orientation);
    auto impulse_curvature = curvy::angle_of_curvature(impulse_circle, impulse_orientation);
    double t;

    if (impulse_curvature >= -pi() && impulse_curvature <= base_curvature) {
        t = inverse_lerp(-pi(), base_curvature, impulse_curvature);
    }
    else if (impulse_curvature > base_curvature && impulse_curvature <= pi()) {
        t = inverse_lerp(pi(), base_curvature, impulse_curvature);
    }
    else {
        throw std::runtime_error("this shouldnt happen.");
    }

    return std::pow(t, k_momentmum_transfer_constant);
}

curvy::curvy_vector curvy::curvy_vector::add(const curvy_vector& cv, const point& pt) const
{
    if (linear_magnitude() == 0)
        return cv;

    if (cv.linear_magnitude() == 0)
        return *this;

    auto sum = from_vector(
        to_vector(*this, pt) + to_vector(cv, pt), 
        linear_magnitude() + cv.linear_magnitude(), 
        pt
    );
    sum.refresh_weight();
    return sum;
}

curvy::curvy_vector curvy::curvy_vector::subtract(const curvy_vector& cv, const point& pt) const
{
    if (cv.linear_magnitude() == 0)
        return *this;

    auto diff = from_vector(
        to_vector(*this, pt) - to_vector(cv, pt), 
        linear_magnitude() - cv.linear_magnitude(), 
        pt
    );
    diff.refresh_weight();
    return diff;
}