#define NOMINMAX
#include "curvy_vector.h"
#include <Windows.h>

namespace {

    const double k_momentmum_transfer_constant = 0.35;
    const double k_arithmetic_weighting_constant = 0.5;

    bool is_pt_in_front_of_puck(const curvy::point& cannonicalized_pt) {
        const auto pi = curvy::pi();
        auto [dx, dy] = cannonicalized_pt;
        auto angle = std::atan2(dy, dx);
        return (angle > -pi / 2.0 && angle < pi / 2.0);
    }

    double weight_from_magnitude(double v) {
        return std::pow(v, k_arithmetic_weighting_constant);
    }

}


curvy::curvy_vector::curvy_vector(const curvy::circle & c, bool o, double m, double w) :
    orientation_(o), circle_(c), linear_magnitude_(m), weight_( (w >= 0) ? w : weight_from_magnitude(m) )
{ 
    int aaa;
    aaa = 5;
}

void curvy::curvy_vector::set_magnitude(double m)
{
    orientation_ = m > 0;
    linear_magnitude_ = std::abs(m);
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

curvy::circle curvy::curvy_vector::circle() const
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
    auto ir = std::abs((d * d) / (2.0 * d * std::sin(theta)));

    if (r < min_radius) {
        throw std::runtime_error("a circle is too small");
    }  

    auto peak = std::atan(d / std::sqrt(-d * d + 4.0 * r * r));
    if (theta < peak) {
        auto pcnt = (theta - (-curvy::pi_over_two())) / (peak - (-curvy::pi_over_two()));
        auto t2 = -curvy::pi_over_two() + pcnt * (curvy::pi_over_two() - peak);
        ir = std::abs((d * d) / (2.0 * d * std::sin(t2)));
    }
    double val = (ir - min_radius) / std::abs(r - min_radius);

    return std::pow(val, k_momentmum_transfer_constant);
}

double curvy::to_angle_of_curvature(const curvy_vector& cv)
{
    return cv.sign() * pi() / cv.circle().radius();
}

curvy::vec3 to_vector(double weight, double direction, double curvature) {
    auto yaw = direction;
    auto pitch = curvature/2.0;
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

    return { weight, yaw, 2.0 * pitch };
}

curvy::curvy_vector curvy::curvy_vector::add(const curvy_vector& cv, const point& pt) const
{
    if (linear_magnitude() == 0)
        return cv;

    if (cv.linear_magnitude() == 0)
        return *this;

    auto m1 = linear_magnitude();
    auto m2 = cv.linear_magnitude();

    auto curvature1 = curvy::to_angle_of_curvature(*this);
    auto curvature2 = curvy::to_angle_of_curvature(cv);

    auto direction1 = direction_at(pt);
    auto direction2 = cv.direction_at(pt);

    auto weight1 = this->weight();
    auto weight2 = cv.weight();

    auto v1 = to_vector(weight1, direction1, curvature1);
    auto v2 = to_vector(weight2, direction2, curvature2);
    auto vector_sum = v1 + v2;

    auto [weight_of_sum, direction_of_sum, curvature_of_sum] = from_vector(vector_sum);
    auto magnitude_of_sum = m1 + m2;

    auto orientation_of_sum = curvature_of_sum >= 0;
    curvature_of_sum = std::abs(curvature_of_sum);
    matrix from_canonical_coords = translation_matrix(pt) * rotation_matrix(direction_of_sum);
    curvy::circle vector_circle(0, 0, 0);

    if (curvature_of_sum > eps()) {
        auto radius = pi() / curvature_of_sum;
        auto center_y = (orientation_of_sum ? 1.0 : -1.0) * radius;
        vector_circle = curvy::circle(0, center_y, radius);
    } else {
        vector_circle = curvy::circle(true, { 0,0 }, 0);
    }

    auto sum = apply_matrix(
        from_canonical_coords,
        curvy_vector(vector_circle, orientation_of_sum, magnitude_of_sum, weight_of_sum)
    );

    return sum;
}

curvy::curvy_vector curvy::curvy_vector::subtract(const curvy_vector& cv, const point& pt) const
{
    if (cv.linear_magnitude() == 0)
        return *this;

    auto m1 = linear_magnitude();
    auto m2 = cv.linear_magnitude();

    auto curvature1 = curvy::to_angle_of_curvature(*this);
    auto curvature2 = curvy::to_angle_of_curvature(cv);

    auto direction1 = direction_at(pt);
    auto direction2 = cv.direction_at(pt);

    auto weight1 = this->weight();
    auto weight2 = cv.weight();

    auto v1 = to_vector(weight1, direction1, curvature1);
    auto v2 = to_vector(weight2, direction2, curvature2);
    auto vector_diff = v1 - v2;

    auto [weight_of_diff, direction_of_diff, curvature_of_diff] = from_vector(vector_diff);
    auto magnitude_of_diff = m1 - m2;

    auto orientation_of_diff = curvature_of_diff >= 0;
    curvature_of_diff = std::abs(curvature_of_diff);
    matrix from_canonical_coords = translation_matrix(pt) * rotation_matrix(direction_of_diff);
    curvy::circle vector_circle(0, 0, 0);

    if (curvature_of_diff > eps()) {
        auto radius = pi() / curvature_of_diff;
        auto center_y = (orientation_of_diff ? 1.0 : -1.0) * radius;
        vector_circle = curvy::circle(0, center_y, radius);

    }
    else {
        vector_circle = curvy::circle(true, { 0,0 }, 0);
    }

    return apply_matrix(
        from_canonical_coords,
        curvy_vector(vector_circle, orientation_of_diff, magnitude_of_diff, weight_of_diff)
    );
}

/*

double lerp_angles_raw(double theta1, double theta2, double t)
{
    auto delta = theta2 - theta1;
    return curvy::normalize_angle(delta * t + theta1);
}

curvy::curvy_vector curvy::curvy_vector::add(const curvy_vector& cv, const point& pt) const
{
    if (linear_magnitude() == 0)
        return cv;

    if (cv.linear_magnitude() == 0)
        return *this;

    auto m1 = linear_magnitude();
    auto m2 = cv.linear_magnitude();

    auto t = weight(m2) / (weight(m1) + weight(m2));

    auto curvature1 = curvy::to_angle_of_curvature(*this);
    auto curvature2 = curvy::to_angle_of_curvature(cv);
    auto curvature_of_sum = lerp_angles_raw(curvature1, curvature2, t);

    auto direction1 = direction_at(pt);
    auto direction2 = cv.direction_at(pt);
    auto direction_of_sum = lerp_angles_raw(direction1, direction2, t);

    auto orientation_of_sum = curvature_of_sum >= 0;
    curvature_of_sum = std::abs(curvature_of_sum);
    matrix from_canonical_coords = translation_matrix(pt) * rotation_matrix(direction_of_sum);
    curvy::circle vector_circle(0, 0, 0);

    if (curvature_of_sum > eps()) {
        auto radius = pi() / curvature_of_sum;
        auto center_y = (orientation_of_sum ? 1.0 : -1.0) * radius;
        vector_circle = curvy::circle(0, center_y, radius);
        
    } else {
        vector_circle = curvy::circle(true, { 0,0 }, 0);
    }

    return apply_matrix(
        from_canonical_coords,
        curvy_vector(vector_circle, orientation_of_sum, m1+m2)
    );
}

curvy::curvy_vector curvy::curvy_vector::subtract(const curvy_vector& cv, const point& pt) const
{
    if (cv.linear_magnitude() == 0)
        return *this;

    auto m_of_sum = linear_magnitude();
    auto m1 = cv.linear_magnitude();
    auto m2 = m_of_sum - m1;

    if (m2 < 0)
        return curvy_vector({ 0,0,0 }, true, 0);

    auto t = weight(m2) / (weight(m1) + weight(m2));

    auto curvature1 = curvy::to_angle_of_curvature(cv);
    auto curvature_of_sum = curvy::to_angle_of_curvature(*this);
    auto curvature2 = normalize_angle(lerp_angles_inverse(curvature_of_sum, curvature1, t));

    if (std::abs(curvature_of_sum - normalize_angle(lerp_angles(curvature1, curvature2, t))) >= 0.0001) {
        auto raw = lerp_angles_inverse(curvature_of_sum, curvature1, t);
        
    }

    auto direction1 = cv.direction_at(pt);
    auto direction_of_sum = direction_at(pt);
    auto direction2 = normalize_angle(lerp_angles_inverse(direction_of_sum, direction1, t));

    if (std::abs(direction_of_sum - normalize_angle(lerp_angles(direction1, direction2, t))) >= 0.0001) {
        direction2 = lerp_angles_inverse(direction_of_sum, direction1, t);
    }

    auto orientation_of_diff = curvature2 >= 0;
    curvature2 = std::abs(curvature2);
    matrix from_canonical_coords = translation_matrix(pt) * rotation_matrix(direction2);
    curvy::circle vector_circle(0, 0, 0);

    if (curvature2 > eps()) {
        auto radius = pi() / curvature2;

        if (radius < 1.0) {
            return curvy_vector({ 0,0,0 }, true, 0);;
        }

        auto center_y = (orientation_of_diff ? 1.0 : -1.0) * radius;
        vector_circle = curvy::circle(0, center_y, radius);
    } else {
        vector_circle = curvy::circle(true, { 0,0 }, 0);
    }

    return apply_matrix(
        from_canonical_coords,
        curvy_vector(vector_circle, orientation_of_diff, m2)
    );
}

*/

