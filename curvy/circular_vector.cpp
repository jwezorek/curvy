#define NOMINMAX
#include "circular_vector.h"
#include <Windows.h>

namespace {
    curvy::circle get_circle_of_impulse(const curvy::point& cannonicalized_pt) {
        auto [px, py] = cannonicalized_pt;
        double circle_of_impulse_y = (px * px + py * py) / (2 * py);
        return curvy::circle(0, circle_of_impulse_y, std::abs(circle_of_impulse_y));
    }

    /*
    double get_momentum_transfer_factor(double theta, const curvy::circle& c1, const curvy::circle& c2, double puck_sz) {
        auto d = puck_sz;
        auto min_radius = d / 2.0;
        auto numerator = std::min(c1.radius(), c2.radius());
        auto denominator = std::max(c1.radius(), c2.radius());

        return (numerator - min_radius) / (denominator - min_radius);
    }
    */
    
    double get_momentum_transfer_factor(double theta, const curvy::circle& c1, const curvy::circle& c2, double puck_sz) {
        auto d = puck_sz;
        auto min_radius = d / 2.0;
        auto r = c1.radius();
        auto peak = std::atan(d / std::sqrt(-d * d + 4.0 * r * r));
        auto imposed_radius = c2.radius();

        if (theta < peak) {
            auto pcnt = (theta - (-curvy::pi_over_two())) / (peak - (-curvy::pi_over_two()));
            auto t2 = -curvy::pi_over_two() + pcnt * (curvy::pi_over_two() - peak);
            imposed_radius = std::abs(((d * std::cos(t2)) * (d * std::cos(t2)) + (d * std::sin(t2)) * (d * std::sin(t2))) / (2.0 * d * std::sin(t2)));
        }
        auto val = (imposed_radius - min_radius) / (r - min_radius);
        return val;
    }

    bool is_pt_in_front_of_puck(const curvy::point& cannonicalized_pt) {
        const auto pi = curvy::pi();
        auto [dx, dy] = cannonicalized_pt;
        auto angle = std::atan2(dy, dx);
        return (angle > -pi / 2.0 && angle < pi / 2.0);
    }

    /*
    // angular momentum
    std::tuple<curvy::circular_vector, curvy::circular_vector> split_canonicalized_vector_into_components(const curvy::circular_vector& cv, const curvy::point& cannonicalized_pt, double min_radius)
    {
        auto src_circle = cv.circle();
        auto circle_of_impulse = get_circle_of_impulse(cannonicalized_pt);

        if (!is_pt_in_front_of_puck(cannonicalized_pt)) {
            return { curvy::circular_vector(circle_of_impulse, 0, 0), {} };
        }

        auto transfer_coefficient = get_momentum_transfer_factor(curvy::atan_of_pt(cannonicalized_pt), src_circle, circle_of_impulse, min_radius);
        auto sign_of_impulse = (circle_of_impulse.y() > 0) ? 1.0 : -1.0;
        auto impulse_signed_linear_magnitude = sign_of_impulse * transfer_coefficient * cv.linear_magnitude();
        auto impulse_linear_magnitude = std::abs(impulse_signed_linear_magnitude);

        auto impulse_vector = circular_vector_from_linear_magnitude(circle_of_impulse, curvy::get_angle_to_pt(circle_of_impulse.center(), cannonicalized_pt), impulse_signed_linear_magnitude);

        auto angular_momentum_total = src_circle.radius() * cv.signed_linear_magnitude();
        auto angular_momentum_impulse = impulse_vector.circle().radius() * impulse_vector.signed_linear_magnitude();
        auto linear_magnitude = cv.linear_magnitude();
        auto residual_linear_momentum = linear_magnitude - impulse_linear_magnitude;
        auto residual_angular_momentum = angular_momentum_total - angular_momentum_impulse;
        auto orientation_of_residual = residual_angular_momentum > 0;
        residual_angular_momentum = std::abs(residual_angular_momentum);
        auto residual_angular_magnitude = (residual_linear_momentum * residual_linear_momentum) / residual_angular_momentum;
        auto residual_radius = residual_angular_momentum / residual_linear_momentum;
        auto residual_center_y = (orientation_of_residual) ? residual_radius : -residual_radius;
        auto residual_theta = (orientation_of_residual) ? (3 * curvy::pi()/2.0) : curvy::pi() / 2.0;
        auto residual_vector = curvy::circular_vector(curvy::circle(0, residual_center_y, residual_radius), residual_theta, orientation_of_residual, residual_angular_magnitude);

        return { impulse_vector, residual_vector };
    }
    */

    /*
    // circle inversion + linear momentum...
    std::tuple<curvy::circular_vector, curvy::circular_vector> split_canonicalized_vector_into_components(const curvy::circular_vector& cv, const curvy::point& cannonicalized_pt, double min_radius)
    {
        auto src_circle = cv.circle();
        auto circle_of_impulse = get_circle_of_impulse(cannonicalized_pt);
        auto residual_circle = src_circle.invert(circle_of_impulse);

        if (!is_pt_in_front_of_puck(cannonicalized_pt)) {
            return { curvy::circular_vector(circle_of_impulse, 0, 0), {} };
        }

        auto total_angular_momentum =  src_circle.radius() * cv.angular_magnitude();

        auto transfer_coefficient = get_momentum_transfer_factor( curvy::atan_of_pt(cannonicalized_pt), src_circle, circle_of_impulse, min_radius);

        auto sign_of_impulse = (circle_of_impulse.y() > 0) ? 1.0 : -1.0;
        auto angular_momentum_of_impulse = transfer_coefficient * total_angular_momentum;
        auto angular_speed_of_impulse = sign_of_impulse * angular_momentum_of_impulse / (circle_of_impulse.radius());

        auto residual_angular_momentum = total_angular_momentum - angular_momentum_of_impulse;
        auto sign_of_residual = (residual_circle.y() > 0) ? 1.0 : -1.0;
        auto residual_angular_speed = sign_of_residual * residual_angular_momentum / (residual_circle.radius());

        auto residual_center_y = (sign_of_residual > 0) ? residual_circle.radius() : -residual_circle.radius();
        auto residual_theta = (sign_of_residual > 0) ? (3 * curvy::pi() / 2.0) : curvy::pi() / 2.0;

        return {
            curvy::circular_vector(circle_of_impulse, curvy::get_angle_to_pt(circle_of_impulse.center(), cannonicalized_pt), angular_speed_of_impulse),
            curvy::circular_vector(residual_circle, residual_theta, residual_angular_speed)
        };
    }
    */

    /*
    // centripetal_force
    std::tuple<curvy::circular_vector, curvy::circular_vector> split_canonicalized_vector_into_components(const curvy::circular_vector& cv, const curvy::point& cannonicalized_pt, double min_radius)
    {
        auto src_circle = cv.circle();
        auto circle_of_impulse = get_circle_of_impulse(cannonicalized_pt);

        if (!is_pt_in_front_of_puck(cannonicalized_pt)) {
            return { curvy::circular_vector(circle_of_impulse, 0, 0), {} };
        }

        auto transfer_coefficient = get_momentum_transfer_factor(curvy::atan_of_pt(cannonicalized_pt), src_circle, circle_of_impulse, min_radius);
        auto sign_of_impulse = (circle_of_impulse.y() > 0) ? 1.0 : -1.0;
        auto impulse_linear_magnitude = transfer_coefficient * cv.linear_magnitude();

        auto impulse_vector = circular_vector_from_linear_magnitude(circle_of_impulse, curvy::get_angle_to_pt(circle_of_impulse.center(), cannonicalized_pt), sign_of_impulse * impulse_linear_magnitude);

        auto centripetal_force_total = cv.sign() * src_circle.radius() * cv.angular_magnitude() * cv.angular_magnitude();
        auto centripetal_force_impulse = impulse_vector.sign() * impulse_vector.circle().radius() * impulse_vector.angular_magnitude() * impulse_vector.angular_magnitude();
        auto residual_centripetal_force = centripetal_force_total - centripetal_force_impulse;

        auto orientation_of_residual = (residual_centripetal_force > 0) ? 1.0: -1.0;

        OutputDebugStringA( (orientation_of_residual > 0) ? "+\n" : "-\n" );

        residual_centripetal_force = std::abs(residual_centripetal_force);
        auto residual_linear_magnitude = cv.linear_magnitude() - impulse_linear_magnitude;

        auto residual_radius = (residual_linear_magnitude * residual_linear_magnitude) / residual_centripetal_force;
        auto residual_center_y = orientation_of_residual * residual_radius;
        auto residual_theta = (orientation_of_residual > 0) ? (3 * curvy::pi() / 2.0) : curvy::pi() / 2.0;

        auto residual_vector = circular_vector_from_linear_magnitude(curvy::circle(0, residual_center_y, residual_radius), residual_theta, orientation_of_residual * residual_linear_magnitude);

        return { impulse_vector, residual_vector };
    }
    */

    
    // inverse radius
    std::tuple<curvy::circular_vector, curvy::circular_vector> split_canonicalized_vector_into_components(const curvy::circular_vector& cv, const curvy::point& cannonicalized_pt, double min_radius)
    {
        auto src_circle = cv.circle();
        auto circle_of_impulse = get_circle_of_impulse(cannonicalized_pt);

        if (!is_pt_in_front_of_puck(cannonicalized_pt)) {
            return { curvy::circular_vector(circle_of_impulse, 0, 0), {} };
        }

        auto transfer_coefficient = get_momentum_transfer_factor(curvy::atan_of_pt(cannonicalized_pt), src_circle, circle_of_impulse, min_radius);
        auto sign_of_impulse = (circle_of_impulse.y() > 0) ? 1.0 : -1.0;
        auto impulse_linear_magnitude = transfer_coefficient * cv.linear_magnitude();

        auto impulse_vector = circular_vector_from_linear_magnitude(circle_of_impulse, curvy::get_angle_to_pt(circle_of_impulse.center(), cannonicalized_pt), sign_of_impulse * impulse_linear_magnitude);

        auto radius_mom_total = cv.signed_angular_magnitude() / src_circle.radius();
        auto radius_mom_impulse = impulse_vector.signed_angular_magnitude() / impulse_vector.circle().radius();
        auto residual_radius_mom = radius_mom_total - radius_mom_impulse;

        auto orientation_of_residual = (residual_radius_mom > 0) ? 1.0 : -1.0;

        //OutputDebugStringA((orientation_of_residual > 0) ? "+\n" : "-\n");

        residual_radius_mom = std::abs(residual_radius_mom);
        auto residual_linear_magnitude = cv.linear_magnitude() - impulse_linear_magnitude;

        auto residual_angular_magnitude = std::sqrt(residual_linear_magnitude * residual_radius_mom);
        auto residual_radius = residual_linear_magnitude / residual_angular_magnitude;
        auto residual_center_y = orientation_of_residual * residual_radius;
        auto residual_theta = (orientation_of_residual > 0) ? (3 * curvy::pi() / 2.0) : curvy::pi() / 2.0;

        auto residual_vector = curvy::circular_vector_from_linear_magnitude(curvy::circle(0, residual_center_y, residual_radius), residual_theta, orientation_of_residual * residual_linear_magnitude);

        return { impulse_vector, residual_vector };
    }
    
}

curvy::circular_vector::circular_vector(double theta, double cx, double cy, double r, double m) : 
    orientation_(m >= 0), theta_(theta), circle_(cx, cy, r), angular_magnitude_(std::abs(m))
{}

curvy::circular_vector::circular_vector(const curvy::circle & c, double t, double m) :
    orientation_(m >= 0), circle_(c), theta_(t), angular_magnitude_(std::abs(m))
{}

curvy::circular_vector::circular_vector(const curvy::circle & c, double t, bool o, double m) :
    orientation_(o), circle_(c), theta_(t), angular_magnitude_(std::abs(m))
{
}

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

double curvy::circular_vector::signed_linear_magnitude() const
{
    return sign() * linear_magnitude();
}

double curvy::circular_vector::circumference() const
{
    return circle_.circumference();
}

double curvy::circular_vector::sign() const
{
    return orientation_ ? 1.0 : -1.0;
}

std::tuple<curvy::circular_vector, curvy::circular_vector> curvy::circular_vector::split_into_components(const point& pt, double min_radius) const
{
    matrix to_canonical_coords = rotation_matrix(-direction_angle()) * translation_matrix(-position());
    matrix from_canonical_coords = translation_matrix(position()) * rotation_matrix(direction_angle());

    auto [impulse_vector, residual_vector] = split_canonicalized_vector_into_components(
        apply_matrix(to_canonical_coords, *this),
        apply_matrix(to_canonical_coords, pt),
        min_radius
    );

    return { 
        apply_matrix(from_canonical_coords, impulse_vector), 
        apply_matrix(from_canonical_coords, residual_vector)
    };
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