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

    double get_curvy_energy(const curvy::circular_vector& cv) {
        return cv.signed_angular_magnitude() / cv.circle().radius();
    }

    curvy::circular_vector unpack_curvy_energy(double curvy_energy, double linear_magnitude) {
        auto orientation = (curvy_energy > 0) ? 1.0 : -1.0;
        curvy_energy = std::abs(curvy_energy);

        auto angular_magnitude = std::sqrt( linear_magnitude * curvy_energy);
        auto radius = angular_magnitude / curvy_energy;
        auto center_y = orientation * radius;
        return curvy::circular_vector(curvy::circle(0, center_y, radius), orientation * angular_magnitude);
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

        auto impulse_vector = circular_vector_from_linear_magnitude(circle_of_impulse, impulse_signed_linear_magnitude);

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
        auto residual_vector = curvy::circular_vector(curvy::circle(0, residual_center_y, residual_radius), orientation_of_residual, residual_angular_magnitude);

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
            curvy::circular_vector(circle_of_impulse, angular_speed_of_impulse),
            curvy::circular_vector(residual_circle, residual_angular_speed)
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

        auto impulse_vector = circular_vector_from_linear_magnitude(circle_of_impulse, sign_of_impulse * impulse_linear_magnitude);

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

        auto residual_vector = circular_vector_from_linear_magnitude(curvy::circle(0, residual_center_y, residual_radius), orientation_of_residual * residual_linear_magnitude);

        return { impulse_vector, residual_vector };
    }
    */

    
    // angular velocity * inverse radius
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

        auto impulse_vector = circular_vector_from_linear_magnitude(circle_of_impulse, sign_of_impulse * impulse_linear_magnitude);

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

        auto residual_vector = curvy::circular_vector(curvy::circle(0, residual_center_y, residual_radius), orientation_of_residual * residual_angular_magnitude);

        return { impulse_vector, residual_vector };
    }
}

curvy::circular_vector::circular_vector(double cx, double cy, double r, double m) : 
    orientation_(m >= 0), circle_(cx, cy, r), angular_magnitude_(std::abs(m))
{}

curvy::circular_vector::circular_vector(const curvy::circle & c, double m) :
    orientation_(m >= 0), circle_(c), angular_magnitude_(std::abs(m))
{}

curvy::circular_vector::circular_vector(const curvy::circle & c, bool o, double m) :
    orientation_(o), circle_(c), angular_magnitude_(std::abs(m))
{
}

void curvy::circular_vector::set_magnitude(double m)
{
    orientation_ = m > 0;
    angular_magnitude_ = std::abs(m);
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


double curvy::circular_vector::angular_magnitude() const
{
    return angular_magnitude_;
}

curvy::circle curvy::circular_vector::circle() const
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

std::tuple<curvy::circular_vector, curvy::circular_vector> curvy::circular_vector::split_into_components(const point& pt_from, const point& pt_to, double min_radius) const
{
    auto direction_angle = direction_at(pt_from);
    matrix to_canonical_coords = rotation_matrix(-direction_angle) * translation_matrix(-pt_from);
    matrix from_canonical_coords = translation_matrix(pt_from) * rotation_matrix(direction_angle);

    auto [impulse_vector, residual_vector] = split_canonicalized_vector_into_components(
        apply_matrix(to_canonical_coords, *this),
        apply_matrix(to_canonical_coords, pt_to),
        min_radius
    );

    return { 
        apply_matrix(from_canonical_coords, impulse_vector), 
        apply_matrix(from_canonical_coords, residual_vector)
    };
}

curvy::point curvy::circular_vector::newtonian_vector_at_point(const point& pt) const
{
    auto angle = direction_at(pt);
    auto magnitude = linear_magnitude();
    auto vec = point{ std::cos(angle), std::sin(angle) };
    return magnitude * vec;
}

curvy::circular_vector curvy::circular_vector::add(const circular_vector& cv, const point& where) const
{
    auto this_vec = newtonian_vector_at_point(where);
    auto that_vec = cv.newtonian_vector_at_point(where);
    auto new_vec = this_vec + that_vec;
    auto new_linear_magnitude = hypot_of_point(new_vec);
    auto new_direction = atan_of_pt(new_vec);

    matrix from_canonical_coords = translation_matrix(where) * rotation_matrix(new_direction);
    auto curvy_energy = get_curvy_energy(*this) + get_curvy_energy(cv);
    auto cv_sum = unpack_curvy_energy(curvy_energy, new_linear_magnitude);

    return apply_matrix(from_canonical_coords, cv_sum);
}

curvy::circular_vector curvy::circular_vector::subtract(const circular_vector& cv, const point& where) const
{
    return circular_vector();
}

std::string curvy::circular_vector::to_string() const
{
    return "[ " + circle_.to_string() + " , " + std::to_string(angular_magnitude_) + " ]";
}

double curvy::circular_vector::direction_at(const point& pt) const
{
    return direction_on_circle(get_angle_to_pt(circle_.center(), pt), orientation_);
}

curvy::circular_vector curvy::circular_vector_from_linear_magnitude(const curvy::circle& circ, double linear_magnitude)
{
    auto angular_magnitude = linear_magnitude / circ.radius();
    return curvy::circular_vector(circ, angular_magnitude);
}

curvy::circular_vector curvy::operator*(double scale, const circular_vector& cv)
{
    return circular_vector(cv.circle(), scale * cv.angular_magnitude());
}

curvy::circular_vector curvy::operator*(const circular_vector& cv, double scale)
{
    return scale * cv;
}

curvy::circular_vector curvy::apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& cv)
{
    auto circle = curvy::apply_matrix(mat, cv.circle());
    return curvy::circular_vector(
        circle,
        cv.sign() * cv.angular_magnitude()
    );
}

curvy::circle curvy::circle_in_direction_through_two_points(const curvy::point& pt1, double direction_at_pt1, const curvy::point& pt2)
{
    matrix to_canonical_coords = rotation_matrix(-direction_at_pt1) * translation_matrix(-pt1);
    matrix from_canonical_coords = translation_matrix(pt1) * rotation_matrix(direction_at_pt1);

    auto [px, py] = apply_matrix(to_canonical_coords, pt2);
    double circle_of_impulse_y = (px * px + py * py) / (2 * py);
    return apply_matrix(from_canonical_coords, curvy::circle(0, circle_of_impulse_y, std::abs(circle_of_impulse_y)));

}
