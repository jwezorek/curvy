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

struct circle_and_orientation {
    double direction;
    curvy::circle c;
    bool orientation;

    circle_and_orientation(double d, const curvy::circle& c, bool o) :
        direction(d), c(c), orientation(o)
    {}
};

void insert_circle_and_orientation(std::vector<circle_and_orientation>& vec, const curvy::circle& c, const curvy::point& where) {
    vec.emplace_back(curvy::direction_on_circle(c, where, true), c, true );
    vec.emplace_back(curvy::direction_on_circle(c, where, false), c, false);
}

std::vector<circle_and_orientation> get_circles_and_orientations(const curvy::circle& c1, const curvy::circle& c2, const curvy::point& where) {
    
    auto midcircles = curvy::mid_circles(c1 , c2);
    if (!midcircles)
        return {};
    auto [mid_circle_1, mid_circle_2] = midcircles.value();

    std::vector<circle_and_orientation> output;
    insert_circle_and_orientation(output, c1, where);
    if (c2.radius() > 0)
        insert_circle_and_orientation(output, c2, where);

    return output;
}

circle_and_orientation find_best_mid_circle(double direction, const curvy::circle& c1, const curvy::circle& c2, const curvy::vector_arithmetic_context& ctxt) {
    using namespace curvy;
    auto candidates = get_circles_and_orientations(c1, c2, ctxt.pt1);
    if (candidates.empty()) {
        return { 0, {0,0,0}, 0 }; // TODO: handle degenerate circles.
    }
    auto max_cosine_measure = -1.0;
    const circle_and_orientation* best_choice = nullptr;

    auto target_vector = curvy::normalize_pt(ctxt.pt1 - ctxt.pt2);
    for (const auto& candidate : candidates) {
        if (candidate.c.radius() < euclidean_distance(ctxt.pt1, ctxt.pt2) / 2.0)
            continue;
        auto candidate_vector = curvy::pt_on_unit_circle(candidate.direction);
        auto cosine_measure = curvy::dot_product(target_vector, candidate_vector);
        if (cosine_measure > max_cosine_measure) {
            max_cosine_measure = cosine_measure;
            best_choice = &candidate;
        }
    }

    return *best_choice;
}

#define INVERSION

#ifdef INVERSION
curvy::curvy_vector curvy::curvy_vector::add(const curvy_vector& cv, const vector_arithmetic_context& ctxt) const
{
    if (angular_magnitude() == 0)
        return cv;

    if (cv.angular_magnitude() == 0)
        return *this;

    auto linear_magnitude_of_sum = this->linear_magnitude() + cv.linear_magnitude();
    auto direction_of_sum = atan_of_pt(this->newtonian_vector_at_point(ctxt.pt1) + cv.newtonian_vector_at_point(ctxt.pt1));
    auto mid_circle = find_best_mid_circle(direction_of_sum, this->circle(), cv.circle(), ctxt);
    auto sign = mid_circle.orientation ? 1.0 : -1.0;

    return curvy::circular_vector_from_linear_magnitude(mid_circle.c, sign * linear_magnitude_of_sum);
}


curvy::curvy_vector curvy::curvy_vector::subtract(const curvy_vector& cv, const vector_arithmetic_context& ctxt) const
{
    auto linear_magnitude_of_sum = this->linear_magnitude() - cv.linear_magnitude();
    auto mid_circle = this->circle().invert(cv.circle());
    auto sign = -1.0 * cv.sign();

    return curvy::circular_vector_from_linear_magnitude(mid_circle, sign * linear_magnitude_of_sum);
}
#else

/* conservation of a second quantity + newtonian vector direction */

curvy::curvy_vector curvy::curvy_vector::add(const curvy_vector& cv, const vector_arithmetic_context& ctxt) const
{
    return circular_vector_arithmetic_aux(*this, cv, ctxt.pt1, [](double v1, double v2) {return v1 + v2; });
}

curvy::curvy_vector curvy::curvy_vector::subtract(const curvy_vector& cv, const vector_arithmetic_context& ctxt) const
{
    return circular_vector_arithmetic_aux(*this, cv, ctxt.pt1, [](double v1, double v2) {return v1 - v2; });
}

#endif


