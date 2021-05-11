#pragma once

#include "circle.h"
#include <tuple>
#include <string>

namespace curvy {

    class curvy_vector {
    public:

        curvy_vector( double cx = 0, double cy = 0, double r = 0, double m = 0 );
        curvy_vector(const curvy::circle& c, double m);
        curvy_vector(const curvy::circle& c, bool o, double m);
        void set_magnitude(double m);
        void set_radius(double r);
        void set_circle(const circle& c);
        curvy::circle& circle();

        bool orientation() const;
        curvy::circle circle() const;
        std::optional<double> angular_magnitude() const;
        std::optional<double> signed_angular_magnitude() const;
        double linear_magnitude() const;
        double signed_linear_magnitude() const;
        double circumference() const;
        double sign() const;
        point newtonian_vector_at_point(const point& pt) const;
        curvy_vector add(const curvy_vector& cv, const point& where) const;
        curvy_vector subtract(const curvy_vector& cv, const point& where) const;
        std::string to_string() const;
        double direction_at(const point& pt) const;

    private:

        bool orientation_;
        double linear_magnitude_;
        curvy::circle circle_;
    };

    curvy_vector operator*(const curvy_vector& cv, double scale);
    curvy_vector operator*(double scale, const curvy_vector& cv);

    curvy_vector circular_vector_from_angular_magnitude(const circle& circ, bool orientation, double linear_magnitude);
    curvy_vector circular_vector_from_angular_magnitude(const circle& circ, double linear_magnitude);
    curvy_vector operator*(double scalar, const curvy_vector& cv);
    curvy::curvy_vector apply_matrix(const curvy::matrix& mat, const curvy::curvy_vector& c);
    std::tuple<circle, bool> circular_direction_through_two_points(const point& pt1, double direction_at_pt1, const point& pt2);
    bool is_in_front_of(const curvy::point& pt1, double direction_at_pt1, const curvy::point& pt2);
    double momentum_transfer_factor(const curvy::point& pt1, double pt1_direction, bool orientation, const curvy::point& pt2, double r, double d);

    double to_angle_of_curvature(const curvy_vector& cv);
    curvy_vector from_angle_of_curvature(double curvature, const curvy::point& center, double magnitude);
}