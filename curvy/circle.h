#pragma once

#include <tuple>
#include "util.h"

namespace curvy {

    struct circle {
        double x;
        double y;
        double r;

        circle(double cx, double cy, double r);
        circle(const std::tuple<double, double>& pt, double r);
        std::tuple<double, double, double, double> bounding_box() const;
        std::tuple<double, double> center() const;
        double circumference() const;
        std::tuple<double, double> invert(const std::tuple<double, double>& pt) const;
        double diameter() const;
    };

    struct circular_vector {
        double theta;
        double angular_magnitude;
        circle circle;

        circular_vector(
            double theta = 0, double cx = 0, double cy = 0,
            double r = 0, double speed = 0
        ) : theta(theta), circle(cx, cy, r), angular_magnitude(speed)
        {}

        circular_vector(const curvy::circle& c, double t, double am) :
            circle(c), theta(t), angular_magnitude(am)
        {}

        std::tuple<double, double> position() const;
        double direction_angle() const;
        double direction_angle(double angle, double speed) const;
        double linear_magnitude() const;
        double circumference() const;
    };

    circular_vector circular_vector_from_linear_magnitude(const circle& circ, double theta, double linear_magnitude);
    circular_vector operator*(double scalar, const circular_vector& cv);
    curvy::circle apply_matrix(const curvy::matrix& mat, const curvy::circle& c);
    curvy::circular_vector apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& c);
}

bool is_pt_on_circle(const curvy::circle& c, const curvy::point& pt, double eps);
bool is_pt_in_circle(const curvy::circle& c, const curvy::point& pt, double eps = 0);
std::tuple<double, double> closest_pt_on_circle(const curvy::circle& c, const curvy::point& pt);