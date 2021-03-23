#pragma once

#include "circle.h"
#include <tuple>

namespace curvy {

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
    curvy::circular_vector apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& c);

}