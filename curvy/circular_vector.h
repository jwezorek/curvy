#pragma once

#include "circle.h"
#include <tuple>

namespace curvy {

    struct circular_vector {
        bool orientation;
        double theta;
        double angular_magnitude_;
        circle circle;

        circular_vector( double theta = 0, double cx = 0, double cy = 0, double r = 0, double m = 0 );
        circular_vector(const curvy::circle& c, double t, double m);

        void set_magnitude(double m);

        std::tuple<double, double> position() const;
        double direction_angle() const;
        double signed_magnitude() const;
        double linear_magnitude() const;
        double circumference() const;
        double sign() const;
    };

    circular_vector circular_vector_from_linear_magnitude(const circle& circ, double theta, double linear_magnitude);
    circular_vector operator*(double scalar, const circular_vector& cv);
    curvy::circular_vector apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& c);

}