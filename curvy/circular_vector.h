#pragma once

#include "circle.h"
#include <tuple>

namespace curvy {

    class circular_vector {
    public:

        circular_vector( double theta = 0, double cx = 0, double cy = 0, double r = 0, double m = 0 );
        circular_vector(const curvy::circle& c, double t, double m);
        void set_magnitude(double m);
        void set_theta(double theta);
        void increment_theta(double amt);
        void set_radius(double r);
        void set_circle(const circle& c);
        curvy::circle& circle();

        bool orientation() const;
        double theta() const;
        double angular_magnitude() const;
        curvy::circle circle() const;
        point position() const;
        double direction_angle() const;
        double signed_magnitude() const;
        double linear_magnitude() const;
        double circumference() const;
        double sign() const;

    private:
        bool orientation_;
        double theta_;
        double angular_magnitude_;
        curvy::circle circle_;
    };

    circular_vector circular_vector_from_linear_magnitude(const circle& circ, double theta, double linear_magnitude);
    circular_vector operator*(double scalar, const circular_vector& cv);
    curvy::circular_vector apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& c);

}