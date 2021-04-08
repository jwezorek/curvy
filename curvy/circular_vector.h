#pragma once

#include "circle.h"
#include <tuple>
#include <string>

namespace curvy {

    class circular_vector {
    public:

        circular_vector( double cx = 0, double cy = 0, double r = 0, double m = 0 );
        circular_vector(const curvy::circle& c, double m);
        circular_vector(const curvy::circle& c, bool o, double m);
        void set_magnitude(double m);
        void set_radius(double r);
        void set_circle(const circle& c);
        curvy::circle& circle();

        bool orientation() const;
        curvy::circle circle() const;
        double angular_magnitude() const;
        double signed_angular_magnitude() const;
        double linear_magnitude() const;
        double signed_linear_magnitude() const;
        double circumference() const;
        double sign() const;
        std::tuple<circular_vector, circular_vector> split_into_components(const point& pt_from, const point& pt_to, double puck_sz) const;

        point newtonian_vector_at_point(const point& pt) const;
        circular_vector add(const circular_vector& cv, const point& where) const;
        circular_vector subtract(const circular_vector& cv, const point& where) const;
        std::string to_string() const;
        double direction_at(const point& pt) const;

    private:

        bool orientation_;
        double angular_magnitude_;
        curvy::circle circle_;
    };

    circular_vector operator*(const circular_vector& cv, double scale);
    circular_vector operator*(double scale, const circular_vector& cv);

    circular_vector circular_vector_from_linear_magnitude(const circle& circ, double linear_magnitude);
    circular_vector operator*(double scalar, const circular_vector& cv);
    curvy::circular_vector apply_matrix(const curvy::matrix& mat, const curvy::circular_vector& c);
    circle circle_in_direction_through_two_points(const point& pt1, double direction_at_pt1, const point& pt2);
}