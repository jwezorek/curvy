#pragma once

#include <tuple>

double pi();
double normalize(const double value, const double start, const double end);
double normalize_angle(const double theta);
double euclidean_distance(double x1, double y1, double x2, double y2);
double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2);

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
}