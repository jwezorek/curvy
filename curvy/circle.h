#pragma once

#include <tuple>

double pi();
double normalize(const double value, const double start, const double end);
double normalize_angle(const double theta);

namespace curvy {

    struct circle {
        double x;
        double y;
        double r;

        circle(double cx, double cy, double r);
        circle(const std::tuple<double, double>& pt, double r);
        std::tuple<double, double, double, double> bounding_box() const;
        std::tuple<double, double> center() const;
    };

    struct circular_vector {
        double theta;
        double magnitude_angle;
        circle circle;

        circular_vector(
            double theta = 0, double cx = 0, double cy = 0,
            double r = 0, double speed = 0
        ) : theta(theta), circle(cx, cy, r), magnitude_angle(speed)
        {}

        std::tuple<double, double> position() const;
        double get_direction_angle() const;
        double get_direction_angle(double angle, double speed) const;
    };

}