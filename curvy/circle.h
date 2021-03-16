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

    struct circle_rotation_state {
        double theta;
        double speed;
        circle circle;

        circle_rotation_state(
            double theta = 0, double cx = 0, double cy = 0,
            double r = 0, double speed = 0
        ) : theta(theta), circle(cx, cy, r), speed(speed)
        {}

        std::tuple<double, double> position() const;
        double get_direction_angle() const;
    };

}