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
    curvy::circle apply_matrix(const curvy::matrix& mat, const curvy::circle& c);
    bool is_pt_on_circle(const curvy::circle& c, const curvy::point& pt, double eps);
    bool is_pt_in_circle(const curvy::circle& c, const curvy::point& pt, double eps = 0);
    std::tuple<double, double> closest_pt_on_circle(const curvy::circle& c, const curvy::point& pt);

}

