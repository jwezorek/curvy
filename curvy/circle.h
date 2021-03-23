#pragma once

#include <tuple>
#include "util.h"

namespace curvy {

    struct circle {
        double x;
        double y;
        double r;

        circle(double cx, double cy, double r);
        circle(const point& pt, double r);
        std::tuple<double, double, double, double> bounding_box() const;
        point center() const;
        bool contains(const curvy::point& pt) const;
        bool is_on(const curvy::point& pt, double eps) const;
        double circumference() const;
        point invert(const point& pt) const;
        double diameter() const;
    };

    curvy::circle apply_matrix(const curvy::matrix& mat, const curvy::circle& c);

}

