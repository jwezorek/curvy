#pragma once

#include <tuple>
#include "util.h"

namespace curvy {

    class circle {
    public:
        circle(double cx, double cy, double r);
        circle(const point& pt, double r);
        void set_center(const point& pt);
        void set_radius(double r);

        std::tuple<double, double, double, double> bounding_box() const;
        point center() const;
        bool contains(const curvy::point& pt) const;
        bool perimeter_contains(const curvy::point& pt, double eps) const;
        double circumference() const;
        point invert(const point& pt) const;
        double diameter() const;
        double x() const;
        double y() const;
        double radius() const;

    private:
        point center_;
        double radius_;
    };

    circle apply_matrix(const matrix& mat, const circle& c);

}

