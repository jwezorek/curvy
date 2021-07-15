#pragma once

#include <tuple>
#include <optional>
#include <string>
#include "util.h"

namespace curvy {

    class circle {
    public:
        circle(double cx, double cy, double r);
        circle(const point& pt, double r);
        circle(bool is_degenerate, const point& pt, double v);

        bool is_degenerate() const;
        double degenerate_angle() const;

        void set_center(const point& pt);
        void set_radius(double r);

        std::tuple<double, double, double, double> bounding_box() const;
        point center() const;
        bool contains(const curvy::point& pt) const;
        bool perimeter_contains(const curvy::point& pt, double eps) const;
        double circumference() const;
        point get_point(double theta) const;
        point invert(const point& pt) const;
        circle invert(const circle& pt) const;
        double diameter() const;
        double x() const;
        double y() const;
        double radius() const;
        std::string to_string() const;

    private:
        point center_;
        double radius_;
        std::optional<double> degenerate_angle_;
    };

    circle degenerate_circle(const point& pt1, const point& pt2);
    bool operator==(const circle& c1, const circle& c2);
    double direction_on_circle(const circle& c, const point& pt, bool orientation);
    std::tuple<std::tuple<point,point>, std::tuple<point, point>> mutual_tangents(const circle& c1, const circle& c2);
    std::optional<std::tuple<point, point>> intersections(const circle& c1, const circle& c2);
    circle orthogonal_circle(const circle& c, const point& pt1, const point& pt2);
    circle apply_matrix(const matrix& mat, const circle& c);
    point closest_pt_on_circle(const curvy::circle& c, const curvy::point& pt);
    std::optional<circle> circle_through_three_points(const point& pt1, const point& pt2, const point& pt3);
    std::optional<std::tuple<circle, circle>> mid_circles(const circle& c1, const circle& c2);
    std::optional<std::tuple<circle, circle>> circles_of_given_radius_through_two_points(double radius, const point& pt1, const point& pt2);
    std::tuple<int, int, int, int> bounding_box_in_pixels(const curvy::circle& c, double log_sz, int pix_sz);
    double angle_of_curvature(const circle& circle, bool orientation);
}

