#pragma once

#include <Windows.h>
#include <gdiplus.h>
#include <tuple>
#include <optional>
#include "colors.h"
#include "circle.h"
#include "circular_vector.h"

namespace gdi = Gdiplus;

namespace curvy {

    class puck {

    public:

        puck(const circular_vector& crs = {}, gdi::Color color = colors::White, double puck_radius = 1, double mass = 1);
        void update(double dt);
        puck update(double dt) const;
        double theta() const;
        std::tuple<double, double> center_of_revolution() const;
        circle circle_of_revolution() const;
        double radius_of_revolution() const;
        double angular_speed() const;
        double radius() const;
        std::tuple<double, double> position() const;
        bool contains_point(double x, double y) const;
        bool contains_point(const std::tuple<double,double>& pt) const;
        bool intersects(const puck& p) const;
        double distance_from_center(const puck& p) const;
        double distance_from_intersection(const puck& p) const;
        double direction() const;
        double puck_radius() const;
        circular_vector circle_rot_state() const;
        circular_vector momentum_vector() const;
        circular_vector momentum_vector_through_point(const std::tuple<double, double>& pt);
        std::optional<double> get_collision_time(const puck& p, double dt, double eps) const;
        gdi::Color color() const;
        void set_color(gdi::Color color);
        void set_theta(double theta);
        void set_speed(double speed);
        void set_puck_radius(double r);
        void set_radius_of_revolution(double r);
        void set_circle_rotation_position(double theta, double cx, double cy, double r);
        void set_center_of_revolution(const std::tuple<double, double>& pt);
        circle get_puck_circle() const;
        std::tuple<int, int, int, int> get_bounding_box_in_pixels(double log_sz, int pix_sz) const;
        void paint(gdi::Graphics& g, double log_sz, int pix_sz) const;
    private:

        circular_vector crs_;
        double puck_radius_;
        double mass_;
        gdi::Color color_;
    };
}