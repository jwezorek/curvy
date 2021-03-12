#pragma once

#include <Windows.h>
#include <gdiplus.h>
#include <tuple>
#include <optional>
#include "colors.h"

namespace gdi = Gdiplus;

namespace curvy {

    struct circle_rotation_state {
        double theta_;
        double cx_;
        double cy_;
        double r_;
        double speed_;

        circle_rotation_state(
            double theta = 0, double cx = 0, double cy = 0,
            double r = 0, double speed = 0
        ) : theta_(theta), cx_(cx), cy_(cy),
            r_(r), speed_(speed)
        {}
    };

    class puck {

    public:

        puck(const circle_rotation_state& crs = {}, gdi::Color color = colors::White, double puck_radius = 1, double mass = 1);
        void update(double dt);
        puck update(double dt) const;
        double theta() const;
        std::tuple<double, double> center_of_revolution() const;
        double radius_of_revolution() const;
        double angular_speed() const;
        double radius() const;
        std::tuple<double, double> position() const;
        bool intersects(const puck& p) const;
        double distance_from_center(const puck& p) const;
        double distance_from_intersection(const puck& p) const;
        std::optional<double> get_collision_time(const puck& p, double dt, double eps) const;
        gdi::Color color() const;
        void set_speed(double speed);
        void set_position(double theta, double cx, double cy, double r);
        std::tuple<int, int, int, int> get_location_in_pixels(float log_sz, int pix_sz) const;
        void paint(gdi::Graphics& g, double log_sz, int pix_sz) const;

    private:

        circle_rotation_state crs_;
        double puck_radius_;
        double mass_;
        gdi::Color color_;
    };
}