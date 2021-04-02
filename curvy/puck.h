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
        void set_color(gdi::Color color);
        void set_theta(double theta);
        void set_speed(double speed);
        void set_puck_radius(double r);
        void set_radius_of_revolution(double r);
        void set_circle_rotation_position(double theta, double cx, double cy, double r);
        void set_center_of_revolution(const point& pt);

        puck update(double dt) const;
        circular_vector state() const;
        circular_vector& state();
        circular_vector momentum_vector() const;
        std::optional<double> get_collision_time(const puck& p, double dt, double eps) const;
        gdi::Color color() const;
        circle puck_circle() const;
        void paint(gdi::Graphics& g, double log_sz, int pix_sz) const;

    private:

        circular_vector state_;
        double puck_radius_;
        double mass_;
        gdi::Color color_;
    };
}