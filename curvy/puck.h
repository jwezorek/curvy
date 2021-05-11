#pragma once

#include <Windows.h>
#include <gdiplus.h>
#include <tuple>
#include <optional>
#include <unordered_set>
#include "colors.h"
#include "circle.h"
#include "curvy_vector.h"

namespace gdi = Gdiplus;

namespace curvy {

    class puck {

    public:

        puck(const curvy_vector& crs = {}, double theta = 0, gdi::Color color = colors::White, double puck_radius = 1, double mass = 1);
        puck(double x, double, gdi::Color color = colors::White, double puck_radius = 1, double mass = 1);
        void update(double dt);
        void set_color(gdi::Color color);
        void set_theta(double theta);
        void set_speed(double speed);
        void set_puck_radius(double r);
        void set_radius_of_revolution(double r);
        void set_circle_rotation_position(double theta, double cx, double cy, double r);
        void set_center_of_revolution(const point& pt);
        void set_position(const point& pt);
        void set_vector(const curvy_vector& v);
        void update_contact_list();
        void add_to_contact_list(const puck& p);

        point position() const;
        double theta() const;
        double direction() const;
        puck update(double dt) const;
        curvy_vector state() const;
        curvy_vector& state();
        curvy_vector momentum_vector() const;
        std::optional<double> get_collision_time(const puck& p, double dt, double eps) const;
        std::optional<double> get_boundary_collision_time(const circle& border, double dt, double eps) const;
        gdi::Color color() const;
        circle puck_circle() const;
        void paint(gdi::Graphics& g, double log_sz, int pix_sz) const;
        bool is_in_contact_list(const puck& p) const;

    private:

        

        std::unordered_set<const puck*> contact_list_;
        double theta_;
        curvy_vector state_;
        double puck_radius_;
        double mass_;
        gdi::Color color_;
    };

    bool is_in_contact_or_intersecting(const puck& p1, const puck& p2);
}