#pragma once

#include <Windows.h>
#include <vector>
#include <tuple>
#include <optional>
#include "colors.h"

namespace curvy {

    struct circle_rotation_state {
        double theta_;
        double cx_;
        double cy_;
        double r_;
        double speed_;
    };

    class puck {

    public:

        puck(const circle_rotation_state& crs, COLORREF color = colors::White, double puck_radius = 1, double mass = 1);
        void update( double dt );
        puck update( double dt ) const;
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
        COLORREF color() const;
        void set_speed(double speed);

    private:

        circle_rotation_state crs_;
        double puck_radius_;
        double mass_;
        COLORREF color_;
    };
    

    double pi();

    class state {

    public:
        state(int px_sz, double log_sz);
        void set_dimensions(int px_sz, double log_sz = 0);
        int get_size() const;
        void insert(const puck& p);
        HBITMAP get_bitmap() const;
        void update(double dt);

    private:

        void render();
        void paint_puck(HDC hdc, const puck& p);
        std::tuple<int, int, int, int> get_location_in_pixels(const puck& p) const;
        std::tuple<int, int, int, int> to_scr_coords(double x1, double y1, double x2, double y2) const;

        using collision = std::tuple<puck*, puck*>;
        using collisions = std::vector<collision>;

        std::tuple<collisions, double> get_next_collisions(double dt, double eps);
        void handle_collision(collision& c);
        void handle_collisions(collisions& c);

        HBITMAP back_buffer_;
        std::vector<puck> pucks_;
        double logical_sz_;
        int pixel_sz_;
    };

}