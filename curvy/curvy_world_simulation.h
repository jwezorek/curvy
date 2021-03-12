#pragma once

#include <memory>
#include <vector>
#include <gdiplus.h>
#include <tuple>
#include <optional>
#include "colors.h"
#include "state.h"
#include "puck.h"

namespace gdi = Gdiplus;

namespace curvy {

    class curvy_world_simulation : public state {

    public:
        curvy_world_simulation(int px_sz, double log_sz);
        void initialize() override;
        void set_pixel_dimensions(int px_sz, bool refresh) override;
        void set_logical_dimensions(double log_sz, bool refresh) override;
        void handle_mouse_click(int x, int y, bool mouse_down) override;
        void handle_mouse_move(int x, int y) override;
        int get_size() const;
        void insert(const puck& p);
        gdi::Bitmap* get_bitmap() const override;
        void update(double dt) override;

    private:

        void render();
        void paint_puck(gdi::Graphics& g, const puck& p);
        std::tuple<int, int, int, int> get_location_in_pixels(const puck& p) const;

        using collision = std::tuple<puck*, puck*>;
        using collisions = std::vector<collision>;

        std::tuple<collisions, double> get_next_collisions(double dt, double eps);
        void handle_collision(collision& c);
        void handle_collisions(collisions& c);

        std::unique_ptr<gdi::Bitmap> back_buffer_;
        std::vector<puck> pucks_;
        double logical_sz_;
        int pixel_sz_;
    };

}