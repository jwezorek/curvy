#pragma once

#include <gdiplus.h>
#include <memory>
#include <vector>
#include <tuple>
#include <optional>
#include "state.h"
#include "colors.h"
#include "puck.h"

namespace gdi = Gdiplus;

namespace curvy {

    enum class interaction {
        dragging_circle_of_rev,
        dragging_a,
        dragging_b,
        resizing_circle_of_rev,
        dragging_circle_b,
        resizing_circle_b,
        dragging_arrow_b,
        none
    };

    class curvy_vector_viewer : public state {

    public:
        curvy_vector_viewer(int px_sz, double log_sz);

        void initialize() override;
        void update() override;
        void update(double dt) override;
        bool handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down) override;
        bool handle_mouse_move(const std::tuple<int, int>& pt) override;
        bool handle_key_press(unsigned int key, bool is_key_down) override;
        void set_pixel_dimensions(int px_sz, bool refresh) override;
        void set_logical_dimensions(double log_sz, bool refresh) override;
        gdi::Bitmap* get_bitmap() const override;

    private:

        void render();
        interaction get_interaction(const std::tuple<double, double>& click_location) const;
        void sync_b_with_a(double old_a_theta = 0);
        void rotate_circle_b(const point& pt);
        void resize_circle_b(const point& pt);
        void drag_arrow_b(const point& pt);
        bool is_in_arrow_b(const point& pt) const;

        std::unique_ptr<gdi::Bitmap> back_buffer_;
        double logical_sz_;
        int pixel_sz_;
        puck puck_a_;
        puck puck_b_;
        double puck_b_theta_;
        interaction interaction_;
        double b_speed_;
        bool show_puck_b_vectors_;
    };

}