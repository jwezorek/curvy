#pragma once

#include <gdiplus.h>
#include <memory>
#include <vector>
#include <tuple>
#include <optional>
#include "state.h"
#include "colors.h"
#include "curvy_vector.h"

namespace gdi = Gdiplus;

namespace curvy {

    class curvy_arithmetic_viewer : public state {

        enum class interaction {
            moving_circle_a,
            resizing_circle_a,
            moving_circle_b,
            resizing_circle_b,
            moving_arrow_a,
            moving_arrow_b,
            none
        };

    public:

        struct move_circle_state {
            point start;
            circle c;
        };

        curvy_arithmetic_viewer(int px_sz, double log_sz);

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

        std::unique_ptr<gdi::Bitmap> back_buffer_;
        double logical_sz_;
        int pixel_sz_;
        bool addition_;
        curvy_vector vector_a_;
        curvy_vector vector_b_;
        interaction interaction_;
        std::optional<move_circle_state> move_circle_;
        std::optional<point> resize_circle_center_;
        std::optional<bool> move_arrow_orientation_;
    };
}