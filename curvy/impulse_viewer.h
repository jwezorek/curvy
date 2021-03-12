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

    class impulse_viewer : public state {

    public:
        impulse_viewer(int px_sz, double log_sz);

        void initialize() override;
        void update(double dt) override;
        void handle_mouse_click(int x, int y, bool mouse_down) override;
        void handle_mouse_move(int x, int y) override;
        void set_pixel_dimensions(int px_sz, bool refresh) override;
        void set_logical_dimensions(double log_sz, bool refresh) override;
        gdi::Bitmap* get_bitmap() const override;

    private:

        void render();

        std::unique_ptr<gdi::Bitmap> back_buffer_;
        double logical_sz_;
        int pixel_sz_;
        puck puck_a_;
        puck puck_b_;
    };

}