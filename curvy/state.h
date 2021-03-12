#pragma once

#include <gdiplus.h>

namespace gdi = Gdiplus;

namespace curvy {

    class state {
    public:
        virtual void initialize() = 0;
        virtual void update(double dt) = 0;
        virtual void handle_mouse_click(int x, int y, bool mouse_down) = 0;
        virtual void handle_mouse_move(int x, int y) = 0;
        virtual void set_pixel_dimensions(int px_sz, bool refresh = true) = 0;
        virtual void set_logical_dimensions(double log_sz, bool refresh = false) = 0;
        virtual gdi::Bitmap* get_bitmap() const = 0;
    };

 }