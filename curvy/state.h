#pragma once

#include <gdiplus.h>
#include <tuple>

namespace gdi = Gdiplus;

namespace curvy {

    class state {
    public:
        virtual void initialize() = 0;
        virtual void update(double dt) = 0;
        virtual bool handle_mouse_click(const std::tuple<int,int>& pt, bool mouse_down) = 0;
        virtual bool handle_mouse_move(const std::tuple<int, int>& pt) = 0;
        virtual void set_pixel_dimensions(int px_sz, bool refresh = true) = 0;
        virtual void set_logical_dimensions(double log_sz, bool refresh = false) = 0;
        virtual gdi::Bitmap* get_bitmap() const = 0;
    };

 }