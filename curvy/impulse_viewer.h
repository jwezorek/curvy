#pragma once

#include <gdiplus.h>
#include <memory>
#include <vector>
#include <tuple>
#include <optional>
#include "colors.h"

namespace gdi = Gdiplus;

namespace curvy {

    class impulse_viewer {

    public:
        impulse_viewer(int px_sz, double log_sz);
        void set_dimensions(int px_sz, double log_sz = 0);
        int get_size() const;
        gdi::Bitmap* get_bitmap() const;

    private:

        void render();

        std::unique_ptr<gdi::Bitmap> back_buffer_;
        double logical_sz_;
        int pixel_sz_;
    };

}