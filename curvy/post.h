#pragma once
#include <Windows.h>
#include <gdiplus.h>
#include <tuple>
#include <optional>
#include "circle.h"
#include "puck.h"

namespace gdi = Gdiplus;

namespace curvy {

    class post {

    public:

        post(const circle& c, gdi::Color color);
        point position() const;
        const circle& post_circle() const;
        std::optional<double> get_collision_time(const puck& p, double dt, double eps) const;
        void paint(gdi::Graphics& g, double log_sz, int pix_sz) const;

    private:

        circle circle_;
        gdi::Color color_;
    };

}