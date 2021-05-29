#pragma once
#include <Windows.h>
#include <gdiplus.h>
#include <tuple>
#include "circle.h"

namespace gdi = Gdiplus;

namespace curvy {

    class post {

    public:

        post(const circle& c, gdi::Color color);
        const circle& post_circle() const;

        void paint(gdi::Graphics& g, double log_sz, int pix_sz) const;

    private:

        circle circle_;
        gdi::Color color_;
    };

}