#include <Windows.h>
#include "impulse_viewer.h"


curvy::impulse_viewer::impulse_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::impulse_viewer::initialize()
{
}

void curvy::impulse_viewer::update(double dt)
{
}

void curvy::impulse_viewer::set_logical_dimensions(double log_sz, bool refresh)
{
    logical_sz_ = log_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(pixel_sz_, pixel_sz_);
        render();
    }
}
void curvy::impulse_viewer::set_pixel_dimensions(int px_sz, bool refresh) {
    pixel_sz_ = px_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

gdi::Bitmap* curvy::impulse_viewer::get_bitmap() const
{
    return back_buffer_.get();
}

void curvy::impulse_viewer::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush(colors::Black);
    gdi::Pen white_pen(colors::White, 1);
    auto* g = gdi::Graphics::FromImage(back_buffer_.get());

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);


    delete g;
}
