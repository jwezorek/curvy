#include <Windows.h>
#include "impulse_viewer.h"
#include "util.h"

curvy::impulse_viewer::impulse_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::impulse_viewer::initialize()
{
    puck_b_.set_position(0, 0, 0, 2);
}

void curvy::impulse_viewer::update(double dt)
{
    auto [scr_x, scr_y] = to_scr_coords( 2.5, 6, logical_sz_, pixel_sz_);
    auto [log_x, log_y] = from_scr_coords(scr_x, scr_y, logical_sz_, pixel_sz_);
}

void curvy::impulse_viewer::handle_mouse_click(int x, int y, bool mouse_down)
{
}

void curvy::impulse_viewer::handle_mouse_move(int x, int y)
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

    puck_a_.paint(*g, logical_sz_, pixel_sz_);
    puck_b_.paint(*g, logical_sz_, pixel_sz_);

    delete g;
}
