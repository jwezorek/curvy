#include <cmath>
#include <array>
#include <Windows.h>
#include "curvy_arithmetic_viewer.h"
#include "util.h"
#include "curvy_vector.h"

namespace {
}

curvy::curvy_arithmetic_viewer::curvy_arithmetic_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz),
    interaction_(interaction::none),
    addition_(true),
    vector_a_(circle(4, 0, 6), -1),
    vector_b_(circle(-4, 0, 6), 1)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::curvy_arithmetic_viewer::initialize()
{

}

void curvy::curvy_arithmetic_viewer::update()
{
    render();
}

void curvy::curvy_arithmetic_viewer::update(double dt)
{

}

bool curvy::curvy_arithmetic_viewer::handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down)
{
    auto loc = from_scr_coords(pt, logical_sz_, pixel_sz_);

    if (interaction_ == interaction::none && mouse_down) {
        interaction_ = get_interaction(loc);
        if (interaction_ == interaction::none)
            return false;

        render();
        return true;
    }

    if (!mouse_down && interaction_ != interaction::none) {
        interaction_ = interaction::none;
        render();
        return true;
    }

    return false;
}

bool curvy::curvy_arithmetic_viewer::handle_mouse_move(const std::tuple<int, int>& pix_pt)
{
    if (interaction_ != interaction::none) {
        auto pt = from_scr_coords(pix_pt, logical_sz_, pixel_sz_);
        auto [x, y] = pt;
        switch (interaction_) {
        
        }
        render();
        return true;
    }
    return false;
}

bool curvy::curvy_arithmetic_viewer::handle_key_press(unsigned int key, bool is_key_down)
{


    return false;
}

void curvy::curvy_arithmetic_viewer::set_logical_dimensions(double log_sz, bool refresh)
{
    logical_sz_ = log_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(pixel_sz_, pixel_sz_);
        render();
    }
}
void curvy::curvy_arithmetic_viewer::set_pixel_dimensions(int px_sz, bool refresh) {
    pixel_sz_ = px_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

gdi::Bitmap* curvy::curvy_arithmetic_viewer::get_bitmap() const
{
    return back_buffer_.get();
}

void curvy::curvy_arithmetic_viewer::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush(colors::Black);
    gdi::Pen white_pen(colors::White, 1);
    std::unique_ptr<gdi::Graphics> g(gdi::Graphics::FromImage(back_buffer_.get()));

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);

}

curvy::curvy_arithmetic_viewer::interaction curvy::curvy_arithmetic_viewer::get_interaction(const std::tuple<double, double>& click_location) const
{
    return interaction::none;
}
