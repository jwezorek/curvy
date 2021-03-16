#include <Windows.h>
#include <cmath>
#include "impulse_viewer.h"
#include "util.h"

namespace {

    void paint_circle(gdi::Graphics& g, const curvy::circle& c, gdi::Color color, double log_sz, int pix_sz) {
        auto [x1, y1, x2, y2] = to_scr_coords(c.bounding_box(), log_sz, pix_sz);
        gdi::Pen pen(color, 3);
        g.DrawEllipse(&pen, x1, y1, x2 - x1, y2 - y1);
    }
}

curvy::impulse_viewer::impulse_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz),
    interaction_(interaction::none)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::impulse_viewer::initialize()
{
    puck_a_.set_circle_rotation_position(0, -3, 0, 3);
    puck_b_.set_circle_rotation_position(0, 0, 0, 2);
}

void curvy::impulse_viewer::update(double dt)
{
    auto [scr_x, scr_y] = to_scr_coords( 2.5, 6, logical_sz_, pixel_sz_);
    auto [log_x, log_y] = from_scr_coords(scr_x, scr_y, logical_sz_, pixel_sz_);
}

bool curvy::impulse_viewer::handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down)
{
    auto loc = from_scr_coords(pt, logical_sz_, pixel_sz_);

    if (interaction_ == interaction::none && mouse_down) {
        interaction_ = get_interaction(loc);
        if (interaction_ == interaction::none)
            return false;
        switch (interaction_) {
            case interaction::dragging_a:
                puck_a_.set_color(colors::Aqua);
                break;
            case interaction::dragging_b:
                puck_b_.set_color(colors::Aqua);
                break;
            case interaction::resizing_circle_of_rev:
                break;
        }
        render();
        return true;
    }

    if (!mouse_down) {
        interaction_ = interaction::none;
        puck_a_.set_color(colors::White);
        puck_b_.set_color(colors::White);
        render();
        return true;
    }

    return false;
}

bool curvy::impulse_viewer::handle_mouse_move(const std::tuple<int, int>& pix_pt)
{
    if (interaction_ != interaction::none) {
        auto pt = from_scr_coords(pix_pt, logical_sz_, pixel_sz_);
        auto [x, y] = pt;
        switch (interaction_) {
            case interaction::dragging_a:
                puck_a_.set_theta(get_angle_to_pt(puck_a_.center_of_revolution(), pt));
                puck_b_.set_center_of_revolution(puck_a_.position());
                break;
            case interaction::dragging_b: 
                puck_b_.set_theta( get_angle_to_pt(puck_a_.position(), pt) );
                break;
             case interaction::resizing_circle_of_rev: 
                puck_a_.set_radius_of_revolution(euclidean_distance(pt, puck_a_.circle_of_revolution().center()));
                puck_b_.set_center_of_revolution(puck_a_.position());
                break;
        }
        render();
        return true;
    }
    return false;
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

    paint_circle(*g, puck_a_.circle_of_revolution(), colors::Yellow, logical_sz_, pixel_sz_);

    puck_a_.paint(*g, logical_sz_, pixel_sz_);
    puck_b_.paint(*g, logical_sz_, pixel_sz_);

    auto c = circle_through_point(puck_a_.position(), puck_b_.position(), puck_a_.direction());
    paint_circle(*g, c, colors::White, logical_sz_, pixel_sz_);
    //paint_circle(*g, circle_through_point(puck_b_.position()), colors::White, logical_sz_, pixel_sz_);

    delete g;
}

curvy::interaction curvy::impulse_viewer::get_interaction(const std::tuple<double, double>& click_location)
{
    if (puck_a_.contains_point(click_location))
        return interaction::dragging_a;

    if (puck_b_.contains_point(click_location))
        return interaction::dragging_b;

    if (is_pt_on_circle(puck_a_.circle_of_revolution(), click_location, 0.1))
        return interaction::resizing_circle_of_rev;

    return interaction::none;
}
