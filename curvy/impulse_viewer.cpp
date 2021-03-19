#include <Windows.h>
#include <cmath>
#include "impulse_viewer.h"
#include "util.h"

namespace {

    gdi::Point to_scr_point(const point& p, double log_sz, int pix_sz) {
        auto [x, y] = to_scr_coords(p, log_sz, pix_sz);
        return { x,y };
    }

    void paint_triangle(gdi::Graphics& g, gdi::Color color, const point& p1, const point& p2, const point& p3, double log_sz, int pix_sz) {
        gdi::Point ary[3] = {
            to_scr_point(p1, log_sz, pix_sz),
            to_scr_point(p2, log_sz, pix_sz),
            to_scr_point(p3, log_sz, pix_sz)
        };
        gdi::SolidBrush brush(color);
        g.FillPolygon(&brush, &(ary[0]), 3);
    }

    void paint_arrow_head(gdi::Graphics& g, gdi::Color color, const point& loc, double wd, double hgt, double direction, double log_sz, int pix_sz) {
        point pts[3] = {
            {wd, 0},
            {-wd, hgt},
            {-wd, -hgt}
        };
        matrix mat = translation_matrix(loc) * rotation_matrix(direction) ;
        for (auto& pt : pts) 
            pt = apply_matrix(mat, pt);
        paint_triangle(g, color, pts[0], pts[1], pts[2], log_sz, pix_sz);
    }

    gdi::Rect to_scr_rect(const std::tuple<double, double, double, double>& r, double log_sz, int pix_sz) {
        auto [x1, y1, x2, y2] = to_scr_coords(r, log_sz, pix_sz);
        auto x = (x1 < x2) ? x1 : x2;
        auto y = (y1 < y2) ? y1 : y2;
        return gdi::Rect(x, y, std::abs(x1 - x2), std::abs(y1 - y2));
    }

    void paint_circle(gdi::Graphics& g, const curvy::circle& c, gdi::Color color, double log_sz, int pix_sz) {
        gdi::Pen pen(color, 3);
        g.DrawEllipse(&pen, to_scr_rect( c.bounding_box(), log_sz, pix_sz));
    }

    double to_degrees(double radians) {
        return radians * 180.0 / pi();
    }

    void paint_arc_arrow(gdi::Graphics& g, const curvy::circle& c, double theta1, double theta2, gdi::Color color, double log_sz, int pix_sz) {
        gdi::Pen pen(color, 3);
        g.DrawArc(&pen, to_scr_rect(c.bounding_box(), log_sz, pix_sz), -to_degrees(theta1), -to_degrees(theta2));
        point pt = {c.x + c.r * std::cos(theta1+theta2), c.y + c.r * std::sin(theta1 + theta2) };
        paint_arrow_head(g, color, pt, 0.5, 0.25, theta1 + theta2 + pi() / 2.0, log_sz, pix_sz);
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
    puck_a_.set_speed(1.0);
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
    paint_arc_arrow(*g, puck_a_.circle_of_revolution(), puck_a_.theta(),  puck_a_.angular_speed(), colors::Red, logical_sz_, pixel_sz_);
    puck_a_.paint(*g, logical_sz_, pixel_sz_);
    puck_b_.paint(*g, logical_sz_, pixel_sz_);

    auto c = circle_through_point(puck_a_.position(), puck_b_.position(), puck_a_.direction());
    paint_circle(*g, c, colors::White, logical_sz_, pixel_sz_);



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
