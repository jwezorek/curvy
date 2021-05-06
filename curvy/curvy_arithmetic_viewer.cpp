#include <cmath>
#include <array>
#include <Windows.h>
#include "curvy_arithmetic_viewer.h"
#include "util.h"
#include "curvy_vector.h"
#include "gdi_util.h"

namespace {
    curvy::point pt_with_lowest_y(const curvy::point& pt1, const curvy::point& pt2) {
        auto y1 = std::get<1>(pt1);
        auto y2 = std::get<1>(pt2);
        return (y1 < y2) ? pt1 : pt2;
   }

    void draw_operation(gdi::Graphics& g, bool add) {
        gdi::FontFamily  fontFamily(L"Arial");
        gdi::Font        font(&fontFamily, 96, gdi::FontStyleRegular, gdi::UnitPixel);
        gdi::PointF      pointF(30.0f, 30.0f);
        gdi::SolidBrush  solidBrush(colors::Yellow);

        auto* txt = (add) ? L"+" : L"-";
        g.DrawString(txt, -1, &font, pointF, &solidBrush);
    }

    curvy::circle handle_move_circle(const curvy::curvy_arithmetic_viewer::move_circle_state& mc, const curvy::point& pt) {
        using namespace curvy;
        auto theta = atan_of_pt( pt) - atan_of_pt(mc.start);
        auto c = apply_matrix( rotation_matrix(theta), mc.c);
        return c;

    }
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
    // translate the curvy vectors so that one of their intersections is at the origin.
    auto [pt1, pt2] = curvy::intersections(vector_a_.circle(), vector_b_.circle()).value();
    auto pt = pt_with_lowest_y(pt1, pt2);
    matrix mat = translation_matrix(-pt);
    vector_a_ = apply_matrix(mat, vector_a_);
    vector_b_ = apply_matrix(mat, vector_b_);
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
        move_circle_ = std::nullopt;
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
            case interaction::moving_circle_a_:
                if (!move_circle_) {
                    move_circle_ = {  pt, vector_a_.circle() };
                }
                vector_a_.set_circle(handle_move_circle(move_circle_.value(), pt));
                break;

            case interaction::moving_circle_b_:
                if (!move_circle_) {
                    move_circle_ = { pt, vector_b_.circle() };
                }
                vector_b_.set_circle(handle_move_circle(move_circle_.value(), pt));
                break;
        }

        render();
        return true;
    }
    return false;
}

bool curvy::curvy_arithmetic_viewer::handle_key_press(unsigned int key, bool is_key_down)
{
    if (key == VK_SPACE) {
        if (is_key_down) {
            addition_ = !addition_;
            return true;
        }
    }

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

    paint_circle_vector(*g, vector_a_, colors::DodgerBlue, 2.0, { 0,0 }, logical_sz_, pixel_sz_);
    paint_circle_vector(*g, vector_b_, colors::Red, 2.0, { 0,0 }, logical_sz_, pixel_sz_);

    auto result = (addition_) ?
        vector_a_.add(vector_b_, { 0,0 }) :
        vector_a_.subtract(vector_b_, { 0,0 });
    paint_circle_vector(*g, result, colors::Purple, 2.0, { 0,0 }, logical_sz_, pixel_sz_);

    draw_operation(*g, addition_);

}

curvy::curvy_arithmetic_viewer::interaction curvy::curvy_arithmetic_viewer::get_interaction(const std::tuple<double, double>& click_location) const
{
    if (vector_a_.circle().perimeter_contains(click_location, 0.1))
        return interaction::moving_circle_a_;

    if (vector_b_.circle().perimeter_contains(click_location, 0.1))
        return interaction::moving_circle_b_;

    return interaction::none;
}

