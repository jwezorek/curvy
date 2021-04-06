#include <Windows.h>
#include <gdiplus.h>
#include "curvy_world_simulation.h"
#include "util.h"
#include <cmath>
#include <string>
#include <map>
#include <limits>

curvy::curvy_world_simulation::curvy_world_simulation(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::curvy_world_simulation::initialize()
{
    insert({
        curvy::circular_vector{  0,  0, 12.0, 4 },
        0,
        colors::Red
    });

    insert({
        curvy::circular_vector{ 0, 0, 12.0, 0 },
        pi(),
        colors::Yellow
    });
}

void curvy::curvy_world_simulation::set_logical_dimensions(double log_sz, bool refresh)
{
    logical_sz_ = log_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(pixel_sz_, pixel_sz_);
        render();
    }
}
bool curvy::curvy_world_simulation::handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down)
{
    return false;
}

bool curvy::curvy_world_simulation::handle_mouse_move(const std::tuple<int, int>& pt)
{
    return false;
}

bool curvy::curvy_world_simulation::handle_key_press(unsigned int key, bool is_key_down)
{
    return false;
}

void curvy::curvy_world_simulation::set_pixel_dimensions(int px_sz, bool refresh) {
    pixel_sz_ = px_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

int curvy::curvy_world_simulation::get_size() const
{
    return pixel_sz_;
}

void curvy::curvy_world_simulation::insert(const puck& p)
{
    pucks_.push_back(p);
    render();
}

gdi::Bitmap* curvy::curvy_world_simulation::get_bitmap() const
{
    return back_buffer_.get();
}

void curvy::curvy_world_simulation::update(double dt)
{
    while (dt > 0) {
        auto [collisions, when] = get_next_collisions(dt, 1e-10);
        for (auto& p : pucks_)
            p.update(when);

        if (!collisions.empty())
            handle_collisions( collisions );

        dt -= when;
    }
    render();
}

void curvy::curvy_world_simulation::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush( colors::Black );
    gdi::Pen white_pen(colors::White, 1);
    auto* g = gdi::Graphics::FromImage(back_buffer_.get());

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);
    g->DrawEllipse(&white_pen, 0, 0, pixel_sz_-1, pixel_sz_-1);

    for (const auto& puck : pucks_)
        paint_puck(*g, puck);

    delete g;
}

void curvy::curvy_world_simulation::paint_puck(gdi::Graphics& g, const puck& p)
{
    p.paint(g, logical_sz_, pixel_sz_);
}

std::tuple<curvy::curvy_world_simulation::collisions, double> curvy::curvy_world_simulation::get_next_collisions(double dt, double eps) {
    std::map<double, collision> collisions;
    int n = static_cast<int>(pucks_.size());

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            puck* p1 = &(pucks_[i]);
            puck* p2 = &(pucks_[j]);
            auto collision_time = p1->get_collision_time(*p2, dt, eps);
            if (collision_time) {
                collisions[*collision_time] = { p1,p2 };
            }
        }
    }

    if (!collisions.empty()) {
        curvy::curvy_world_simulation::collisions collision_set;
        double collision_time = -1;
        for (const auto& [time, pair] : collisions) {
            if (collision_time < 0 || std::abs(collision_time - time) < eps) {
                collision_time = (collision_time < 0) ? time : collision_time;
                collision_set.push_back(pair);
            }
        }
        return { collision_set, collision_time };
    }

    return { curvy::curvy_world_simulation::collisions(), dt };
}

void curvy::curvy_world_simulation::handle_collision( collision& collision) {
    auto [p1, p2] = collision;
    auto tmp = p1->state().signed_angular_magnitude();
    p1->set_speed( p2->state().signed_angular_magnitude());
    p2->set_speed( tmp );
}

void curvy::curvy_world_simulation::handle_collisions( collisions& pairs)
{
    for (auto& pair : pairs)
        handle_collision(pair);
}


