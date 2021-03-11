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
        curvy::circle_rotation_state{  0, 0,  0, 12.0, 4 },
        colors::Red
    });

    insert({
        curvy::circle_rotation_state{ pi(),  0, 0, 12.0, 0 },
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
    gdi::SolidBrush brush( p.color() );
    auto [x1, y1, x2, y2] = get_location_in_pixels(p);
    g.FillEllipse(&brush, x1, y1, x2 - x1, y2 - y1);
}

std::tuple<int, int, int, int> curvy::curvy_world_simulation::get_location_in_pixels(const puck& p) const
{
    return p.get_location_in_pixels(logical_sz_, pixel_sz_);
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
    auto tmp = p1->angular_speed();
    p1->set_speed( p2->angular_speed() );
    p2->set_speed( tmp );
}

void curvy::curvy_world_simulation::handle_collisions( collisions& pairs)
{
    for (auto& pair : pairs)
        handle_collision(pair);
}


