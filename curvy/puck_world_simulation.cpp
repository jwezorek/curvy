#include <Windows.h>
#include <gdiplus.h>
#include "puck_world_simulation.h"
#include "puck.h"
#include "util.h"
#include <cmath>
#include <string>
#include <map>
#include <limits>

curvy::puck_world_simulation::puck_world_simulation(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::puck_world_simulation::initialize()
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

void curvy::puck_world_simulation::set_logical_dimensions(double log_sz, bool refresh)
{
    logical_sz_ = log_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(pixel_sz_, pixel_sz_);
        render();
    }
}
void curvy::puck_world_simulation::set_pixel_dimensions(int px_sz, bool refresh) {
    pixel_sz_ = px_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

int curvy::puck_world_simulation::get_size() const
{
    return pixel_sz_;
}

void curvy::puck_world_simulation::insert(const puck& p)
{
    pucks_.push_back(p);
    render();
}

gdi::Bitmap* curvy::puck_world_simulation::get_bitmap() const
{
    return back_buffer_.get();
}

void curvy::puck_world_simulation::update(double dt)
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

void curvy::puck_world_simulation::render()
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

void curvy::puck_world_simulation::paint_puck(gdi::Graphics& g, const puck& p)
{
    
    gdi::SolidBrush brush( p.color() );
    auto [x1, y1, x2, y2] = get_location_in_pixels(p);
    g.FillEllipse(&brush, x1, y1, x2 - x1, y2 - y1);

}

std::tuple<int, int, int, int> curvy::puck_world_simulation::to_scr_coords(double x1, double y1, double x2, double y2) const
{
    y1 *= -1;
    y2 *= -1;

    x1 += logical_sz_ / 2;
    y1 += logical_sz_ / 2;
    x2 += logical_sz_ / 2;
    y2 += logical_sz_ / 2;

    double log_to_scr = pixel_sz_ / logical_sz_;

    x1 *= log_to_scr;
    y1 *= log_to_scr;
    x2 *= log_to_scr;
    y2 *= log_to_scr;

    return std::tuple<int, int, int, int>(
        static_cast<int>(std::round(x1)), 
        static_cast<int>(std::round(y1)),
        static_cast<int>(std::round(x2)),
        static_cast<int>(std::round(y2))
    );
}

std::tuple<int, int, int, int> curvy::puck_world_simulation::get_location_in_pixels(const puck& p) const
{
    auto [cx, cy] = p.center_of_revolution();
    auto x = cx + p.radius_of_revolution() * std::cos(p.theta());
    auto y = cy + p.radius_of_revolution() * std::sin(p.theta());

    auto x1 = x - p.radius();
    auto y1 = y - p.radius();
    auto x2 = x + p.radius();
    auto y2 = y + p.radius();

    return to_scr_coords(x1, y1, x2, y2);
}

std::tuple<curvy::puck_world_simulation::collisions, double> curvy::puck_world_simulation::get_next_collisions(double dt, double eps) {
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
        curvy::puck_world_simulation::collisions collision_set;
        double collision_time = -1;
        for (const auto& [time, pair] : collisions) {
            if (collision_time < 0 || std::abs(collision_time - time) < eps) {
                collision_time = (collision_time < 0) ? time : collision_time;
                collision_set.push_back(pair);
            }
        }
        return { collision_set, collision_time };
    }

    return { curvy::puck_world_simulation::collisions(), dt };
}

void curvy::puck_world_simulation::handle_collision( collision& collision) {
    auto [p1, p2] = collision;
    auto tmp = p1->angular_speed();
    p1->set_speed( p2->angular_speed() );
    p2->set_speed( tmp );
}

void curvy::puck_world_simulation::handle_collisions( collisions& pairs)
{
    for (auto& pair : pairs)
        handle_collision(pair);
}


