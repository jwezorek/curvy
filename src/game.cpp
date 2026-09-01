#define NOMINMAX
#include <Windows.h>
#include <gdiplus.h>
#include "game.h"
#include "util.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <map>
#include <limits>

curvy::game::game(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz),
    running_(false)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}


void curvy::game::initialize()
{
    double s = 2.2;
    pucks_.clear();
    insert({
        0,0,
        colors::Red
        });

    insert({
        curvy::curvy_vector{ 8, 0, 8, 78 },
        3.0 * pi() / 2.0 - pi() / 6.0,
        colors::White
        });

}

void curvy::game::set_logical_dimensions(double log_sz, bool refresh)
{
    logical_sz_ = log_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(pixel_sz_, pixel_sz_);
        render();
    }
}
bool curvy::game::handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down)
{
    return false;
}

bool curvy::game::handle_mouse_move(const std::tuple<int, int>& pt)
{
    return false;
}

bool curvy::game::handle_key_press(unsigned int key, bool is_key_down)
{
    if (is_key_down && key == VK_SPACE) {
        running_ = !running_;
        return true;
    }
    return false;
}

void curvy::game::set_pixel_dimensions(int px_sz, bool refresh) {
    pixel_sz_ = px_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

int curvy::game::get_size() const
{
    return pixel_sz_;
}

void curvy::game::insert(const puck& p)
{
    pucks_.push_back(p);
    render();
}

gdi::Bitmap* curvy::game::get_bitmap() const
{
    return back_buffer_.get();
}

void curvy::game::update()
{
    render();
}

void curvy::game::update(double dt)
{
    if (!running_) {
        return;
    }

    for (auto& p : pucks_) {
        p.update_contact_list();
    }

    for (auto& p : pucks_)
        p.apply_friction(dt);

    int consecutive_zero_time_events = 0;
    while (dt > 0) {
        auto [collisions, when] = get_next_collisions(dt, eps());
        auto made_no_progress =
            when <= std::numeric_limits<double>::epsilon() * std::max(1.0, dt);
        if (made_no_progress && consecutive_zero_time_events > 0)
            break;

        for (auto& p : pucks_)
            p.update(when);

        if (!collisions.empty())
            handle_collisions(collisions);

        consecutive_zero_time_events = made_no_progress ? 1 : 0;

        dt -= when;
    }

    render();
}

curvy::circle curvy::game::border() const
{
    return curvy::circle(0, 0, logical_sz_ / 2.0);
}

void curvy::game::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush(colors::Black);
    gdi::Pen white_pen(colors::White, 1);
    auto* g = gdi::Graphics::FromImage(back_buffer_.get());

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);

    auto [x1, y1, x2, y2] = curvy::to_scr_coords(
        border().bounding_box(),
        logical_sz_, pixel_sz_
    );
    g->DrawEllipse(&white_pen, x1, y1, x2 - x1, y2 - y1);

    for (const auto& puck : pucks_)
        paint_puck(*g, puck);

    delete g;
}

void curvy::game::paint_puck(gdi::Graphics& g, const puck& p)
{
    p.paint(g, logical_sz_, pixel_sz_);
}

std::tuple<curvy::game::collisions, double> curvy::game::get_next_collisions(double dt, double eps) {
    std::multimap<double, collision> collisions;
    int n = static_cast<int>(pucks_.size());

    for (int i = 0; i < n; ++i) {
        puck* p1 = &(pucks_[i]);

        auto collision_time = p1->get_boundary_collision_time(border(), dt, eps);
        if (collision_time) {
            collisions.emplace(*collision_time, collision{ p1, nullptr });
        }

        for (int j = i + 1; j < n; ++j) {
            puck* p2 = &(pucks_[j]);

            if (p1->is_in_contact_list(*p2))
                continue;

            auto collision_time = p1->get_collision_time(*p2, dt, eps);
            if (collision_time) {
                collisions.emplace(*collision_time, collision{ p1,p2 });
            }
        }
    }

    if (!collisions.empty()) {
        curvy::game::collisions collision_set;
        double collision_time = -1;
        for (const auto& [time, pair] : collisions) {
            if (collision_time < 0 || std::abs(collision_time - time) < eps) {
                collision_time = (collision_time < 0) ? time : collision_time;
                collision_set.push_back(pair);
            }
        }
        return { collision_set, collision_time };
    }

    return { curvy::game::collisions(), dt };
}

void curvy::game::handle_collision(collision& collision) {
    auto [p1, p2] = collision;
    if (!p2) {
        handle_boundary_collision(p1);
        return;
    }

    if (p1->is_in_contact_list(*p2))
        return;

    auto& puck_a = *p1;
    auto& puck_b = *p2;

    double sz_constant = puck_a.puck_circle().radius() + puck_b.puck_circle().radius();
    const auto& a = puck_a.state();
    const auto& b = puck_b.state();
    auto pt_a = puck_a.position();
    auto pt_b = puck_b.position();
    auto orientation_a = a.orientation();
    auto orientation_b = b.orientation();
    auto radius_a = puck_a.state().circle().radius();
    auto radius_b = puck_b.state().circle().radius();
    auto direction_a = puck_a.direction();
    auto direction_b = puck_b.direction();

    auto [a_to_b_circle, a_to_b_orientation] = curvy::circular_direction_through_two_points(pt_a, direction_a, pt_b);
    auto coefficient_a_to_b = momentum_transfer_factor(pt_a, direction_a, orientation_a, pt_b, radius_a, sz_constant);
    auto impulse_a_to_b = curvy::curvy_vector(a_to_b_circle, (a_to_b_orientation ? 1.0 : -1.0) * coefficient_a_to_b * a.linear_magnitude());
    auto residual_vector_a_to_b = a.subtract(impulse_a_to_b, pt_a);

    auto [b_to_a_circle, b_to_a_orientation] = curvy::circular_direction_through_two_points(pt_b, direction_b, pt_a);
    auto coefficient_b_to_a = momentum_transfer_factor(pt_b, direction_b, orientation_b, pt_a, radius_b, sz_constant);
    auto impulse_b_to_a = curvy::curvy_vector(b_to_a_circle, (b_to_a_orientation ? 1.0 : -1.0) * coefficient_b_to_a * b.linear_magnitude());
    auto residual_vector_b_to_a = b.subtract(impulse_b_to_a, pt_b);

    auto final_a = residual_vector_a_to_b.add(impulse_b_to_a, pt_a);
    auto final_b = residual_vector_b_to_a.add(impulse_a_to_b, pt_b);

    puck_a.set_vector(final_a);
    puck_b.set_vector(final_b);

    if (is_in_contact_or_intersecting(puck_a, puck_b)) {
        puck_a.add_to_contact_list(puck_b);
        puck_b.add_to_contact_list(puck_a);
    }
}

void curvy::game::handle_boundary_collision(puck* p)
{
    auto position = p->position();
    auto circle_of_rev = p->state().circle();
    auto lin_mag = p->state().linear_magnitude();
    auto theta = atan_of_pt(position);

    matrix reflect = rotation_matrix(theta) * scale_matrix(1, -1) * rotation_matrix(-theta);
    circle_of_rev = apply_matrix(reflect, circle_of_rev);

    p->set_vector(
        curvy_vector(
            circle_of_rev,
            lin_mag
        )
    );
    p->set_theta(angle_to_pt(circle_of_rev.center(), position));
}

void curvy::game::handle_collisions(collisions& pairs)
{
    for (auto& pair : pairs)
        handle_collision(pair);
}
