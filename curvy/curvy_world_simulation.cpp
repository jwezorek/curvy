#include <Windows.h>
#include <gdiplus.h>
#include "gdi_util.h"
#include "curvy_world_simulation.h"
#include "util.h"
#include <cmath>
#include <string>
#include <map>
#include <limits>

namespace {

    const double k_cheater_time_delta = 0.02;
    const double k_cheater_eps = 0.00005;
    const bool k_do_cheating = true;
    const int k_cheater_max_stack_depth = 20;

}

curvy::curvy_world_simulation::curvy_world_simulation(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz),
    running_(false)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::curvy_world_simulation::initialize()
{
    pucks_.clear();
    insert({
        curvy::make_curvy_vector(  circle(0, 0, 5), true, 20 ),
        0,
        colors::Red
        });

    insert({
         curvy::make_curvy_vector(circle(0, 0, 5), true,  20 ),
        pi() / 2.0,
        colors::Green
        });

    insert({
         curvy::make_curvy_vector(circle(0, 0, 5), true, 0 ),
        pi(),
        colors::Orange
        });

    insert({
         curvy::make_curvy_vector(circle(0, 0, 5), true, 0 ),
        -pi() / 2.0,
        colors::Blue
        });

    insert({
         curvy::make_curvy_vector(circle(6, 0, 8), true,  11 ),
        -pi() / 4.0,
        colors::Purple
        });
}


/*
void curvy::curvy_world_simulation::initialize()
{
    double s = 2.2;
    pucks_.clear();
    insert({
        0,0,
        colors::Red
        });
    
    insert({
        s * -0.5,  s * -0.866,
        colors::Green
        });

    insert({
        s * 0.5, s * -0.866,
        colors::Orange
        });
    
    insert({
        s * -1, s * 2 * -0.866,
        colors::Blue
        });

    insert({
        0, s * 2 * -0.866,
        colors::Purple
        });
    
    insert({
        s * 1, s * 2 * -0.866,
        colors::DeepPink
        });

    insert({
        curvy::make_curvy_vector( circle(7.8, 0, 8), true, 75 ),
        3.0 * pi() / 2.0 - pi()/6.0,
        colors::White
        });

}
*/

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
    if (is_key_down && key == VK_SPACE) {
        running_ = !running_;
        return true;
    }
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

void curvy::curvy_world_simulation::update()
{
    render();
}

void curvy::curvy_world_simulation::update(double dt)
{
    if (!running_) {
        return;
    }

    for (auto& p : pucks_) {
        p.update_contact_list();
    }

   // for (auto& p : pucks_)
   //    p.apply_friction( dt );

    while (dt > 0) {
        auto [collisions, when] = get_next_collisions(dt, eps() );
        for (auto& p : pucks_)
            p.update(when);

        if (!collisions.empty())
            handle_collisions( collisions );

        dt -= when;
    }

    render();
}

curvy::circle curvy::curvy_world_simulation::border() const
{
    return curvy::circle(0, 0, logical_sz_ / 2.0);
}

void curvy::curvy_world_simulation::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush(get_background_color());
    gdi::Pen white_pen(get_foreground_color(), 1);
    auto* g = gdi::Graphics::FromImage(back_buffer_.get());

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);

    auto [x1, y1, x2, y2] = curvy::to_scr_coords(
        border().bounding_box(),
        logical_sz_, pixel_sz_
    ); 
    g->DrawEllipse(&white_pen, x1, y1, x2-x1, y2-y1);

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
        puck* p1 = &(pucks_[i]);

        auto collision_time = p1->get_boundary_collision_time(border(), dt, eps);
        if (collision_time) {
            collisions[*collision_time] = { p1, nullptr };
        }

        for (int j = i + 1; j < n; ++j) {
            puck* p2 = &(pucks_[j]);

            if (p1->is_in_contact_list(*p2))
                continue;

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

bool intersects_at_time(double t, const curvy::curvy_vector& a, const curvy::point& pt_a, const curvy::curvy_vector& b, const curvy::point& pt_b, double sz_constant) {
    using namespace curvy;

    auto initial_theta_a = angle_to_pt(a.circle().center(), pt_a);
    auto theta_a = initial_theta_a + *a.signed_angular_magnitude() * t;
    auto new_pt_a = a.circle().center() + a.circle().radius() * pt_on_unit_circle(theta_a);

    auto initial_theta_b = curvy::angle_to_pt(b.circle().center(), pt_b);
    auto theta_b = initial_theta_b + *b.signed_angular_magnitude() * t;
    auto new_pt_b = b.circle().center() + b.circle().radius() * pt_on_unit_circle(theta_b);

    return euclidean_distance(new_pt_a, new_pt_b) < sz_constant;
}

double distance_at_time(double t, const curvy::curvy_vector& a, const curvy::point& pt_a, const curvy::curvy_vector& b, const curvy::point& pt_b) {
    using namespace curvy;

    auto initial_theta_a = angle_to_pt(a.circle().center(), pt_a);
    auto theta_a = initial_theta_a + *a.signed_angular_magnitude() * t;
    auto new_pt_a = a.circle().center() + a.circle().radius() * pt_on_unit_circle(theta_a);

    auto initial_theta_b = curvy::angle_to_pt(b.circle().center(), pt_b);
    auto theta_b = initial_theta_b + *b.signed_angular_magnitude() * t;
    auto new_pt_b = b.circle().center() + b.circle().radius() * pt_on_unit_circle(theta_b);

    return euclidean_distance(new_pt_a, new_pt_b);
}

std::tuple<double, double> find_nonintersecting_collision_resolution_aux(const curvy::circle& a_to_b_circle, bool a_to_b_orientation, double initial_a_to_b, const curvy::curvy_vector& a, const curvy::point& pt_a,
    const curvy::circle& b_to_a_circle, bool b_to_a_orientation, double initial_b_to_a, const curvy::curvy_vector& b, const curvy::point& pt_b, double sz_constant, double eps, double t1, double t2, int stack_depth, double time_delta) {

    auto t = (t1 + t2) / 2.0;

    auto coefficient_a_to_b = initial_a_to_b + t * (1.0 - initial_a_to_b);
    auto coefficient_b_to_a = initial_b_to_a + t * (1.0 - initial_b_to_a);

    if (stack_depth > k_cheater_max_stack_depth)
        return { coefficient_a_to_b, coefficient_b_to_a };

    auto impulse_a_to_b = curvy::make_curvy_vector(a_to_b_circle, a_to_b_orientation, coefficient_a_to_b * a.linear_magnitude());
    auto residual_vector_a_to_b = a.subtract(impulse_a_to_b, pt_a);

    auto impulse_b_to_a = curvy::make_curvy_vector(b_to_a_circle, b_to_a_orientation, coefficient_b_to_a * b.linear_magnitude());
    auto residual_vector_b_to_a = b.subtract(impulse_b_to_a, pt_b);

    auto final_a = residual_vector_a_to_b.add(impulse_b_to_a, pt_a);
    auto final_b = residual_vector_b_to_a.add(impulse_a_to_b, pt_b);

    auto dist_at_t = distance_at_time(time_delta, final_a, pt_a, final_b, pt_b);
    auto delta = sz_constant - dist_at_t;

    if (std::abs(delta) <= eps)
        return { coefficient_a_to_b, coefficient_b_to_a };

    if (delta > 0)
        return find_nonintersecting_collision_resolution_aux(a_to_b_circle, a_to_b_orientation, initial_a_to_b, a, pt_a,
            b_to_a_circle, b_to_a_orientation, initial_b_to_a, b, pt_b, sz_constant, eps, t, t2, stack_depth+1, time_delta);
  
    return find_nonintersecting_collision_resolution_aux(a_to_b_circle, a_to_b_orientation, initial_a_to_b, a, pt_a,
        b_to_a_circle, b_to_a_orientation, initial_b_to_a, b, pt_b, sz_constant, eps, t1, t, stack_depth + 1, time_delta);
}

std::tuple<curvy::curvy_vector, curvy::curvy_vector> find_nonintersecting_collision_resolution(const curvy::circle& a_to_b_circle, bool a_to_b_orientation, double coefficient_a_to_b, const curvy::curvy_vector& a, const curvy::point& pt_a,
        const curvy::circle& b_to_a_circle, bool b_to_a_orientation, double coefficient_b_to_a, const curvy::curvy_vector& b, const curvy::point& pt_b, double sz_constant, double eps, double time_delta) {

    auto impulse_a_to_b = curvy::make_curvy_vector(a_to_b_circle, a_to_b_orientation, coefficient_a_to_b * a.linear_magnitude());
    auto residual_vector_a_to_b = a.subtract(impulse_a_to_b, pt_a);

    auto impulse_b_to_a = curvy::make_curvy_vector(b_to_a_circle, b_to_a_orientation, coefficient_b_to_a * b.linear_magnitude());
    auto residual_vector_b_to_a = b.subtract(impulse_b_to_a, pt_b);

    auto final_a = residual_vector_a_to_b.add(impulse_b_to_a, pt_a);
    auto final_b = residual_vector_b_to_a.add(impulse_a_to_b, pt_b);

    if (k_do_cheating && intersects_at_time(k_cheater_time_delta, final_a, pt_a, final_b, pt_b, sz_constant)) {
        auto [revised_coefficient_a_to_b, revised_coefficient_b_to_a] = find_nonintersecting_collision_resolution_aux(a_to_b_circle, a_to_b_orientation, coefficient_a_to_b, a, pt_a,
            b_to_a_circle, b_to_a_orientation, coefficient_b_to_a, b, pt_b, sz_constant, eps, 0.0, 1.0, 1, time_delta);
        impulse_a_to_b = curvy::make_curvy_vector(a_to_b_circle, a_to_b_orientation, revised_coefficient_a_to_b * a.linear_magnitude());
        residual_vector_a_to_b = a.subtract(impulse_a_to_b, pt_a);

        impulse_b_to_a = curvy::make_curvy_vector(b_to_a_circle, b_to_a_orientation, revised_coefficient_b_to_a * b.linear_magnitude());
        residual_vector_b_to_a = b.subtract(impulse_b_to_a, pt_b);

        final_a = residual_vector_a_to_b.add(impulse_b_to_a, pt_a);
        final_b = residual_vector_b_to_a.add(impulse_a_to_b, pt_b);
    }

    return { final_a, final_b };
}

void curvy::curvy_world_simulation::handle_collision( collision& collision) {
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
    auto [b_to_a_circle, b_to_a_orientation] = curvy::circular_direction_through_two_points(pt_b, direction_b, pt_a);
    auto coefficient_a_to_b = momentum_transfer_factor(pt_a, direction_a, orientation_a, pt_b, radius_a, sz_constant);
    auto coefficient_b_to_a = momentum_transfer_factor(pt_b, direction_b, orientation_b, pt_a, radius_b, sz_constant);

    auto [final_a, final_b] = find_nonintersecting_collision_resolution(a_to_b_circle, a_to_b_orientation, coefficient_a_to_b, a, pt_a,
        b_to_a_circle, b_to_a_orientation, coefficient_b_to_a, b, pt_b, sz_constant, k_cheater_eps, k_cheater_time_delta);

    puck_a.set_vector(final_a);
    puck_b.set_vector(final_b);

    if (is_in_contact_or_intersecting(puck_a, puck_b)) {
        puck_a.add_to_contact_list( puck_b );
        puck_b.add_to_contact_list( puck_a );
    }
}

void curvy::curvy_world_simulation::handle_boundary_collision(puck* p)
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
            p->state().orientation(),
            lin_mag,
            p->state().weight()
        )
    );
    p->set_theta(angle_to_pt(circle_of_rev.center(), position));
}

void curvy::curvy_world_simulation::handle_collisions( collisions& pairs)
{
    for (auto& pair : pairs)
        handle_collision(pair);
}


