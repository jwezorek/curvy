#include <cmath>
#include <array>
#include <Windows.h>
#include "curvy_vector_viewer.h"
#include "util.h"
#include "curvy_vector.h"

namespace {

    gdi::Point to_scr_point(const curvy::point& p, double log_sz, int pix_sz) {
        auto [x, y] = curvy::to_scr_coords(p, log_sz, pix_sz);
        return { x,y };
    }

    void paint_line_segment(gdi::Graphics& g, gdi::Color color, const curvy::point& p1, const curvy::point& p2, double log_sz, int pix_sz) {
        auto [x1, y1] = to_scr_point(p1, log_sz, pix_sz);
        auto [x2, y2] = to_scr_point(p2, log_sz, pix_sz);
        gdi::Pen pen(color, 3);
        g.DrawLine(&pen, x1, y1, x2, y2);
    }

    void paint_triangle(gdi::Graphics& g, gdi::Color color, const curvy::point& p1, const curvy::point& p2, const curvy::point& p3, double log_sz, int pix_sz) {
        gdi::Point ary[3] = {
            to_scr_point(p1, log_sz, pix_sz),
            to_scr_point(p2, log_sz, pix_sz),
            to_scr_point(p3, log_sz, pix_sz)
        };
        gdi::SolidBrush brush(color);
        g.FillPolygon(&brush, &(ary[0]), 3);
    }

    std::array<curvy::point, 3> get_arrow_poly(const curvy::point& loc, double wd, double hgt, double direction) {
        std::array<curvy::point, 3> pts = {
            curvy::point{wd, 0},
            curvy::point{-wd, hgt},
            curvy::point{-wd, -hgt}
        };
        curvy::matrix mat = curvy::translation_matrix(loc) * curvy::rotation_matrix(direction);
        for (auto& pt : pts)
            pt = curvy::apply_matrix(mat, pt);

        return pts;
    }

    gdi::Rect to_scr_rect(const std::tuple<double, double, double, double>& r, double log_sz, int pix_sz) {
        auto [x1, y1, x2, y2] = curvy::to_scr_coords(r, log_sz, pix_sz);
        auto x = (x1 < x2) ? x1 : x2;
        auto y = (y1 < y2) ? y1 : y2;
        return gdi::Rect(x, y, std::abs(x1 - x2), std::abs(y1 - y2));
    }

    void paint_circle(gdi::Graphics& g, const curvy::circle& c, gdi::Color color, double log_sz, int pix_sz) {
        gdi::Pen pen(color, 3);
        g.DrawEllipse(&pen, to_scr_rect( c.bounding_box(), log_sz, pix_sz));
    }

    gdi::REAL to_degrees(double radians) {
        return static_cast<gdi::REAL>( radians * 180.0 / curvy::pi() );
    }

    std::array<curvy::point, 3> arrow_poly_from_circle_vec(const curvy::curvy_vector& crc, const curvy::point& pt, double puck_sz) {
        auto theta = curvy::normalize_angle(curvy::angle_to_pt(crc.circle().center(), pt));
        auto arrow_theta = theta + crc.signed_angular_magnitude();
        auto arror_direction = curvy::direction_on_circle(arrow_theta, crc.orientation());
        curvy::point arrow_pt = {
            crc.circle().x() + crc.circle().radius() * std::cos(arrow_theta),
            crc.circle().y() + crc.circle().radius() * std::sin(arrow_theta)
        };
        auto pts = get_arrow_poly(arrow_pt, puck_sz * 0.25, puck_sz * 0.125, arror_direction);
        return pts;
    }

    void paint_arc_arrow(gdi::Graphics& g, const curvy::curvy_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz) {
        auto theta = curvy::normalize_angle(curvy::angle_to_pt(crc.circle().center(), pt));
        gdi::Pen pen(color, 3);
        g.DrawArc(&pen, to_scr_rect(crc.circle().bounding_box(), log_sz, pix_sz), -to_degrees(theta), -to_degrees(crc.signed_angular_magnitude()));
        auto pts = arrow_poly_from_circle_vec(crc, pt, puck_sz);
        paint_triangle(g, color, pts[0], pts[1], pts[2], log_sz, pix_sz);
    }

    void paint_circle_vector(gdi::Graphics& g, const curvy::curvy_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz) {
        paint_circle(g, crc.circle(), gdi::Color(100, color.GetR(), color.GetG(), color.GetB()), log_sz, pix_sz);
        if (crc.angular_magnitude() != 0)
            paint_arc_arrow(g, crc, color, puck_sz, pt, log_sz, pix_sz);
    }
}

curvy::curvy_vector_viewer::curvy_vector_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz),
    interaction_(interaction::none),
    b_speed_(0),
    puck_b_theta_(0),
    show_puck_b_vectors_(false)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void curvy::curvy_vector_viewer::initialize()
{
    auto r = 0.35;
    const auto south = 3.0 * pi() / 2.0;

    puck_a_.set_circle_rotation_position( south, 0, 3, 3 );
    puck_a_.set_speed(1.0);
    puck_a_.set_color(colors::Red);
    puck_a_.set_puck_radius(r);

    puck_b_.set_color(colors::DodgerBlue);
    puck_b_.set_puck_radius(r);
    puck_b_.set_circle_rotation_position(south, 0, 3, 3);
    puck_b_.set_speed(1.0);

    sync_b_with_a();
}

void curvy::curvy_vector_viewer::update()
{
    render();
}

void curvy::curvy_vector_viewer::update(double dt)
{
    if (b_speed_ != 0) {
        double old_a_theta = puck_a_.theta();
        puck_b_theta_ += b_speed_ * dt;
        sync_b_with_a(old_a_theta);
        render();
    }
}

bool curvy::curvy_vector_viewer::handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down)
{
    auto loc = from_scr_coords(pt, logical_sz_, pixel_sz_);

    if (interaction_ == interaction::none && mouse_down) {
        interaction_ = get_interaction(loc);
        if (interaction_ == interaction::none)
            return false;
        switch (interaction_) {
            case interaction::dragging_circle_of_rev:
                break;
            case interaction::dragging_a:
                break;
            case interaction::dragging_b:
                break;
            case interaction::resizing_circle_of_rev:
                break;
        }
        render();
        return true;
    }

    if (!mouse_down && interaction_!= interaction::none) {
        interaction_ = interaction::none;
        render();
        return true;
    }

    return false;
}

bool curvy::curvy_vector_viewer::handle_mouse_move(const std::tuple<int, int>& pix_pt)
{
    if (interaction_ != interaction::none) {
        auto circle_of_rev = puck_a_.state().circle();
        auto pt = from_scr_coords(pix_pt, logical_sz_, pixel_sz_);
        auto [x, y] = pt;
        double old_a_theta = puck_a_.theta();
        switch (interaction_) {
            case interaction::dragging_circle_of_rev:
                puck_a_.set_center_of_revolution(pt);
                sync_b_with_a(old_a_theta);
                break;
            case interaction::dragging_a:
                puck_a_.set_theta(angle_to_pt(circle_of_rev.center(), pt));
                sync_b_with_a(old_a_theta);
                break;
            case interaction::dragging_b: 
                puck_b_theta_ = angle_to_pt(puck_a_.position(), pt);
                sync_b_with_a(old_a_theta);
                break;
             case interaction::resizing_circle_of_rev:
                puck_a_.set_radius_of_revolution(euclidean_distance(pt, circle_of_rev.center()));
                sync_b_with_a(old_a_theta);
                break;
             case interaction::dragging_circle_b:
                rotate_circle_b(pt);
                break;
             case interaction::resizing_circle_b:
                resize_circle_b(pt);
                break;
             case interaction::dragging_arrow_b:
                drag_arrow_b(pt);
                break;
        }
        render();
        return true;
    }
    return false;
}

bool curvy::curvy_vector_viewer::handle_key_press(unsigned int key, bool is_key_down)
{
    if (key == VK_INSERT) {
        if (is_key_down) {
            b_speed_ = (b_speed_ == 0) ? 0.15 : 0;
            return true;
        } 
    }

    if (key == VK_SPACE) {
        if (is_key_down) {
            b_speed_ *= -1;
            return true;
        }
    }

    if (key == 'B' && is_key_down) {
        show_puck_b_vectors_ = !show_puck_b_vectors_;
        return true;
    }

    return false;
}

void curvy::curvy_vector_viewer::set_logical_dimensions(double log_sz, bool refresh)
{
    logical_sz_ = log_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(pixel_sz_, pixel_sz_);
        render();
    }
}
void curvy::curvy_vector_viewer::set_pixel_dimensions(int px_sz, bool refresh) {
    pixel_sz_ = px_sz;
    if (logical_sz_ && pixel_sz_ && refresh) {
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

gdi::Bitmap* curvy::curvy_vector_viewer::get_bitmap() const
{
    return back_buffer_.get();
}

std::array<double, 4> get_four_levels(double bottom) {
    auto delta = (1.0 - bottom) / 3.0;
    return {
        bottom,
        bottom + delta,
        bottom + 2 * delta,
        1.0
    };
}

std::tuple< gdi::Color, gdi::Color, gdi::Color, gdi::Color> get_four_color_levels(gdi::Color color) {
    double red = static_cast<double>( color.GetR() );
    double green = static_cast<double>(color.GetG() );
    double blue = static_cast<double>(color.GetB() );
    auto levels = get_four_levels(0.35);
    auto level_to_color = [&](int n)->gdi::Color {
        BYTE rb = static_cast<BYTE>(std::round(levels[n] * red));
        BYTE gb = static_cast<BYTE>(std::round(levels[n] * green));
        BYTE bb = static_cast<BYTE>(std::round(levels[n] * blue));
        return gdi::Color(rb, gb, bb);
    };
    return {
        level_to_color(0),
        level_to_color(1),
        level_to_color(2),
        level_to_color(3)
    };
}

struct collision_vectors {
    curvy::curvy_vector initial;
    curvy::curvy_vector residual;
    curvy::curvy_vector received;
    curvy::curvy_vector final;

    void paint(gdi::Graphics& g, gdi::Color color, const curvy::point& pt, double sz_const, double log_sz, int pix_sz) {
        auto [initial_color, residual_color, received_color, final_color] = get_four_color_levels(color);

        if (initial.angular_magnitude() > 0)
            paint_circle_vector(g, initial, initial_color, sz_const, pt, log_sz, pix_sz);

        if (residual.angular_magnitude() > 0)
            paint_circle_vector(g, residual, residual_color, sz_const, pt, log_sz, pix_sz);

        if (received.angular_magnitude() > 0)
            paint_circle_vector(g, received, received_color, sz_const, pt, log_sz, pix_sz);

        if (final.angular_magnitude() > 0)
            paint_circle_vector(g, final, final_color, sz_const, pt, log_sz, pix_sz);
    }

};

std::tuple<collision_vectors, collision_vectors> get_collision_vectors(bool b_is_moving, const curvy::puck& puck_a, const curvy::puck& puck_b) {
    static const auto nil = curvy::curvy_vector();

    double sz_constant = puck_a.puck_circle().radius() + puck_b.puck_circle().radius();
    const auto& a = puck_a.state();
    const auto& b = (b_is_moving ? 1.0 : 0.0) * puck_b.state();
    auto pt_a = puck_a.position();
    auto pt_b = puck_b.position();
    auto orientation_a = a.orientation();
    auto orientation_b = b.orientation();
    auto radius_a = puck_a.state().circle().radius();
    auto radius_b = puck_b.state().circle().radius();
    auto direction_a = puck_a.direction();
    auto direction_b = puck_b.direction();

    auto [a_to_b_circle, a_to_b_orientation] = curvy::circular_direction_through_two_points(pt_a, direction_a, pt_b);
    auto coefficient_a_to_b = curvy::momentum_transfer_factor(pt_a, direction_a, orientation_a,  pt_b, radius_a,  sz_constant);
    auto impulse_a_to_b = curvy::circular_vector_from_linear_magnitude(a_to_b_circle, (a_to_b_orientation ? 1.0 : -1.0) * coefficient_a_to_b * a.linear_magnitude());
    auto residual_vector_a_to_b = a.subtract(impulse_a_to_b, pt_a);

    auto [b_to_a_circle, b_to_a_orientation] = curvy::circular_direction_through_two_points(pt_b, direction_b, pt_a);
    auto coefficient_b_to_a = curvy::momentum_transfer_factor(pt_b, direction_b, orientation_b, pt_a, radius_b,  sz_constant);
    auto impulse_b_to_a = curvy::circular_vector_from_linear_magnitude(b_to_a_circle, (b_to_a_orientation ? 1.0 : -1.0) * coefficient_b_to_a * b.linear_magnitude());
    auto residual_vector_b_to_a = b.subtract(impulse_b_to_a, pt_b);

    auto final_a = residual_vector_a_to_b.add(impulse_b_to_a, pt_a);
    auto final_b = residual_vector_b_to_a.add(impulse_a_to_b, pt_b);

    curvy::output_debug_message(std::to_string(final_a.linear_magnitude() + final_b.linear_magnitude()));

    return {{
        a,
        residual_vector_a_to_b,
        impulse_b_to_a,
        final_a
    },{
        b,
        residual_vector_b_to_a,
        impulse_a_to_b,
        final_b
    }};
}

void curvy::curvy_vector_viewer::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush(colors::Black);
    gdi::Pen white_pen(colors::White, 1);
    std::unique_ptr<gdi::Graphics> g( gdi::Graphics::FromImage(back_buffer_.get()) );

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);

    
    auto [a, b] = get_collision_vectors(show_puck_b_vectors_, puck_a_, puck_b_);
    auto sz_const = puck_a_.puck_circle().radius() + puck_b_.puck_circle().radius();

    a.paint(*g, puck_a_.color(), puck_a_.position(), sz_const, logical_sz_, pixel_sz_);
    b.paint(*g, puck_b_.color(), puck_b_.position(), sz_const, logical_sz_, pixel_sz_);

    puck_a_.paint(*g, logical_sz_, pixel_sz_);
    puck_b_.paint(*g, logical_sz_, pixel_sz_);

}

curvy::interaction curvy::curvy_vector_viewer::get_interaction(const std::tuple<double, double>& click_location) const
{
    const auto& circle_of_rev = puck_a_.state().circle();
    if (curvy::circle(circle_of_rev.center(), 1.0).contains(click_location))
        return interaction::dragging_circle_of_rev;

    if (puck_a_.puck_circle().contains(click_location))
        return interaction::dragging_a;

    if (puck_b_.puck_circle().contains(click_location))
        return interaction::dragging_b;

    if (circle_of_rev.perimeter_contains( click_location, 0.1))
        return interaction::resizing_circle_of_rev;

    if (show_puck_b_vectors_) {

        if (is_in_arrow_b(click_location))
            return interaction::dragging_arrow_b;

        if (puck_b_.state().circle().perimeter_contains(click_location, 0.1)) {
            if (GetAsyncKeyState(VK_SHIFT) & (1 << 15))
                return interaction::resizing_circle_b;
            else
                return interaction::dragging_circle_b;
        }
    }

    return interaction::none;
}

void curvy::curvy_vector_viewer::sync_b_with_a(double old_a_theta)
{
    auto theta_offset = puck_a_.theta() - old_a_theta;
    puck_b_theta_ += theta_offset;
    auto r = puck_a_.puck_circle().radius() + puck_b_.puck_circle().radius();
    auto synced_b_position = puck_a_.position() + r * pt_on_unit_circle(puck_b_theta_);
    puck_b_.set_position(synced_b_position);

}

void curvy::curvy_vector_viewer::rotate_circle_b(const point& pt)
{
    auto old_theta = puck_b_.theta();
    auto old_position = puck_b_.position();
    auto theta = angle_to_pt(pt, puck_b_.position());

    matrix mat = translation_matrix(puck_b_.position()) * rotation_matrix(theta - old_theta)  * translation_matrix(-puck_b_.position());
    auto transformed_circle = apply_matrix(mat, puck_b_.state().circle());
    puck_b_.state().set_circle(transformed_circle);
    puck_b_.set_theta(angle_to_pt(transformed_circle.center(), old_position));
}

void curvy::curvy_vector_viewer::resize_circle_b(const point& pt)
{
    auto old_position = puck_b_.position();
    auto angle_from_puck = angle_to_pt(puck_b_.position(), pt);
    auto radius = euclidean_distance(puck_b_.position(), pt) / 2;
    auto new_center = 0.5 * (pt + puck_b_.position());
    auto c = circle(new_center, radius);
    puck_b_.state().set_circle(c);
    puck_b_.set_theta(angle_to_pt(c.center(), old_position));
}

void curvy::curvy_vector_viewer::drag_arrow_b(const point& pt)
{
    auto angle = angle_to_pt(puck_b_.state().circle().center(), pt);
    auto omega = angle - puck_b_.theta();
    puck_b_.set_speed(omega);
}

bool curvy::curvy_vector_viewer::is_in_arrow_b(const point& pt) const
{
    double sz_constant = puck_a_.puck_circle().radius() + puck_b_.puck_circle().radius();
    auto pts = arrow_poly_from_circle_vec(puck_b_.state(), puck_b_.position(), sz_constant);
    return pt_in_triangle( pt, pts[0], pts[1], pts[2] );
}
