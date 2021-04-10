#include <Windows.h>
#include <cmath>
#include "impulse_viewer.h"
#include "util.h"
#include "circular_vector.h"

namespace {

    gdi::Point to_scr_point(const curvy::point& p, double log_sz, int pix_sz) {
        auto [x, y] = curvy::to_scr_coords(p, log_sz, pix_sz);
        return { x,y };
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

    void paint_arrow_head(gdi::Graphics& g, gdi::Color color, const curvy::point& loc, double wd, double hgt, double direction, double log_sz, int pix_sz) {
        curvy::point pts[3] = {
            {wd, 0},
            {-wd, hgt},
            {-wd, -hgt}
        };
        curvy::matrix mat = curvy::translation_matrix(loc) * curvy::rotation_matrix(direction) ;
        for (auto& pt : pts) 
            pt = curvy::apply_matrix(mat, pt);
        paint_triangle(g, color, pts[0], pts[1], pts[2], log_sz, pix_sz);
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

    void paint_arc_arrow(gdi::Graphics& g, const curvy::circular_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz) {
        auto theta = curvy::normalize_angle(curvy::angle_to_pt(crc.circle().center(), pt));
        gdi::Pen pen(color, 3);
        g.DrawArc(&pen, to_scr_rect(crc.circle().bounding_box(), log_sz, pix_sz), -to_degrees(theta), -to_degrees(crc.signed_angular_magnitude()));
        auto arrow_theta = theta + crc.signed_angular_magnitude();
        auto arror_direction = curvy::direction_on_circle( arrow_theta, crc.orientation() );
        curvy::point arrow_pt = {
            crc.circle().x() + crc.circle().radius() * std::cos(arrow_theta),
            crc.circle().y() + crc.circle().radius() * std::sin(arrow_theta)
        };
        paint_arrow_head(g, color, arrow_pt, puck_sz * 0.25, puck_sz * 0.125, arror_direction, log_sz, pix_sz);
    }

    void paint_circle_vector(gdi::Graphics& g, const curvy::circular_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz) {
        paint_circle(g, crc.circle(), colors::White, log_sz, pix_sz);
        if (crc.angular_magnitude() != 0)
            paint_arc_arrow(g, crc, color, puck_sz, pt, log_sz, pix_sz);
    }
}

curvy::impulse_viewer::impulse_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz),
    interaction_(interaction::none),
    in_motion_(false),
    puck_b_theta_(0),
    show_puck_b_vectors_(false)
{
    set_logical_dimensions(log_sz, false);
    set_pixel_dimensions(px_sz, true);
}

void DebugAngles(const curvy::puck& a, const curvy::puck& b) {
    double d = a.puck_circle().radius() + b.puck_circle().radius();
    double r = a.state().circle().radius();
    auto on_circle_theta = std::asin(d / (2 * r));
    auto equals_circle_theta = std::asin(-d / (2 * r));
    auto half_circle_theta = std::atan(
        (d * d) / std::sqrt(-d * d * d * d + d * d * r * r)
    );

    std::stringstream ss;
    /*
    ss << "negative pi over two => -90.0\n";
    ss << "equals_circle_theta => " << equals_circle_theta * 180.0 / curvy::pi() << "\n";
    ss << "on_circle_theta => " << on_circle_theta * 180.0 / curvy::pi() << "\n";
    ss << "half_circle_theta =>" << half_circle_theta * 180.0 / curvy::pi() << "\n";
    ss << "pi over two => 90.0\n";
    */

    ss << "negative pi over two => " << -curvy::pi_over_two() << "\n";
    ss << "equals_circle_theta => " << equals_circle_theta  << "\n";
    ss << "on_circle_theta => " << on_circle_theta  << "\n";
    ss << "half_circle_theta =>" << half_circle_theta  << "\n";
    ss << "pi over two => " << curvy::pi_over_two() << "\n";

    OutputDebugStringA(ss.str().c_str());
}

void curvy::impulse_viewer::initialize()
{
    auto r = 0.35;
    const auto south = 3.0 * pi() / 2.0;

    puck_a_.set_circle_rotation_position( south, 0, 3, 3 );
    puck_a_.set_speed(1.0);
    puck_a_.set_color(colors::Red);
    puck_a_.set_puck_radius(r);

    puck_b_.set_color(colors::Yellow);
    puck_b_.set_puck_radius(r);
    puck_b_.set_circle_rotation_position(south, 0, 3, 3);
    puck_b_.set_speed(1.0);

    sync_b_with_a();
}

void curvy::impulse_viewer::update()
{
    render();
}

void curvy::impulse_viewer::update(double dt)
{
    if (in_motion_) {
        puck_b_.update(dt);
        render();
    }
}

bool curvy::impulse_viewer::handle_mouse_click(const std::tuple<int, int>& pt, bool mouse_down)
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

bool curvy::impulse_viewer::handle_mouse_move(const std::tuple<int, int>& pix_pt)
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
                 resizing_circle_b(pt);
                 break;

        }
        render();
        return true;
    }
    return false;
}

bool curvy::impulse_viewer::handle_key_press(unsigned int key, bool is_key_down)
{
    if (key == VK_INSERT) {
        if (is_key_down) {
            in_motion_ = !in_motion_;
            return true;
        } 
    }

    if (key == VK_SPACE) {
        if (is_key_down) {
            auto speed = puck_b_.state().signed_angular_magnitude();
            puck_b_.state().set_magnitude(-1.0 * speed);
            return true;
        }
    }

    if (key == '1' && is_key_down) {
        show_puck_b_vectors_ = !show_puck_b_vectors_;
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

double momentum_transfer_factor(const curvy::point& pt1, double pt1_direction, const curvy::point& pt2, double r1, double r2, double d) {
    auto theta = curvy::angle_to_point_relative_to_direction(pt1, pt1_direction, pt2);
    auto min_radius = d / 2.0;
    auto peak = std::atan(d / std::sqrt(-d * d + 4.0 * r1 * r1));

    if (theta < peak) {
        auto pcnt = (theta - (-curvy::pi_over_two())) / (peak - (-curvy::pi_over_two()));
        auto t2 = -curvy::pi_over_two() + pcnt * (curvy::pi_over_two() - peak);
        r2 = std::abs( (d*d) / (2.0 * d * std::sin(t2)));
    }

    auto val = (r2 - min_radius) / (r1 - min_radius);
    return val;
}

void curvy::impulse_viewer::render()
{
    if (!pixel_sz_)
        return;

    gdi::SolidBrush black_brush(colors::Black);
    gdi::Pen white_pen(colors::White, 1);
    std::unique_ptr<gdi::Graphics> g( gdi::Graphics::FromImage(back_buffer_.get()) );

    g->SetSmoothingMode(gdi::SmoothingModeAntiAlias);
    g->FillRectangle(&black_brush, 0, 0, pixel_sz_, pixel_sz_);

    double sz_constant = puck_a_.puck_circle().radius() + puck_b_.puck_circle().radius();
    auto circle_vector_a = puck_a_.state();
    auto circle_vector_b = puck_b_.state();
    auto pt_a = puck_a_.position();
    auto radius_a = puck_a_.state().circle().radius();
    auto direction_a = puck_a_.direction();
    auto pt_b = puck_b_.position();

    auto [circle_of_impulse, impulse_orientation] = circular_direction_through_two_points(pt_a, direction_a, pt_b);
    auto coefficient = momentum_transfer_factor(pt_a, direction_a, pt_b, radius_a, circle_of_impulse.radius(), sz_constant);
    auto impulse_vector = curvy::circular_vector_from_linear_magnitude(circle_of_impulse, (impulse_orientation ? 1.0 : -1.0) * coefficient * circle_vector_a.linear_magnitude());
    auto residual_vector = circle_vector_a.subtract(impulse_vector, puck_a_.position());

    paint_circle_vector(*g, circle_vector_a, colors::Red, sz_constant, pt_a, logical_sz_, pixel_sz_);
    paint_circle_vector(*g, impulse_vector, colors::Yellow, sz_constant, pt_b, logical_sz_, pixel_sz_);
    paint_circle_vector(*g, residual_vector, colors::Blue, sz_constant, pt_a, logical_sz_, pixel_sz_);

    if (show_puck_b_vectors_) {
        paint_circle_vector(*g, circle_vector_b, colors::Blue, sz_constant, pt_b, logical_sz_, pixel_sz_);
    }

    puck_a_.paint(*g, logical_sz_, pixel_sz_);
    puck_b_.paint(*g, logical_sz_, pixel_sz_);

}

curvy::interaction curvy::impulse_viewer::get_interaction(const std::tuple<double, double>& click_location)
{
    auto circle_of_rev = puck_a_.state().circle();
    if (curvy::circle(circle_of_rev.center(), 1.0).contains(click_location))
        return interaction::dragging_circle_of_rev;

    if (puck_a_.puck_circle().contains(click_location))
        return interaction::dragging_a;

    if (puck_b_.puck_circle().contains(click_location))
        return interaction::dragging_b;

    if (circle_of_rev.perimeter_contains( click_location, 0.1))
        return interaction::resizing_circle_of_rev;

    if (show_puck_b_vectors_ && puck_b_.state().circle().perimeter_contains(click_location, 0.1)) {
        if (GetAsyncKeyState(VK_SHIFT) & (1 << 15))
            return interaction::resizing_circle_b;
        else
            return interaction::dragging_circle_b;
    }

    return interaction::none;
}

void curvy::impulse_viewer::sync_b_with_a(double old_a_theta)
{
    auto theta_offset = puck_a_.theta() - old_a_theta;
    puck_b_theta_ += theta_offset;
    auto r = puck_a_.puck_circle().radius() + puck_b_.puck_circle().radius();
    auto synced_b_position = puck_a_.position() + r * pt_on_unit_circle(puck_b_theta_);
    puck_b_.set_position(synced_b_position);

}

void curvy::impulse_viewer::rotate_circle_b(const point& pt)
{
    auto old_theta = puck_b_.theta();
    auto old_position = puck_b_.position();
    auto theta = angle_to_pt(pt, puck_b_.position());

    matrix mat = translation_matrix(puck_b_.position()) * rotation_matrix(theta - old_theta)  * translation_matrix(-puck_b_.position());
    auto transformed_circle = apply_matrix(mat, puck_b_.state().circle());
    puck_b_.state().set_circle(transformed_circle);
    puck_b_.set_theta(angle_to_pt(transformed_circle.center(), old_position));
}

void curvy::impulse_viewer::resizing_circle_b(const point& pt)
{
    auto old_position = puck_b_.position();
    auto angle_from_puck = angle_to_pt(puck_b_.position(), pt);
    auto radius = euclidean_distance(puck_b_.position(), pt) / 2;
    auto new_center = 0.5 * (pt + puck_b_.position());
    auto c = circle(new_center, radius);
    puck_b_.state().set_circle(c);
    puck_b_.set_theta(angle_to_pt(c.center(), old_position));
}
