#include "gdi_util.h"
#include <array>

namespace {

    void paint_triangle(gdi::Graphics& g, gdi::Color color, const curvy::point& p1, const curvy::point& p2, const curvy::point& p3, double log_sz, int pix_sz) {
        gdi::Point ary[3] = {
            curvy::to_scr_point(p1, log_sz, pix_sz),
            curvy::to_scr_point(p2, log_sz, pix_sz),
            curvy::to_scr_point(p3, log_sz, pix_sz)
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

    void paint_arc_arrow(gdi::Graphics& g, const curvy::curvy_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz) {
        auto theta = curvy::normalize_angle(curvy::angle_to_pt(crc.circle().center(), pt));
        gdi::Pen pen(color, 3);
        g.DrawArc(&pen, curvy::to_scr_rect(crc.circle().bounding_box(), log_sz, pix_sz), -curvy::to_degrees_gdi(theta), -curvy::to_degrees_gdi(crc.signed_angular_magnitude()));
        auto pts = arrow_poly_from_circle_vec(crc, pt, puck_sz);
        paint_triangle(g, color, pts[0], pts[1], pts[2], log_sz, pix_sz);
    }

    void paint_arrow(gdi::Graphics& g, const curvy::curvy_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz) {
        auto theta = crc.circle().degenerate_angle();
        gdi::Pen pen(color, 3);

        auto [x, y] = crc.circle().center();
        curvy::point end_pt = {
            x + crc.linear_magnitude() * std::cos(theta),
            y + crc.linear_magnitude() * std::sin(theta)
        };
        curvy::paint_line_segment(g, color, crc.circle().center(), end_pt, log_sz, pix_sz);
        auto pts = curvy::arrow_poly_at_pt(theta, end_pt, puck_sz);
        paint_triangle(g, color, pts[0], pts[1], pts[2], log_sz, pix_sz);
    }

}

std::array<curvy::point, 3> curvy::arrow_poly_from_circle_vec(const curvy::curvy_vector& crc, const curvy::point& pt, double puck_sz) {
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

std::array<curvy::point, 3> curvy::arrow_poly_at_pt(double theta, const curvy::point& pt, double puck_sz)  {
    auto pts = get_arrow_poly(pt, puck_sz * 0.25, puck_sz * 0.125, theta);
    return pts;
}

gdi::Point curvy::to_scr_point(const curvy::point& p, double log_sz, int pix_sz) {
    auto [x, y] = curvy::to_scr_coords(p, log_sz, pix_sz);
    return { x,y };
}

void curvy::paint_line_segment(gdi::Graphics& g, gdi::Color color, const curvy::point& p1, const curvy::point& p2, double log_sz, int pix_sz) {
    auto [x1, y1] = to_scr_point(p1, log_sz, pix_sz);
    auto [x2, y2] = to_scr_point(p2, log_sz, pix_sz);
    gdi::Pen pen(color, 3);
    g.DrawLine(&pen, x1, y1, x2, y2);
}

gdi::Rect curvy::to_scr_rect(const std::tuple<double, double, double, double>& r, double log_sz, int pix_sz)
{
    auto [x1, y1, x2, y2] = curvy::to_scr_coords(r, log_sz, pix_sz);
    auto x = (x1 < x2) ? x1 : x2;
    auto y = (y1 < y2) ? y1 : y2;
    return gdi::Rect(x, y, std::abs(x1 - x2), std::abs(y1 - y2));
}

void curvy::paint_circle(gdi::Graphics& g, const curvy::circle& c, gdi::Color color, double log_sz, int pix_sz)
{
    gdi::Pen pen(color, 3);
    if (!c.is_degenerate()) {
        g.DrawEllipse(&pen, to_scr_rect(c.bounding_box(), log_sz, pix_sz));
    } else {
        auto [x, y] = c.center();
        auto theta = c.degenerate_angle();
        auto pt1 = point{ x - 20.0 * std::cos(theta), y - 20.0 * std::sin(theta) };
        auto pt2 = point{ x + 20.0 * std::cos(theta), y + 20.0 * std::sin(theta) };
        paint_line_segment(g, color, pt1, pt2, log_sz, pix_sz);
    }
}

void curvy::paint_point(gdi::Graphics& g, const curvy::point& pt, gdi::Color color, double log_sz, int pix_sz)
{
    paint_circle(g, circle(pt, 0.05), color, log_sz, pix_sz);
}

gdi::REAL curvy::to_degrees_gdi(double radians)
{
    return static_cast<gdi::REAL>(radians * 180.0 / curvy::pi());
}

void curvy::paint_circle_vector(gdi::Graphics& g, const curvy::curvy_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz)
{
    paint_circle(g, crc.circle(), gdi::Color(100, color.GetR(), color.GetG(), color.GetB()), log_sz, pix_sz);
    if (crc.linear_magnitude() != 0) {
        if (! crc.circle().is_degenerate())
            paint_arc_arrow(g, crc, color, puck_sz, pt, log_sz, pix_sz);
        else
            paint_arrow(g, crc, color, puck_sz, pt, log_sz, pix_sz);

    }
}
