#pragma once

#include <Windows.h>
#include <gdiplus.h>
#include "util.h"
#include "circle.h"
#include "curvy_vector.h"

namespace gdi = Gdiplus;

namespace curvy {

    gdi::Point to_scr_point(const curvy::point& p, double log_sz, int pix_sz);
    void paint_line_segment(gdi::Graphics& g, gdi::Color color, const curvy::point& p1, const curvy::point& p2, double log_sz, int pix_sz);
    gdi::Rect to_scr_rect(const std::tuple<double, double, double, double>& r, double log_sz, int pix_sz);
    void paint_circle(gdi::Graphics& g, const curvy::circle& c, gdi::Color color, double log_sz, int pix_sz);
    void paint_point(gdi::Graphics& g, const curvy::point& pt, gdi::Color color, double log_sz, int pix_sz);
    gdi::REAL to_degrees_gdi(double radians);
    void paint_circle_vector(gdi::Graphics& g, const curvy::curvy_vector& crc, gdi::Color color, double puck_sz, const curvy::point& pt, double log_sz, int pix_sz);
    std::array<curvy::point, 3> arrow_poly_from_circle_vec(const curvy::curvy_vector& crc, const curvy::point& pt, double puck_sz);
    std::array<curvy::point, 3> arrow_poly_at_pt(double theta, const curvy::point& pt, double puck_sz);
    bool is_light_theme();
    gdi::Color get_background_color();
    gdi::Color get_foreground_color();
}