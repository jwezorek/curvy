#include "curvy.h"
#include <cmath>
#include <string>
#include <map>
#include <limits>

namespace {

    const long double g_pi = acos(-1.L);

    HBITMAP create_bitmap( int wd, int hgt ) {
        HDC hdcScreen = GetDC(NULL);
        HBITMAP hbm = CreateCompatibleBitmap(hdcScreen, wd, hgt);
        ReleaseDC(NULL, hdcScreen);
        return hbm;
    }

    double euclidean_distance(double x1, double y1, double x2, double y2) {
        auto diff_x = x2 - x1;
        auto diff_y = y2 - y1;
        return std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2) {
        auto [x1, y1] = p1;
        auto [x2, y2] = p2;
        return euclidean_distance(x1, y1, x2, y2);
    }

    std::optional<double> get_collision_time(const curvy::puck& p1, const curvy::puck& p2, double t1, double t2, double eps) {
        auto p1_at_t2 = p1.update(t2);
        auto p2_at_t2 = p2.update(t2);

        auto distance_from_touching = p1_at_t2.distance_from_intersection(p2_at_t2);
        if (distance_from_touching > 0)
            return std::nullopt;
        
        if (std::abs(distance_from_touching) <= eps)
            return t2;

        auto half_time = (t1 + t2) / 2.0;
        bool intersects_at_halftime = p1.update(half_time).intersects(p2.update(half_time));
        if (intersects_at_halftime)
            return get_collision_time(p1, p2, t1, half_time, eps);
        else
            return get_collision_time(p1, p2, half_time, t2, eps);
    }

}

/*-------------------------------------------- puck --------------------------------------------*/

double normalize(const double value, const double start, const double end) {
    const double width = end - start; 
    const double offsetValue = value - start; 
    return (offsetValue - (floor(offsetValue / width) * width)) + start;
}

double curvy::pi() {
    return g_pi;
}

curvy::puck::puck(const circle_rotation_state& crs, COLORREF color, double puck_radius, double mass) :
    crs_(crs),
    color_(color),
    puck_radius_(puck_radius),
    mass_(mass)
{ }

void curvy::puck::update(double dt) {
    crs_.theta_ = normalize(crs_.theta_ + dt * crs_.speed_, 0, 2 * g_pi);
}

curvy::puck curvy::puck::update(double dt) const {
    puck clone(*this);
    clone.update(dt);
    return clone;
}

double curvy::puck::theta() const {
    return crs_.theta_;
}

std::tuple<double, double> curvy::puck::center_of_revolution() const {
    return { crs_.cx_, crs_.cy_ };
}

double curvy::puck::radius_of_revolution() const {
    return crs_.r_;
}

double curvy::puck::angular_speed() const
{
    return crs_.speed_;
}

double curvy::puck::radius() const
{
    return puck_radius_;
}

std::tuple<double, double> curvy::puck::position() const
{
    return {
        crs_.cx_ + crs_.r_ * std::cos(crs_.theta_),
        crs_.cy_ + crs_.r_ * std::sin(crs_.theta_)
    };
}

bool curvy::puck::intersects(const puck& p) const
{
    return distance_from_intersection(p) <= 0;
}

double curvy::puck::distance_from_center(const puck& p) const
{
    return euclidean_distance(position(), p.position());
}

double curvy::puck::distance_from_intersection(const puck& p) const
{
    auto dist = distance_from_center(p);
    auto distance_when_touching = puck_radius_ + p.radius();
    return dist - distance_when_touching;
}

std::optional<double> curvy::puck::get_collision_time(const puck& p, double dt, double eps) const
{
    return ::get_collision_time(*this, p, 0, dt, eps);
}

COLORREF curvy::puck::color() const
{
    return color_;
}

void curvy::puck::set_speed(double speed)
{
    crs_.speed_ = speed;
}

/*----------------------------------------------------------------------------------------------*/

curvy::state::state(int px_sz, double log_sz) : 
    back_buffer_(0),
    pixel_sz_(px_sz),
    logical_sz_(log_sz)
{
    set_dimensions( px_sz, log_sz);
}

void curvy::state::set_dimensions(int px_sz, double log_sz) 
{
    if (log_sz) {
        logical_sz_ = log_sz;
    }
    if (px_sz) {
        pixel_sz_ = px_sz;
        back_buffer_ = create_bitmap(px_sz, px_sz);
        render();
    }
}

int curvy::state::get_size() const
{
    return pixel_sz_;
}

void curvy::state::insert(const puck& p)
{
    pucks_.push_back(p);
    render();
}

HBITMAP curvy::state::get_bitmap() const
{
    return back_buffer_;
}

void curvy::state::update(double dt)
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

/*
void curvy::state::update(double dt)
{
    for (auto& p : pucks_)
        p.update(dt);
    render();
}
*/

void curvy::state::render()
{
    if (!pixel_sz_)
        return;

    HDC hdcScreen = GetDC(NULL);
    HDC hdc = CreateCompatibleDC(hdcScreen);
    auto hbmOld = SelectObject(hdc, back_buffer_);

    RECT r = { 0, 0, pixel_sz_, pixel_sz_ };
    FillRect(hdc, &r, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));
    
    SelectObject(hdc, static_cast<HBRUSH>(GetStockObject(NULL_BRUSH)));
    SelectObject(hdc, static_cast<HPEN>(GetStockObject(WHITE_PEN)));
    Ellipse(hdc, 0, 0, pixel_sz_, pixel_sz_);

    for (const auto& puck : pucks_)
        paint_puck(hdc, puck);

    SelectObject(hdc, hbmOld);
    DeleteDC(hdc);
    ReleaseDC(NULL, hdcScreen);
}

void curvy::state::paint_puck(HDC hdc, const puck& p)
{
    HBRUSH hbr = CreateSolidBrush(p.color());
    auto old_brush = SelectObject(hdc, static_cast<HBRUSH>(hbr));
    SelectObject(hdc, static_cast<HPEN>(GetStockObject(NULL_PEN)));
    auto [x1, y1, x2, y2] = get_location_in_pixels(p);
    Ellipse(hdc, x1, y1, x2, y2);
    SelectObject(hdc, old_brush);
    DeleteObject(hbr);
}

std::tuple<int, int, int, int> curvy::state::to_scr_coords(double x1, double y1, double x2, double y2) const
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

std::tuple<int, int, int, int> curvy::state::get_location_in_pixels(const puck& p) const
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

std::tuple<curvy::state::collisions, double> curvy::state::get_next_collisions(double dt, double eps) {
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
        curvy::state::collisions collision_set;
        double collision_time = -1;
        for (const auto& [time, pair] : collisions) {
            if (collision_time < 0 || std::abs(collision_time - time) < eps) {
                collision_time = (collision_time < 0) ? time : collision_time;
                collision_set.push_back(pair);
            }
        }
        return { collision_set, collision_time };
    }

    return { curvy::state::collisions(), dt };
}

void curvy::state::handle_collision( collision& collision) {
    auto [p1, p2] = collision;
    auto tmp = p1->angular_speed();
    p1->set_speed( p2->angular_speed() );
    p2->set_speed( tmp );
}

void curvy::state::handle_collisions( collisions& pairs)
{
    for (auto& pair : pairs)
        handle_collision(pair);
}


