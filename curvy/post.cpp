#include "post.h"

curvy::post::post(const circle& c, gdi::Color color) : circle_(c), color_(color)
{
}

const curvy::circle& curvy::post::post_circle() const
{
    return circle_;
}

void curvy::post::paint(gdi::Graphics& g, double log_sz, int pix_sz) const
{
    gdi::SolidBrush brush(color_);
    auto [x1, y1, x2, y2] = bounding_box_in_pixels(circle_, log_sz, pix_sz);
    g.FillEllipse(&brush, x1, y1, x2 - x1, y2 - y1);
}
