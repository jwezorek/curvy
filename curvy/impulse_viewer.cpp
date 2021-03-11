#include <Windows.h>
#include "impulse_viewer.h"

curvy::impulse_viewer::impulse_viewer(int px_sz, double log_sz) :
    pixel_sz_(px_sz),
    logical_sz_(log_sz)
{
    set_dimensions(px_sz, log_sz);
}

void curvy::impulse_viewer::set_dimensions(int px_sz, double log_sz)
{
    if (log_sz) {
        logical_sz_ = log_sz;
    }
    if (px_sz) {
        pixel_sz_ = px_sz;
        back_buffer_ = std::make_unique<gdi::Bitmap>(px_sz, px_sz);
        render();
    }
}

int curvy::impulse_viewer::get_size() const
{
    return 0;
}

gdi::Bitmap* curvy::impulse_viewer::get_bitmap() const
{
    return nullptr;
}

void curvy::impulse_viewer::render()
{
}
