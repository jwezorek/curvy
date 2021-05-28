#include "post.h"

curvy::post::post(const circle& c, gdi::Color color) : circle_(c), color_(color)
{
}

const curvy::circle& curvy::post::post_circle() const
{
    return circle_;
}
