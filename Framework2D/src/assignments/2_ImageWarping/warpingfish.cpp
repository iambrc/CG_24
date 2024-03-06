#include "warpingfish.h"
#include <cmath>

namespace USTC_CG
{

std::pair<int, int> WarpingFish::warping(int x, int y)
{
    float center_x = width / 2.0f;
    float center_y = height / 2.0f;
    float dx = x - center_x;
    float dy = y - center_y;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Simple non-linear transformation r -> r' = f(r)
    float new_distance = std::sqrt(distance) * 10;

    if (distance == 0)
    {
        return { static_cast<int>(center_x), static_cast<int>(center_y) };
    }
    // (x', y')
    float ratio = new_distance / distance;
    int new_x = static_cast<int>(center_x + dx * ratio);
    int new_y = static_cast<int>(center_y + dy * ratio);

    return { new_x, new_y };
}

void WarpingFish::set_pq(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points)
{
}

void WarpingFish::set_hw(int h, int w)
{
    height = h;
    width = w;
}

}