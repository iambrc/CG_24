#include "view/shapes/freehand.h"
#include <cmath>

#include <imgui.h>

namespace USTC_CG
{
// Draw freehand figure using ImGui

void Freehand::draw() const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    size_t len = point_x.size();

    for (size_t i = 0; i < len - 1; i++)
    {
        draw_list->AddLine(
        ImVec2(
            config.bias[0] + point_x[i], config.bias[1] + point_y[i]),
        ImVec2(config.bias[0] + point_x[i + 1], config.bias[1] + point_y[i + 1]),
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        config.line_thickness);
    }

}

void Freehand::update(float x, float y)
{
    current_point_x = x;
    current_point_y = y;
    addpoint();
}

void Freehand::addpoint()
{
    point_x.push_back(current_point_x);
    point_y.push_back(current_point_y);
}
}