#include "view/shapes/polygon.h"
#include <cmath>

#include <imgui.h>

namespace USTC_CG
{
// Draw the polygon using ImGui

void Polygon::draw() const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    std::vector<float> points_draw_x = point_x;
    std::vector<float> points_draw_y = point_y;
    points_draw_x.push_back(current_point_x);
    points_draw_y.push_back(current_point_y);
    size_t len = points_draw_x.size();

    // line the polygon by order 0,1...,n,0
    for (size_t i = 0; i < len - 1; i++)
    {
        draw_list->AddLine(
        ImVec2(
            config.bias[0] + points_draw_x[i], config.bias[1] + points_draw_y[i]),
        ImVec2(config.bias[0] + points_draw_x[i + 1], config.bias[1] + points_draw_y[i + 1]),
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        config.line_thickness);
    }
    // line the first point and the last point to make a polygon
    draw_list->AddLine(
        ImVec2(
            config.bias[0] + points_draw_x[len - 1], config.bias[1] + points_draw_y[len - 1]),
        ImVec2(config.bias[0] + points_draw_x[0], config.bias[1] + points_draw_y[0]),
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        config.line_thickness);
}

void Polygon::update(float x, float y)
{
    current_point_x = x;
    current_point_y = y;
}

void Polygon::addpoint()
{
    point_x.push_back(current_point_x);
    point_y.push_back(current_point_y);
}

std::vector<float> Polygon::get_pointx()
{
    std::vector<float> points_draw_x = point_x;
    points_draw_x.push_back(current_point_x);
    return points_draw_x;
}

std::vector<float> Polygon::get_pointy()
{
    std::vector<float> points_draw_y = point_y;
    points_draw_y.push_back(current_point_y);
    return points_draw_y;
}

}