#include "view/shapes/ellipse.h"
#include <cmath>

#include <imgui.h>

namespace USTC_CG
{
//Draw the ellipse using ImGui

void Ellipse::draw() const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddEllipse(
        ImVec2(
            (2*config.bias[0] + start_point_x_ + end_point_x_) / 2, 
            (2*config.bias[1] + start_point_y_ + end_point_y_) / 2),
            fabs(end_point_x_ - start_point_x_) / 2, 
            fabs(end_point_y_ - start_point_y_) / 2,  // using fabs to make sure radius > 0
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        0.f,
        ImDrawFlags_None,
        config.line_thickness);
}

void Ellipse::update(float x, float y)
{
    end_point_x_ = x;
    end_point_y_ = y;
}

void Ellipse::addpoint()
{
}

}