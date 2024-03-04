#include "window_minidraw.h"

#include <iostream>

namespace USTC_CG
{
MiniDraw::MiniDraw(const std::string& window_name) : Window(window_name)
{
    p_canvas_ = std::make_shared<Canvas>("Cmpt.Canvas");
}

MiniDraw::~MiniDraw()
{
}

void MiniDraw::draw()
{
    draw_canvas();
}

void MiniDraw::draw_canvas()
{
    // Set a full screen canvas view
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    if (ImGui::Begin(
            "Canvas",
            &flag_show_canvas_view_,
            ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoBackground))
    {
        // Buttons for shape types
        if (ImGui::Button("Line"))
        {
            std::cout << "Set shape to Line" << std::endl;
            p_canvas_->set_line();
        }
        ImGui::SameLine();
        if (ImGui::Button("Rect"))
        {
            std::cout << "Set shape to Rect" << std::endl;
            p_canvas_->set_rect();
        }
        ImGui::SameLine();
        if (ImGui::Button("Ellipse"))
        {
            std::cout << "Set shape to Ellipse" << std::endl;
            p_canvas_->set_ellipse();
        }
        ImGui::SameLine();
        if (ImGui::Button("Polygon"))
        {
            std::cout << "Set shape to Polygon" << std::endl;
            p_canvas_->set_polygon();
        }
        ImGui::SameLine();
        if (ImGui::Button("FreeHand"))
        {
            std::cout << "Set shape to Freehand" << std::endl;
            p_canvas_->set_freehand();
        }
        ImGui::SameLine();
        
        // Canvas component
        ImGui::Text("Press left mouse to add shapes.");

        // set the line color
        if (ImGui::ColorButton("0", {0,0,0,1}))
        {
            p_canvas_->set_color({0,0,0,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("1", {1,0,0,1}))
        {
            p_canvas_->set_color({1,0,0,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("2", {0,1,0,1}))
        {
            p_canvas_->set_color({0,1,0,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("3", {0,0,1,1}))
        {
            p_canvas_->set_color({0,0,1,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("4", {1,1,0,1}))
        {
            p_canvas_->set_color({1,1,0,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("5", {1,0,1,1}))
        {
            p_canvas_->set_color({1,0,1,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("6", {0,1,1,1}))
        {
            p_canvas_->set_color({0,1,1,1});
        }
        ImGui::SameLine();
        if (ImGui::ColorButton("7", {1,1,1,1}))
        {
            p_canvas_->set_color({1,1,1,1});
        }
        ImGui::SameLine();
        // here change the line thickness
        ImGui::SliderFloat("LineThickness",&p_canvas_->current_line_thickness,1.0,10.0);
        // Set the canvas to fill the rest of the window
        const auto& canvas_min = ImGui::GetCursorScreenPos();
        const auto& canvas_size = ImGui::GetContentRegionAvail();
        p_canvas_->set_attributes(canvas_min, canvas_size);
        p_canvas_->draw();
    }
    ImGui::End();
}
}  // namespace USTC_CG