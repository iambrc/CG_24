#include "comp_source_image.h"
#include "view/shapes/rect.h"
#include "view/shapes/polygon.h"

#include <algorithm>
#include <cmath>

namespace USTC_CG
{
using uchar = unsigned char;

CompSourceImage::CompSourceImage(
    const std::string& label,
    const std::string& filename)
    : ImageEditor(label, filename)
{
    if (data_)
        selected_region_ =
            std::make_shared<Image>(data_->width(), data_->height(), 1);
}

void CompSourceImage::draw()
{
    // Draw the image
    ImageEditor::draw();
    // Draw selected region
    if (flag_enable_selecting_region_)
        select_region();
}

void CompSourceImage::enable_selecting(bool flag, int type)
{
    flag_enable_selecting_region_ = flag;
    if (type == 1)
        region_type_ = kRect;
    else if (type == 2)
        region_type_ = kPolygon;
    else
        region_type_ = kDefault;
}

void CompSourceImage::select_region()
{
    /// Invisible button over the canvas to capture mouse interactions.
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    // Record the current status of the invisible button
    bool is_hovered_ = ImGui::IsItemHovered();
    // HW3_TODO(optional): You can add more shapes for region selection. You can
    // also consider using the implementation in HW1. (We use rectangle for
    // example)
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        if (!draw_status_)
        {
            draw_status_ = true;
            current_shape_.reset();
            for (int i = 0; i < selected_region_->width(); ++i)
                    for (int j = 0; j < selected_region_->height(); ++j)
                        selected_region_->set_pixel(i, j, { 0 });
            
            size_ = 0;
            index_.resize(selected_region_->width(), std::vector<int>(selected_region_->height()));
            start_ = end_ = ImVec2(
            std::clamp<float>(
                io.MousePos.x - position_.x, 0, (float)image_width_),
            std::clamp<float>(
                io.MousePos.y - position_.y, 0, (float)image_height_));
            switch (region_type_)
            {
            case USTC_CG::CompSourceImage::kDefault: break;
            case USTC_CG::CompSourceImage::kRect:
            {
                current_shape_ = std::make_shared<Rect>(
                    start_.x, start_.y, end_.x, end_.y);
                break;
            }
            case USTC_CG::CompSourceImage::kPolygon:
            {
                current_shape_ = std::make_shared<Polygon>(start_.x, start_.y);
                is_drawing_polygon = true;
                break;
            }
            default:
                break;
            }
        }
        else
        {
            if (is_drawing_polygon)
            {
                current_shape_->addpoint();
            }
            else
            {
                draw_status_ = false;
                switch (region_type_)
                {
                    case USTC_CG::CompSourceImage::kDefault: break;
                    case USTC_CG::CompSourceImage::kRect:
                    {
                        for (int i = static_cast<int>(start_.x);
                            i < static_cast<int>(end_.x);
                            ++i)
                        {
                            for (int j = static_cast<int>(start_.y);
                                j < static_cast<int>(end_.y);
                                ++j)
                            {
                                selected_region_->set_pixel(i, j, { 255 });
                                // here we define the index
                                size_++;
                                index_[i][j] = size_;
                            }
                        }
                        break;
                    }
                    // polygon case is in mouse clicked right event
                    default: break;
                }
            }
            
        }
        
    }
    if (draw_status_)
    {
        end_ = ImVec2(
            std::clamp<float>(
                io.MousePos.x - position_.x, 0, (float)image_width_),
            std::clamp<float>(
                io.MousePos.y - position_.y, 0, (float)image_height_));
        current_shape_->update(end_.x,end_.y);
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
        if (is_drawing_polygon)
        {
            is_drawing_polygon = false;
            draw_status_ = false;
            size_ = 0;
            index_.resize(selected_region_->width(), std::vector<int>(selected_region_->height()));
            std::vector<float> pointsx = current_shape_->get_pointx();
            std::vector<float> pointsy = current_shape_->get_pointy();
            Scan_line(pointsx, pointsy,index_,selected_region_);
        }
    }

    if (current_shape_)
        draw_shapes();
}

void CompSourceImage::draw_shapes()
{
    auto draw_list = ImGui::GetWindowDrawList();
    current_shape_->config.bias[0] = position_.x;
    current_shape_->config.bias[1] = position_.y;
    current_shape_->draw();
}

std::shared_ptr<Image> CompSourceImage::get_region()
{
    return selected_region_;
}
std::shared_ptr<Image> CompSourceImage::get_data()
{
    return data_;
}
ImVec2 CompSourceImage::get_position() const
{
    return start_;
}
std::vector<std::vector<int>> CompSourceImage::get_index()
{
    return index_;
}
int CompSourceImage::get_size()
{
    return size_;
}

void CompSourceImage::Scan_line(std::vector<float>& pointsx, std::vector<float>& pointsy, 
std::vector<std::vector<int>>& index, std::shared_ptr<Image>& selected)
{
    // step1. fix the bias and find the boundary rect of polygon
    size_t num = pointsx.size();
    float min_y = pointsy[0],max_y =pointsy[0],min_x=pointsx[0],max_x=pointsx[0];
    for (size_t i = 0; i < num; i++)
    {
        if (pointsx[i] < min_x)
            min_x = pointsx[i];
        if (pointsx[i] > max_x)
            max_x = pointsx[i];
        if (pointsy[i] < min_y)
            min_y = pointsy[i];
        if (pointsy[i] > max_y)
            max_y = pointsy[i];
    }

    // step2. Scan: scan the polygon from min_y to max_y to get intersections
    int len = int(max_y)-int(min_y);
    for (int i = int(min_y); i < int(max_y); i++)
    {
        std::vector<float> intersections_x(num);
        for (size_t j = 0; j < num - 1; j++)
        {
            if (pointsy[j+1] != pointsy[j])
            {
                intersections_x[j] = (pointsx[j+1]-pointsx[j])/(pointsy[j+1]-pointsy[j])*(i-pointsy[j])+pointsx[j];
                if (intersections_x[j] < (pointsx[j] < pointsx[j+1] ? pointsx[j] : pointsx[j+1]) ||
                    intersections_x[j] > (pointsx[j] < pointsx[j+1] ? pointsx[j+1] : pointsx[j]))
                    intersections_x[j] = min_x - 1;
            }
            else
                intersections_x[j] = min_x - 1;
        }
        if (pointsy[num-1] != pointsy[0])
        {
            intersections_x[num-1] = (pointsx[0]-pointsx[num-1])/(pointsy[0]-pointsy[num-1])*(i-pointsy[num-1])+pointsx[num-1];
            if (intersections_x[num-1] < (pointsx[0] < pointsx[num-1] ? pointsx[0] : pointsx[num-1]) ||
                intersections_x[num-1] > (pointsx[0] < pointsx[num-1] ? pointsx[num-1] : pointsx[0]))
                intersections_x[num-1] = min_x - 1;
        }
        else
        {
            intersections_x[num-1] = min_x - 1;
        }
        
        std::sort(intersections_x.begin(),intersections_x.end());
        for (size_t k = 0; k < num - 1; k++)
        {
            if (intersections_x[k] > min_x && intersections_x[k] < max_x &&
            intersections_x[k+1] > min_x && intersections_x[k+1] < max_x)
            {
                for (int start = int(intersections_x[k]); start < int(intersections_x[k+1]); start++)
                {
                    size_++;
                    selected->set_pixel(start, i, { 255 });
                    index[start][i] = size_;
                }
                k++;
            }
        }
    }
}
}  // namespace USTC_CG