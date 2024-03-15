#include "comp_warping.h"
#include "warping.h"
#include "warpingIDW.h"
#include "warpingfish.h"
#include "warpingRBF.h"

#include <cmath>

namespace USTC_CG
{
using uchar = unsigned char;

CompWarping::CompWarping(const std::string& label, const std::string& filename)
    : ImageEditor(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void CompWarping::draw()
{
    // Draw the image
    ImageEditor::draw();
    // Draw the canvas
    if (flag_enable_selecting_points_)
        select_points();
}

void CompWarping::invert()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            data_->set_pixel(
                i,
                j,
                { static_cast<uchar>(255 - color[0]),
                  static_cast<uchar>(255 - color[1]),
                  static_cast<uchar>(255 - color[2]) });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}
void CompWarping::mirror(bool is_horizontal, bool is_vertical)
{
    Image image_tmp(*data_);
    int width = data_->width();
    int height = data_->height();

    if (is_horizontal)
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i,
                        j,
                        image_tmp.get_pixel(width - 1 - i, height - 1 - j));
                }
            }
        }
        else
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(width - 1 - i, j));
                }
            }
        }
    }
    else
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(i, height - 1 - j));
                }
            }
        }
    }

    // After change the image, we should reload the image data to the renderer
    update();
}
void CompWarping::gray_scale()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            uchar gray_value = (color[0] + color[1] + color[2]) / 3;
            data_->set_pixel(i, j, { gray_value, gray_value, gray_value });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}
void CompWarping::set_fish()
{
    current_type = kfish;
}
void CompWarping::set_IDW()
{
    current_type = kIDW;
}
void CompWarping::set_RBF()
{
    current_type = kRBF;
}

void CompWarping::warping()
{
    // HW2_TODO: You should implement your own warping function that interpolate
    // the selected points.
    // You can design a class for such warping operations, utilizing the
    // encapsulation, inheritance, and polymorphism features of C++. More files
    // like "*.h", "*.cpp" can be added to this directory or anywhere you like.

    // Create a new image to store the result
    Image warped_image(*data_);
    std::vector<std::vector<bool>> is_changed(data_->width(),std::vector<bool>(data_->height()));
    // we use this matrix to judge if the pixel need to be fixed or not
    for (size_t i = 0; i < is_changed.size(); i++)
    {
        for (size_t j = 0; j < is_changed[0].size(); j++)
        {
            is_changed[i][j] = false;
        }
    }
    // Initialize the color of result image
    for (int y = 0; y < data_->height(); ++y)
    {
        for (int x = 0; x < data_->width(); ++x)
        {
            warped_image.set_pixel(x, y, { 255, 255, 255 });
        }
    }

    // Example: (simplified) "fish-eye" warping
    // For each (x, y) from the input image, the "fish-eye" warping transfer it
    // to (x', y') in the new image:
    // Note: For this transformation ("fish-eye" warping), one can also
    // calculate the inverse (x', y') -> (x, y) to fill in the "gaps".
    switch (current_type)
    {
        case kDefault:
        {
            break;
        } 
        case USTC_CG::CompWarping::kfish:
        {
            current_warp_method = std::make_shared<WarpingFish>();
            current_warp_method->set_hw(data_->height(),data_->width());
            break;
        }
        case USTC_CG::CompWarping::kIDW:
        {
            current_warp_method = std::make_shared<WarpingIDW>();
            current_warp_method->set_pq(start_points_, end_points_);
            break;
        }
        case USTC_CG::CompWarping::kRBF:
        {
            current_warp_method = std::make_shared<WarpingRBF>();
            current_warp_method->set_pq(start_points_, end_points_);
            current_warp_method->prepare();
            break;
        }
        default:break;
    }
    for (int y = 0; y < data_->height(); ++y)
    {
        for (int x = 0; x < data_->width(); ++x)
        {
            // Apply warping function to (x, y), and we can get (x', y')
            auto [new_x, new_y] = current_warp_method->warping(x, y);
            
            // Copy the color from the original image to the result image
            if (new_x >= 0 && new_x < data_->width() && new_y >= 0 &&
                new_y < data_->height())
            {
                std::vector<unsigned char> pixel = data_->get_pixel(x, y);
                warped_image.set_pixel(new_x, new_y, pixel);
                is_changed[new_x][new_y] = true;
            }
        }
    }
    // here we fix the white hole in picture
    for (int y = 2; y < data_->height() - 2; ++y)
    {
        for (int x = 2; x < data_->width() - 2; ++x)
        {
            // we find the default pixels and fix them
            if (!is_changed[x][y])
            {
                std::vector<float> changed_x{}, changed_y{};
                std::vector<float> change_color{0.0f, 0.0f, 0.0f};
                for (int i = x - 2; i < x + 3; i++)
                {
                    for (int j = y - 2; j < y + 3; j++)
                    {
                        // we use pixels that are not default to calculate the center pixel
                        if (is_changed[i][j])
                        {
                            changed_x.push_back(float(i));
                            changed_y.push_back(float(j));
                        }
                    }
                }
                size_t len = changed_x.size();
                std::vector<float> sigma(len);
                float sum_sigma = 0.0f;
                for (size_t k = 0; k < len; k++)
                {
                    sigma[k] = 1.0f / float((changed_x[k]-x)*(changed_x[k]-x) + 
                        (changed_y[k]-y)*(changed_y[k]-y));
                    sum_sigma += sigma[k];
                }
                for (size_t k = 0; k < len; k++)
                {
                    std::vector<unsigned char> xy_pixel = warped_image.get_pixel(int(changed_x[k]),int(changed_y[k]));
                    change_color[0] += (sigma[k] / sum_sigma) * float(xy_pixel[0]);
                    change_color[1] += (sigma[k] / sum_sigma) * float(xy_pixel[1]);
                    change_color[2] += (sigma[k] / sum_sigma) * float(xy_pixel[2]);
                }
                // change float to unsigned char
                std::vector<unsigned char> changed_color(3);
                changed_color[0] = unsigned char(change_color[0]);
                changed_color[1] = unsigned char(change_color[1]);
                changed_color[2] = unsigned char(change_color[2]);
                warped_image.set_pixel(x, y, changed_color);
            }
        }
    }
    *data_ = std::move(warped_image);
    update();
}
void CompWarping::restore()
{
    *data_ = *back_up_;
    update();
}
void CompWarping::enable_selecting(bool flag)
{
    flag_enable_selecting_points_ = flag;
}
void CompWarping::select_points()
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
    // Selections
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        draw_status_ = true;
        start_ = end_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
    }
    if (draw_status_)
    {
        end_ = ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            start_points_.push_back(start_);
            end_points_.push_back(end_);
            draw_status_ = false;
        }
    }
    // Visualization
    auto draw_list = ImGui::GetWindowDrawList();
    for (size_t i = 0; i < start_points_.size(); ++i)
    {
        ImVec2 s(
            start_points_[i].x + position_.x, start_points_[i].y + position_.y);
        ImVec2 e(
            end_points_[i].x + position_.x, end_points_[i].y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
        draw_list->AddCircleFilled(e, 4.0f, IM_COL32(0, 255, 0, 255));
    }
    if (draw_status_)
    {
        ImVec2 s(start_.x + position_.x, start_.y + position_.y);
        ImVec2 e(end_.x + position_.x, end_.y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
    }
}
void CompWarping::init_selections()
{
    start_points_.clear();
    end_points_.clear();
}
}