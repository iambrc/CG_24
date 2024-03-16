#include "comp_target_image.h"
#include "seamlessD.h"
#include "seamlessMix.h"
#include<iostream>

#include <cmath>

namespace USTC_CG
{
using uchar = unsigned char;

CompTargetImage::CompTargetImage(
    const std::string& label,
    const std::string& filename)
    : ImageEditor(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void CompTargetImage::draw()
{
    // Draw the image
    ImageEditor::draw();
    // Invisible button for interactions
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    bool is_hovered_ = ImGui::IsItemHovered();
    // When the mouse is clicked or moving, we would adapt clone function to
    // copy the selected region to the target.
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        edit_status_ = true;
        mouse_position_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        clone();
    }
    if (edit_status_)
    {
        mouse_position_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (flag_realtime_updating)
            clone();
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            edit_status_ = false;
        }
    }
}

void CompTargetImage::set_source(std::shared_ptr<CompSourceImage> source)
{
    source_image_ = source;
}

void CompTargetImage::set_realtime(bool flag)
{
    flag_realtime_updating = flag;
}

void CompTargetImage::restore()
{
    *data_ = *back_up_;
    update();
}

void CompTargetImage::set_paste()
{
    clone_type_ = kPaste;
}

void CompTargetImage::set_seamless()
{
    clone_type_ = kSeamless;
    prepared_ = false;
}

void CompTargetImage::set_seamless_mix()
{
    clone_type_ = kSeamlessMix;
    prepared_ = false;
}

void CompTargetImage::clone()
{
    // The implementation of different types of cloning
    // HW3_TODO: In this function, you should at least implement the "seamless"
    // cloning labeled by `clone_type_ ==kSeamless`.
    //
    // The realtime updating (update when the mouse is moving) is only available
    // when the checkboard is selected. It is required to improve the efficiency
    // of your seamless cloning to achieve realtime editing. (Use decomposition
    // of sparse matrix before solve the linear system)
    if (data_ == nullptr || source_image_ == nullptr ||
        source_image_->get_region() == nullptr)
        return;
    std::shared_ptr<Image> mask = source_image_->get_region();
    std::vector<std::vector<int>> index = source_image_->get_index();
    std::shared_ptr<Image> source_data = source_image_->get_data();

    switch (clone_type_)
    {
        case USTC_CG::CompTargetImage::kDefault: break;
        case USTC_CG::CompTargetImage::kPaste:
        {
            restore();

            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i,j)[0] > 0)
                    {
                        data_->set_pixel(
                            tar_x,
                            tar_y,
                            source_data->get_pixel(i, j));
                    }
                }
            }
            break;
        }
        case USTC_CG::CompTargetImage::kSeamless:
        {
            restore();
            if (!(flag_realtime_updating && prepared_))
            {
                current_method = std::make_shared<SeamlessD>();
                current_method->set_size(source_image_->get_size());
            }
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {                        
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i,j)[0] > 0)
                    {
                        bool flag[4] = {index[i][j-1] != 0, index[i][j+1] != 0,
                        index[i-1][j] != 0, index[i+1][j] != 0};
                        current_method->update(i,j,source_data,data_,flag,index,tar_x,tar_y);
                    }
                }
            }
            if (!(flag_realtime_updating && prepared_))
            {
                current_method->prepare();
                prepared_ = true;
            }
            current_method->solve();

            // fill the new pixel
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i,j)[0] > 0)
                    {
                        std::vector<unsigned char> new_pixel(3);
                        new_pixel = current_method->get_result(index[i][j]-1);
                        data_->set_pixel(
                            tar_x,
                            tar_y,
                            new_pixel);
                    }
                }
            }
            break;
        }
        case USTC_CG::CompTargetImage::kSeamlessMix:
        {
            restore();
            if (!(flag_realtime_updating && prepared_))
            {
                current_method = std::make_shared<SeamlessMix>();
                current_method->set_size(source_image_->get_size());
            }   
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {                        
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i,j)[0] > 0)
                    {
                        bool flag[4] = {index[i][j-1] != 0, index[i][j+1] != 0,
                        index[i-1][j] != 0, index[i+1][j] != 0};
                        current_method->update(i,j,source_data,data_,flag,index,tar_x,tar_y);
                    }
                }
            }
            if (!(flag_realtime_updating && prepared_))
            {
                current_method->prepare();
                prepared_ = true;
            }
            
            current_method->solve();
            
            // fill the new pixel
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i,j)[0] > 0)
                    {
                        std::vector<unsigned char> new_pixel(3);
                        new_pixel = current_method->get_result(index[i][j]-1);
                        data_->set_pixel(
                            tar_x,
                            tar_y,
                            new_pixel);
                    }
                }
            }
            break;
        }

        default: break;
    }

    update();
}

}  // namespace USTC_CG