#pragma once

#include "view/comp_image.h"

namespace USTC_CG
{
class CompSourceImage : public ImageEditor
{
   public:
    // HW3_TODO(optional): Add more region shapes like polygon and freehand.
    enum RegionType
    {
        kDefault = 0,
        kRect = 1
    };

    explicit CompSourceImage(
        const std::string& label,
        const std::string& filename);
    virtual ~CompSourceImage() noexcept = default;

    void draw() override;

    // Point selecting interaction
    void enable_selecting(bool flag);
    void select_region();
    // Get the selected region in the source image, this would be a binary mask
    std::shared_ptr<Image> get_region();
    // Get the image data
    std::shared_ptr<Image> get_data();
    // Get the position to locate the region in the target image
    ImVec2 get_position() const;
    // Get the index of selected region
    std::vector<std::vector<int>> get_index();
    // Get total pixel in the region
    int get_size();

   private:
    int size_;
    RegionType region_type_ = kRect;
    std::shared_ptr<Image> selected_region_;
    ImVec2 start_, end_;
    bool flag_enable_selecting_region_ = false;
    bool draw_status_ = false;
    std::vector<std::vector<int>> index_{};
};

}  // namespace USTC_CG