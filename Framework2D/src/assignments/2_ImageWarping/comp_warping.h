#pragma once

#include "view/comp_image.h"
#include "warping.h"

namespace USTC_CG
{
// Image component for warping and other functions
class CompWarping : public ImageEditor
{
   public:
    explicit CompWarping(const std::string& label, const std::string& filename);
    virtual ~CompWarping() noexcept = default;

    void draw() override;

    // Simple edit functions
    void invert();
    void mirror(bool is_horizontal, bool is_vertical);
    void gray_scale();
    void warping();
    void restore();

    // Point selecting interaction
    void enable_selecting(bool flag);
    void select_points();
    void init_selections();

    void set_fish();
    void set_IDW();
    void set_RBF();
    
    enum WarpType
    {
        kDefault = 0,
        kfish = 1,
        kIDW = 2,
        kRBF = 3,
    };

   private:
    // Store the original image data
    std::shared_ptr<Image> back_up_;
    // The selected point couples for image warping
    std::vector<ImVec2> start_points_, end_points_;

    ImVec2 start_, end_;
    bool flag_enable_selecting_points_ = false;
    bool draw_status_ = false;

    // here we define current method of warping
    std::shared_ptr<Warping> current_warp_method;
    WarpType current_type;
};

}  // namespace USTC_CG