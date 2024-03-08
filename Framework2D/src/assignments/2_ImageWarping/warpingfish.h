#pragma once
#include "warping.h"
#include "imgui.h"

namespace USTC_CG
{
    class WarpingFish : public Warping
    {
        public:
         WarpingFish() = default;
         
         virtual ~WarpingFish() = default;

         std::pair<int, int> warping(int x, int y);
         
         void set_pq(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points);
         
         void set_hw(int h, int w);
         
         void prepare();

        private:
         int height,width;
    };
}