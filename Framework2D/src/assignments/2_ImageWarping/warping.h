# pragma once

#include <vector>
#include "view/image.h"
#include "imgui.h"

namespace USTC_CG
{
    class Warping
    {
        public:
         virtual ~Warping() = default;

         virtual std::pair<int, int> warping(int x, int y) = 0;

         virtual void set_pq(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points) = 0;
         // fish warping need set height and width
         virtual void set_hw(int h, int w) = 0;
         // RBF warping need prepare
         virtual void prepare() = 0;

    };
}