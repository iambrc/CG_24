# pragma once

#include "warping.h"
#include "imgui.h"
#include <Eigen/Dense>

namespace USTC_CG
{
    class WarpingRBF : public Warping
    {
        public:
         WarpingRBF() = default;

         virtual ~WarpingRBF() = default;

         std::pair<int, int> warping(int x, int y);

         void set_pq(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points);
         
         void set_hw(int h, int w);

         void prepare();

        private:
         std::vector<ImVec2> p,q;
         std::vector<float> px{}, py{}, qx{}, qy{};
         std::vector<float> result;

    };
}