#pragma once
#include <Eigen/Sparse>
#include "view/image.h"

namespace USTC_CG
{
    class Seamless
    {
        public:
         virtual ~Seamless() = default;

         // prepare the matrix to be solved
         virtual void prepare() = 0;
         // solve the linear equation
         virtual void solve() = 0;
         
         virtual void update(int x, int y, std::shared_ptr<Image>& source_data, std::shared_ptr<Image>& target, 
            bool *flag, std::vector<std::vector<int>>& index, int tarx, int tary)=0;

         virtual void set_size(int n)=0;

         virtual std::vector<unsigned char> get_result(int index)=0;
    };

}