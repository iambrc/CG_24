#pragma once
#include "seamless.h"
#include "comp_source_image.h"
#include "view/image.h"
#include <vector>

namespace USTC_CG
{
    class SeamlessD : public Seamless
    {
        public:
         SeamlessD() = default;

         virtual ~SeamlessD() = default;

         void prepare();

         void set_size(int n);

         void solve();

         std::vector<unsigned char> get_result(int index);

         void update(int x, int y, std::shared_ptr<Image>& source_data, std::shared_ptr<Image>& target, 
            bool *flag, std::vector<std::vector<int>>& index, int tarx, int tary);

        private:
         int size;
         Eigen::SparseMatrix<float> A;
         std::vector<Eigen::Triplet<float>> tList;
         Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> lu;
         Eigen::VectorXf R,G,B,resultR,resultG,resultB;
    };

}