#include "seamlessMix.h"
#include <cmath>

namespace USTC_CG
{
typedef Eigen::Triplet<float> T;

void SeamlessMix::set_size(int n)
{
    size = n;
    R.resize(n);
    G.resize(n);
    B.resize(n);
    resultR.resize(n);
    resultG.resize(n);
    resultB.resize(n);
    A.resize(n,n);
}

void SeamlessMix::update(int x, int y, std::shared_ptr<Image>& source_data, std::shared_ptr<Image>& target, 
    bool *flag, std::vector<std::vector<int>>& index, int tarx, int tary)
{
    int step = index[x][y];
    tList.push_back(T(step-1, step-1, -4.0f));
    std::vector<unsigned char> pixel = source_data->get_pixel(x,y);
    std::vector<unsigned char> pixel_up = source_data->get_pixel(x,y-1),
        pixel_bottom = source_data->get_pixel(x,y+1),
        pixel_left = source_data->get_pixel(x-1,y),
        pixel_right = source_data->get_pixel(x+1,y),
        tar = target->get_pixel(tarx, tary),
        tar_up = target->get_pixel(tarx,tary-1),
        tar_bottom = target->get_pixel(tarx, tary+1),
        tar_left = target->get_pixel(tarx-1,tary),
        tar_right = target->get_pixel(tarx+1,tary);
    R(step-1)=0;G(step-1)=0;B(step-1)=0;
    // if target image's gradient bigger, use target pixel instead of source
    if (fabs(tar_up[0]-tar[0])>fabs(pixel_up[0]-pixel[0]))
        R(step-1) += float(tar_up[0] - tar[0]); 
    else
        R(step-1) += float(pixel_up[0] - pixel[0]);
    if (fabs(tar_bottom[0]-tar[0])>fabs(pixel_bottom[0]-pixel[0]))
        R(step-1) += float(tar_bottom[0] - tar[0]);
    else
        R(step-1) += float(pixel_bottom[0] - pixel[0]);
    if (fabs(tar_left[0]-tar[0])>fabs(pixel_left[0]-pixel[0]))
        R(step-1) += float(tar_left[0] - tar[0]);
    else
        R(step-1) += float(pixel_left[0] - pixel[0]);
    if (fabs(tar_right[0]-tar[0])>fabs(pixel_right[0]-pixel[0]))
        R(step-1) += float(tar_right[0] - tar[0]);
    else
        R(step-1) += float(pixel_right[0] - pixel[0]);

    if (fabs(tar_up[1]-tar[1])>fabs(pixel_up[1]-pixel[1]))
        G(step-1) += float(tar_up[1] - tar[1]); 
    else
        G(step-1) += pixel_up[1] - pixel[1];
    if (fabs(tar_bottom[1]-tar[1])>fabs(pixel_bottom[1]-pixel[1]))
        G(step-1) += float(tar_bottom[1] - tar[1]);
    else
        G(step-1) += pixel_bottom[1] - pixel[1];
    if (fabs(tar_left[1]-tar[1])>fabs(pixel_left[1]-pixel[1]))
        G(step-1) += float(tar_left[1] - tar[1]);
    else
        G(step-1) += float(pixel_left[1] - pixel[1]);
    if (fabs(tar_right[1]-tar[1])>fabs(pixel_right[1]-pixel[1]))
        G(step-1) += float(tar_right[1] - tar[1]);
    else
        G(step-1) += float(pixel_right[1] - pixel[1]);

    
    if (fabs(tar_up[2]-tar[2])>fabs(pixel_up[2]-pixel[2]))
        B(step-1) += float(tar_up[2] - tar[2]); 
    else
        B(step-1) += pixel_up[2] - pixel[2];
    if (fabs(tar_bottom[2]-tar[2])>fabs(pixel_bottom[2]-pixel[2]))
        B(step-1) += float(tar_bottom[2] - tar[2]);
    else
        B(step-1) += pixel_bottom[2] - pixel[2];
    if (fabs(tar_left[2]-tar[2])>fabs(pixel_left[2]-pixel[2]))
        B(step-1) += float(tar_left[2] - tar[2]);
    else
        B(step-1) += float(pixel_left[2] - pixel[2]);
    if (fabs(tar_right[2]-tar[2])>fabs(pixel_right[2]-pixel[2]))
        B(step-1) += float(tar_right[2] - tar[2]);
    else
        B(step-1) += float(pixel_right[2] - pixel[2]);

    if (flag[0]) //up
    {
        tList.push_back(T(index[x][y]-1, index[x][y-1]-1, 1.0f));
    }
    else
    {
        R(step-1) -= float(tar_up[0]);
        G(step-1) -= float(tar_up[1]);
        B(step-1) -= float(tar_up[2]);
    }
    if (flag[1]) // bottom
    {
        tList.push_back(T(index[x][y]-1, index[x][y+1]-1, 1.0f));
    }
    else
    {
        R(step-1) -= float(tar_bottom[0]);
        G(step-1) -= float(tar_bottom[1]);
        B(step-1) -= float(tar_bottom[2]);
    }
    if (flag[2]) // left
    {
        tList.push_back(T(index[x][y]-1, index[x-1][y]-1, 1.0f));
    }
    else
    {
        R(step-1) -= float(tar_left[0]);
        G(step-1) -= float(tar_left[1]);
        B(step-1) -= float(tar_left[2]);
    }
    if (flag[3]) // right
    {
        tList.push_back(T(index[x][y]-1, index[x+1][y]-1, 1.0f));
    }
    else
    {
        R(step-1) -= float(tar_right[0]);
        G(step-1) -= float(tar_right[1]);
        B(step-1) -= float(tar_right[2]);
    }

}

// after updating, we use tList to construct A and decompose A
void SeamlessMix::prepare()
{
    A.setFromTriplets(tList.begin(),tList.end());
    lu.compute(A);
}

// solving linear equation
void SeamlessMix::solve()
{
    resultG = lu.solve(G);
    resultB = lu.solve(B);
    resultR = lu.solve(R);
}

std::vector<unsigned char> SeamlessMix::get_result(int index)
{
    std::vector<unsigned char> new_rgb(3);
    resultR(index) = resultR(index)<=255 ? resultR(index):255.0f;
    resultG(index) = resultG(index)<=255 ? resultG(index):255.0f;
    resultB(index) = resultB(index)<=255 ? resultB(index):255.0f;
    new_rgb[0] = unsigned char(resultR(index)>0 ? resultR(index):0);
    new_rgb[1] = unsigned char(resultG(index)>0 ? resultG(index):0);
    new_rgb[2] = unsigned char(resultB(index)>0 ? resultB(index):0);
    return new_rgb;
}
}