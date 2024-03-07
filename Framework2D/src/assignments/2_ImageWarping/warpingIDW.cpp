#include "warpingIDW.h"
#include <cmath>

namespace USTC_CG
{

std::pair<int, int> WarpingIDW::warping(int x, int y)
{
    float mu = 2.0f;
    size_t len = px.size();
    std::vector<float> sigma(len), w(len);
    float sum_sigma = 0.0f;
    std::vector<float> sum_w_pxpx(len), sum_w_pxpy(len), sum_w_pypy(len),
        sum_pxqx(len), sum_pxqy(len), sum_pyqx(len), sum_pyqy(len);
    // if input x,y = p, then output q
    // calculation
    for (size_t i = 0; i < len; i++)
    {
        if (x == px[i] && y == py[i])
        {
            return std::make_pair(int(qx[i]), int(qy[i]));
        }
        else
        {
            sigma[i] = 1.0f / float(std::pow((px[i]-float(x))*(px[i]-float(x))+(py[i]-float(y))*(py[i]-float(y)),mu/2));
            sum_sigma += sigma[i];
        }
    }
    for (size_t i = 0; i < len; i++)
    {
        w[i] = sigma[i] / sum_sigma;
    }

    // calculate D_i
    for (size_t i = 0; i < len; i++)
    {
        for (size_t j = 0; j < len; j++)
        {
            sum_w_pxpx[i] += w[j] * (px[j] - px[i]) * (px[j] - px[i]);
            sum_w_pxpy[i] += w[j] * (px[j] - px[i]) * (py[j] - py[i]);
            sum_w_pypy[i] += w[j] * (py[j] - py[i]) * (py[j] - py[i]);
            sum_pxqx[i] += w[j] * (px[j] - px[i]) * (qx[j] - qx[i]);
            sum_pxqy[i] += w[j] * (px[j] - px[i]) * (qy[j] - qy[i]);
            sum_pyqx[i] += w[j] * (py[j] - py[i]) * (qx[j] - qx[i]);
            sum_pyqy[i] += w[j] * (py[j] - py[i]) * (qy[j] - qy[i]);
        }
    }
    float new_x = 0.0f, new_y = 0.0f;
    // calculate new x,y
    for (size_t i = 0; i < len; i++)
    {
        float delta_x = float(x) - qx[i], delta_y = float(y) - qy[i];
        new_x += w[i]*qx[i] + w[i]* (delta_x*(sum_w_pypy[i]*sum_pxqx[i]-sum_w_pxpy[i]*sum_pxqy[i]) + 
            delta_y*(sum_w_pypy[i]*sum_pyqx[i]-sum_w_pxpy[i]*sum_pyqy[i])) 
            / (sum_w_pxpx[i]*sum_w_pypy[i] - sum_w_pxpy[i]*sum_w_pxpy[i]);
        new_y += w[i]*qy[i] + w[i] * (delta_x*(sum_w_pxpx[i]*sum_pxqy[i]-sum_w_pxpy[i]*sum_pxqx[i]) + 
            delta_y*(sum_w_pxpx[i]*sum_pyqy[i]-sum_w_pxpy[i]*sum_pyqx[i])) 
            / (sum_w_pxpx[i]*sum_w_pypy[i] - sum_w_pxpy[i]*sum_w_pxpy[i]);
    }
    return std::make_pair(int(new_x), int(new_y));
}

void WarpingIDW::set_pq(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points)
{
    size_t l = start_points.size();
    px.resize(l);
    py.resize(l);
    qx.resize(l);
    qy.resize(l);
    for (size_t i = 0; i < l; i++)
    {
        px[i] = start_points[i].x;
        py[i] = start_points[i].y;
        qx[i] = end_points[i].x;
        qy[i] = end_points[i].y;
    }
}

void WarpingIDW::set_hw(int h, int w)
{
}

void WarpingIDW::prepare()
{
    
}
}