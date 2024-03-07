#include "warpingRBF.h"
#include <cmath>

namespace USTC_CG
{
// function of R
float R(float x1, float y1, float x2, float y2)
{
    // R(d) = exp(-d*d)
    //return float(std::exp(-(x1-x2)*(x1-x2)-(y1-y2)*(y1-y2)));

    // R(d) = (d*d + r*r)^mu, r = 1, mu = 1
    return float(std::pow((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2) + 1,0.5));
}

std::pair<int, int> WarpingRBF::warping(int x, int y)
{
    size_t len = px.size();
    for (size_t i = 0; i < len; i++)
    {
        if (x == px[i] && y == py[i])
        {
            return std::make_pair(int(qx[i]), int(qy[i]));
        }
    }
    // calculate new_x & new_y
    float new_x = 0.0f, new_y = 0.0f;
    for (size_t i = 0; i < len; i++)
    {
        new_x += result[i] * R(float(x), float(y), px[i], py[i]);
        new_y += result[i + len] * R(float(x), float(y), px[i], py[i]);
    }
    new_x += result[2*len]*float(x) + result[2*len + 1]*float(y) + result[2*len + 4];
    new_y += result[2*len + 2]*float(x) + result[2*len + 3]*float(y) + result[2*len + 5];
    return std::make_pair(int(new_x), int(new_y));
}

void WarpingRBF::set_pq(std::vector<ImVec2> start_points, std::vector<ImVec2> end_points)
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

void WarpingRBF::set_hw(int h, int w)
{
}

void WarpingRBF::prepare()
{
    // This function can calculate the result before calculating new_x & new_y
    // In this way we can avoid repeating solving linear equations
    size_t len = px.size();
    Eigen::MatrixXf A(2*len + 6, 2*len + 6);
    Eigen::MatrixXf b(2*len + 6, 1);
    // set the matrix A & vector b
    for (size_t i = 0; i < len; i++)
    {
        for (size_t j = 0; j < len; j++)
        {
            A(i, j) = R(px[i], py[i], px[j], py[j]);
            A(i+len, j+len) = A(i, j);
            A(i+len, j) = 0;
            A(i, j+len) = 0;
        }
    }
    for (size_t i = 0; i < len; i++)
    {
        A(i, 2*len) = px[i];
        A(i, 2*len + 1) = py[i];
        A(i, 2*len + 2) = 0;
        A(i, 2*len + 3) = 0;
        A(i, 2*len + 4) = 1;
        A(i, 2*len + 5) = 0;
    }
    for (size_t i = len; i < 2*len; i++)
    {
        A(i, 2*len) = 0;
        A(i, 2*len + 1) = 0;
        A(i, 2*len + 2) = px[i-len];
        A(i, 2*len + 3) = py[i-len];
        A(i, 2*len + 4) = 0;
        A(i, 2*len + 5) = 1;
    }
    for (size_t j = 0; j < len; j++)
    {
        A(2*len, j) = px[j];
        A(2*len + 1, j) = py[j];
        A(2*len + 2, j) = 0;
        A(2*len + 3, j) = 0;
        A(2*len + 4, j) = 1;
        A(2*len + 5, j) = 0;
    }
    for (size_t j = len; j < 2*len; j++)
    {
        A(2*len, j) = 0;
        A(2*len + 1, j) = 0;
        A(2*len + 2, j) = px[j-len];
        A(2*len + 3, j) = py[j-len];
        A(2*len + 4, j) = 0;
        A(2*len + 5, j) = 1;
    }
    for (size_t i = 2*len; i < 2*len + 6; i++)
    {
        for (size_t j = 2*len; j < 2*len + 6; j++)
        {
            A(i, j) = 0;
        }
    }
    // set b
    for (size_t i = 0; i < len; i++)
    {
        b(i, 0) = qx[i];
    }
    for (size_t i = len; i < 2*len; i++)
    {
        b(i, 0) = qy[i-len];
    }
    for (size_t i = 2*len; i < 2*len + 6; i++)
    {
        b(i, 0) = 0;
    }
    Eigen::MatrixXf x_ = A.colPivHouseholderQr().solve(b);
    result.resize(2*len + 6);
    for(size_t i = 0; i < 2*len + 6; i++)
    {
        result[i] = x_(i, 0);
    }
}

}