#pragma once

#include "shape.h"
#include <vector>

namespace USTC_CG
{
    class Polygon : public Shape
    {
        public:
         Polygon() = default;

         // Initialize a polygon with its start point
         Polygon(float current_point_x, float current_point_y)
         :current_point_x(current_point_x), current_point_y(current_point_y)
         {
            addpoint();
         }

         virtual ~Polygon() = default;

         // Draws the polygon on the screen
         // Overrides draw function to implement polygon-specific drawing logic
         void draw() const override;

         // Overrides Shape's update function to adjust the polygon size during
         // interaction
         void update(float x, float y) override;

         // add current point to the vertex of the polygon
         void addpoint();

        private:
         std::vector<float> point_x{};
         std::vector<float> point_y{};
         // current point to be added
         float current_point_x = 0.0f;
         float current_point_y = 0.0f;

    };
}