#pragma once

#include "shape.h"
#include <vector>

namespace USTC_CG
{
    class Freehand : public Shape
    {
        public:
         Freehand() = default;

         // Initialize a polygon with its start point
         Freehand(float current_point_x, float current_point_y)
         :current_point_x(current_point_x), current_point_y(current_point_y)
         {
            addpoint();
         }

         virtual ~Freehand() = default;

         // Draws the freehand on the screen
         // Overrides draw function to implement freehand-specific drawing logic
         void draw() const override;

         // Overrides Shape's update function to adjust the freehand figure
         // interaction
         void update(float x, float y) override;

         // add current point to the freehand figure
         void addpoint();


        private:
         float current_point_x = 0.0f;
         float current_point_y = 0.0f;

         std::vector<float> point_x{};
         std::vector<float> point_y{};
    };
}