#pragma once

namespace USTC_CG
{
class Shape
{
   public:
    // Draw Settings
    struct Config
    {
        // Offset to convert canvas position to screen position
        float bias[2] = { 0.f, 0.f };
        // Line color in RGBA format
        unsigned char line_color[4] = {255, 0, 0, 255};
        float line_thickness = 2.0f;

        void setcolor(unsigned char *color)
        {
            line_color[0] = color[0];
            line_color[1] = color[1];
            line_color[2] = color[2];
            line_color[3] = color[3];
        }
    };
    /*
    * Setting a config for each shape so that we can change their line_color 
    * and line_thickness easily.
    */
    Config config;

   public:
    virtual ~Shape() = default;

    /**
     * Draws the shape on the screen.
     * This is a pure virtual function that must be implemented by all derived
     * classes.
     *
     * @param config The configuration settings for drawing, including line
     * color, thickness, and bias.
     *               - line_color defines the color of the shape's outline.
     *               - line_thickness determines how thick the outline will be.
     *               - bias is used to adjust the shape's position on the
     * screen.
     * 
     *  now we nolonger need config as parameter,because each shape has its config,
     *  we nolonger use same config.
     */
    virtual void draw() const = 0;
    /**
     * Updates the state of the shape.
     * This function allows for dynamic modification of the shape, in response
     * to user interactions like dragging.
     *
     * @param x, y Dragging point. e.g. end point of a line.
     */
    virtual void update(float x, float y) = 0;

    // add point while drawing polygon
    virtual void addpoint() = 0;
};
}  // namespace USTC_CG