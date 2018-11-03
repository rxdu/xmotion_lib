/* 
 * curve_viz.cpp
 * 
 * Created on: Oct 19, 2018 11:21
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/curve_viz.hpp"

#include <cmath>

#include "lightviz/details/geometry_draw.hpp"

using namespace librav;
using namespace CvDraw;

void LightViz::ShowCubicSpline(const CSpline &spline, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    canvas.SetupCanvas(-15, 15, -10, 10);

    GeometryDraw gdraw(canvas);

    gdraw.DrawCubicSpline(spline, step);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowCubicSpline(const std::vector<CSpline> &splines, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
}

void LightViz::ShowCubicSplinePosition(const std::vector<CSpline> &splines, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
}

void LightViz::ShowParametricCurve(const ParametricCurve &pcurve, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    canvas.SetupCanvas(-15, 15, -10, 10);

    GeometryDraw gdraw(canvas);

    gdraw.DrawParametricCurve(pcurve, step);

    ShowImage(canvas.paint_area, window_name, save_img);
}