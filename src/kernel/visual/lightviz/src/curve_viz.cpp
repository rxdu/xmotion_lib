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

#include "coreviz/coreviz.hpp"

using namespace librav;

void LightViz::ShowCubicSpline(const CSpline &spline, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    canvas.Resize(-15, 15, -10, 10);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    GeometryDraw::DrawCubicSpline(canvas, spline, step);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowCubicSpline(const std::vector<CSpline> &splines, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
}

void LightViz::ShowCubicSplinePosition(const std::vector<CSpline> &splines, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
}

void LightViz::ShowParametricCurve(const ParametricCurve &pcurve, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    canvas.Resize(-15, 15, -10, 10);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    GeometryDraw::DrawParametricCurve(canvas, pcurve, step);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}
