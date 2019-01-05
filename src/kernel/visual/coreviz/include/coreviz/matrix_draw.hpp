/* 
 * matrix_draw.hpp
 * 
 * Created on: Aug 10, 2018 09:32
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATRIX_DRAW_HPP
#define MATRIX_DRAW_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace librav
{
namespace LightViz
{
cv::Mat CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix, bool invert_y = false);
}
} // namespace librav

#endif /* MATRIX_DRAW_HPP */
