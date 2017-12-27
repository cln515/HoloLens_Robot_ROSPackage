/*
Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/


#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#ifndef LINEARSOLVER
#define LINEARSOLVER

void linearCompute(std::vector<cv::Point2f> ,std::vector<cv::Point2f>,Eigen::Matrix2d& R,Eigen::Vector2d& T);




#endif