#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CheckLibs {
public:
    void testEigen();
    void testOpenCV();
    void testCeres();
    void testPCL();
};

