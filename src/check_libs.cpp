#include "check_libs.h"
#include <iostream>
#include <cmath>

struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = x[0]*x[0] - T(4.0);
        return true;
    }
};

void CheckLibs::testCeres() {
    double x = 0.5;
    ceres::Problem problem;

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor),
        nullptr,
        &x
    );

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    if (std::abs(x - 2.0) < 1e-3 || std::abs(x + 2.0) < 1e-3) {
        std::cout << "Ceres is working correctly." << std::endl;
    } else {
        std::cerr << "Ceres test failed!" << std::endl;
    }
}

void CheckLibs::testEigen() {
    Eigen::Matrix2d mat;
    mat << 1, 2,
           3, 4;
    Eigen::Vector2d vec(1, 1);
    Eigen::Vector2d result = mat * vec;

    if (result == Eigen::Vector2d(3, 7)) {
        std::cout << "Eigen is working correctly." << std::endl;
    } else {
        std::cerr << "Eigen test failed!" << std::endl;
    }
}

void CheckLibs::testOpenCV() {
    cv::Mat img = cv::Mat::zeros(3, 3, CV_8UC1);
    img.at<uchar>(1,1) = 255;

    if (img.at<uchar>(1,1) == 255) {
        std::cout << "OpenCV is working correctly." << std::endl;
    } else {
        std::cerr << "OpenCV test failed!" << std::endl;
    }
}

void CheckLibs::testPCL() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 1;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.points[0].x = 1.0f;
    cloud.points[0].y = 2.0f;
    cloud.points[0].z = 3.0f;

    const auto& pt = cloud.points[0];
    if (pt.x == 1.0f && pt.y == 2.0f && pt.z == 3.0f) {
        std::cout << "PCL is working correctly." << std::endl;
    } else {
        std::cerr << "PCL test failed!" << std::endl;
    }
}
