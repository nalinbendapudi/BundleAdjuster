#include <ceres/ceres.h>
#include <iostream>

#include "bundle_adjuster.h"
#include "reprojection_error.h"

BundleAdjuster::BundleAdjuster(std::vector<Camera>& cameras,
                               std::vector<Point3D>& points3d,
                               std::vector<Observation>& observations)
        : cameras_(cameras), points3d_(points3d), observations_(observations) {}

bool BundleAdjuster::optimize() {
    ceres::Problem problem;

    for (const auto& observation : observations_) {
        Camera& camera = cameras_[observation.getCameraId()];
        Point3D& point3d = points3d_[observation.getPointId()];
        const Eigen::Vector2d& pixel = observation.getPixelCoords();

        // Create the cost function
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 2, 1, 3>(
            new ReprojectionError(pixel.x(), pixel.y()));

        // Add residual block
        problem.AddResidualBlock(cost_function,
                                 nullptr, // standard least-squares loss
                                 camera.getPositionRawDataPointer(),
                                 camera.getOrientationRawDataPointer(),
                                 camera.getDistortionCoeffsRawDataPointer(),
                                 camera.getFocalLengthRawDataPointer(),
                                 point3d.getCoordinatesRawDataPointer());

        // Set the parameter blocks to be constant (by uncommenting the corresponding line)
        // if they should not be optimized
        // problem.SetParameterBlockConstant(camera.getPositionRawDataPointer());
        // problem.SetParameterBlockConstant(camera.getOrientationRawDataPointer());
        // problem.SetParameterBlockConstant(camera.getDistortionCoeffsRawDataPointer());
        // problem.SetParameterBlockConstant(camera.getFocalLengthRawDataPointer());
        // problem.SetParameterBlockConstant(point3d.getCoordinatesRawDataPointer());
    }

    // Configure solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    return true;
}