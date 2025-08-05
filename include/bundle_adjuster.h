# pragma once

#include "camera.h"
#include "point3d.h"
#include "observation.h"

class BundleAdjuster {
public:
    BundleAdjuster(std::vector<Camera>& cameras,
                   std::vector<Point3D>& points3d,
                   std::vector<Observation>& observations);
    bool optimize();

private:
    std::vector<Camera>& cameras_; // Reference to the vector of cameras
    std::vector<Point3D>& points3d_; // Reference to the vector of 3D points
    std::vector<Observation>& observations_; // Reference to the vector of observations
};