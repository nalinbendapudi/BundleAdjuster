#include "point3d.h"

Point3D::Point3D(const Eigen::Vector3d& coordinates) : coordinates_(coordinates) {}

const Eigen::Vector3d& Point3D::getCoordinates() const {
    return coordinates_;
}

double* Point3D::getCoordinatesRawDataPointer() {
    return coordinates_.data();
}