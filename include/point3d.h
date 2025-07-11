#pragma once
#include <Eigen/Core>

class Point3D {
public:
    // Constructor that initializes the point with an Eigen vector
    Point3D(const Eigen::Vector3d& coordinates);
    // Returns the coordinates of the point as an Eigen vector
    const Eigen::Vector3d& getCoordinates() const;
    // Returns the coordinates of the point as a raw double pointer
    double* getCoordinatesRawDataPointer();
private:
    Eigen::Vector3d coordinates_;
};