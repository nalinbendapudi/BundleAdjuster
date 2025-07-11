#pragma once
#include <Eigen/Core>

class Camera {
public:
    // Constructor that initializes the camera parameters with given values
    Camera(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation,
                 const Eigen::Vector2d& distortion_coeffs, double focal_length);
    
    // Getters for camera parameters
    const Eigen::Vector3d& getPosition() const;
    const Eigen::Vector3d& getOrientation() const;
    const Eigen::Vector2d& getDistortionCoeffs() const;
    double getFocalLength() const;

    // Returns a pointer to the raw data of the position vector
    double* getPositionRawDataPointer();
    // Returns a pointer to the raw data of the orientation vector
    double* getOrientationRawDataPointer();   
    // Returns a pointer to the raw data of the distortion coefficients vector
    double* getDistortionCoeffsRawDataPointer();
    // Returns a pointer to the raw data of the focal length
    double* getFocalLengthRawDataPointer();

private:
    Eigen::Vector3d position_;
    Eigen::Vector3d orientation_;
    Eigen::Vector2d distortion_coeffs_;
    double focal_length_;
};