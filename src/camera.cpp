#include "camera.h"

Camera::Camera(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation,
                                   const Eigen::Vector2d& distortion_coeffs, double focal_length)
    : position_(position), orientation_(orientation), distortion_coeffs_(distortion_coeffs),
      focal_length_(focal_length) {}

      const Eigen::Vector3d& Camera::getPosition() const {
    return position_;
}   

const Eigen::Vector3d& Camera::getOrientation() const {
    return orientation_;
}

const Eigen::Vector2d& Camera::getDistortionCoeffs() const {
    return distortion_coeffs_;
}

double Camera::getFocalLength() const {
    return focal_length_;
}

double* Camera::getPositionRawDataPointer() {
    return position_.data();
}

double* Camera::getOrientationRawDataPointer() {
    return orientation_.data();
}

double* Camera::getDistortionCoeffsRawDataPointer() {
    return distortion_coeffs_.data();
}

double* Camera::getFocalLengthRawDataPointer() {
    return &focal_length_;
}