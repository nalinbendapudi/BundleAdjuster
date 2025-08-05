#pragma once
#include "ceres/rotation.h"

class ReprojectionError {
public:
    ReprojectionError(double observed_image_point_x, double observed_image_point_y)
        : observed_image_point_x_(observed_image_point_x), observed_image_point_y_(observed_image_point_y) {}

    // Since this is a templated method, it needs to be implemented in the header file.
    template <typename T>
    bool operator()(const T* const camera_position,
                const T* const camera_orientation,
                const T* const distortion_coeffs,
                const T* const focal_length,
                const T* const point3d,
                T* residuals) const {
        
        T transformed_point3d[3];
        // Applying rotation
        ceres::AngleAxisRotatePoint(camera_orientation, point3d, transformed_point3d);
        // Applying translation
        transformed_point3d[0] += camera_position[0];
        transformed_point3d[1] += camera_position[1];
        transformed_point3d[2] += camera_position[2];

        // Projecting the point onto the image plane
        T projected_point_x = -transformed_point3d[0] / transformed_point3d[2];
        T projected_point_y = -transformed_point3d[1] / transformed_point3d[2];

        // Calculatinf radial distortion
        T radial_distance = projected_point_x * projected_point_x + projected_point_y * projected_point_y;
        T radial_distortion = T(1.0) +  radial_distance * (distortion_coeffs[0] + distortion_coeffs[1] * radial_distance);

        // Predicting the image point
        T predicted_image_point_x = focal_length[0] * radial_distortion * projected_point_x;
        T predicted_image_point_y = focal_length[0] * radial_distortion * projected_point_y;
        
        // Calculating residuals
        residuals[0] = predicted_image_point_x - T(observed_image_point_x_);
        residuals[1] = predicted_image_point_y - T(observed_image_point_y_);

        return true;
    }


    double observed_image_point_x_;
    double observed_image_point_y_;
};
