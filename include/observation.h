#pragma once
#include <Eigen/Core>

class Observation {
public:
    // Constructor that initializes the observation with given camera ID, point ID, and pixel coordinates
    Observation(int camera_id, int point_id, const Eigen::Vector2d& pixel_coords);

    // Getters for observation parameters
    int getCameraId() const;
    int getPointId() const;
    const Eigen::Vector2d& getPixelCoords() const;

    // Returns a pointer to the raw data of the pixel coordinates vector
    double* getPixelCoordsRawDataPointer();

private:
    int camera_id_; // ID of the camera that captured the observation
    int point_id_; // ID of the 3D point being observed
    Eigen::Vector2d pixel_coords_; // Pixel coordinates of the observation in the image
};