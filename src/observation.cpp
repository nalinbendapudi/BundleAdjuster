#include "observation.h"

Observation::Observation(int camera_id, int point_id, const Eigen::Vector2d& pixel_coords)
    : camera_id_(camera_id), point_id_(point_id), pixel_coords_(pixel_coords) {}

int Observation::getCameraId() const {
    return camera_id_;
}

int Observation::getPointId() const {
    return point_id_;
}

const Eigen::Vector2d& Observation::getPixelCoords() const {
    return pixel_coords_;
}

double* Observation::getPixelCoordsRawDataPointer() {
    return pixel_coords_.data();
}