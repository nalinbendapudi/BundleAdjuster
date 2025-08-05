#include <iostream>
#include <fstream>

#include "data_writer.h"

DataWriter::DataWriter(const std::string& file_path) : file_path_(file_path) {}

bool DataWriter::writeData(const std::vector<Camera>& cameras,
                          const std::vector<Point3D>& points3d,
                          const std::vector<Observation>& observations) {
    
    std::ofstream file(file_path_);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << file_path_ << std::endl;
        return false;
    }

    // Write the number of observations, cameras, and points
    file << cameras.size() << " " << points3d.size() << " " << observations.size() << "\n";

    // Write observations
    for (const auto& observation : observations) {
        const Eigen::Vector2d& pixel_coords = observation.getPixelCoords();
        file << observation.getCameraId() << " " << observation.getPointId() << " " << pixel_coords.x() << " " << pixel_coords.x() << "\n";
    }

    // Write cameras
    for (const auto& camera : cameras) {
        const Eigen::Vector3d& position = camera.getPosition();
        const Eigen::Vector3d& orientation = camera.getOrientation();
        const Eigen::Vector2d& distortion_coeffs = camera.getDistortionCoeffs();
        double focal_length = camera.getFocalLength();

        file << position.x() << "\n" << position.y() << "\n" << position.z() << "\n";
        file << orientation.x() << "\n" << orientation.y() << "\n" << orientation.z() << "\n";
        file << distortion_coeffs.x() << "\n" << distortion_coeffs.y() << "\n";
        file << focal_length << "\n";
    }

    // Write points
    for (const auto& point3d : points3d) {
        const Eigen::Vector3d& point_coords = point3d.getCoordinates();
        file << point_coords.x() << "\n" << point_coords.y() << "\n" << point_coords.z() << "\n";
    }
    
    file.close();
    return true;
}
