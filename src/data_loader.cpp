#include <fstream>
#include <sstream>
#include <iostream>

#include "data_loader.h"

DataLoader::DataLoader(const std::string& file_path) : file_path_(file_path) {}

// Load data from txt file into a vector of cameras, points, and observations

bool DataLoader::loadData(std::vector<Camera>& cameras, std::vector<Point3D>& points3d, std::vector<Observation>& observations) {
    std::ifstream file(file_path_);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << file_path_ << std::endl;
        return false;
    }

    std::string line;
    int line_count = 0;

    // First line contains the number of cameras, points, and observations
    line_count++;
    if (!std::getline(file, line)) {
        std::cerr << "Error: Could not read the first line of the file." << std::endl;
        return false;
    }
    std::istringstream firstLineStream(line);
    int numCameras, numPoints, numObservations;
    if (!(firstLineStream >> numCameras >> numPoints >> numObservations)) {
        std::cerr << "Error: Invalid format in the first line of the file." << std::endl;
        return false;
    }
    cameras.reserve(numCameras);
    points3d.reserve(numPoints);
    observations.reserve(numObservations);
    
    // Read Observations
    for (int i = 0; i < numObservations; ++i) {
        line_count++;
        if (!std::getline(file, line)) {
            std::cerr << "Error: Could not read observation. Line: " << line_count << std::endl;
            return false;
        }
        std::istringstream obsStream(line);
        int cam_id, pt_id;
        Eigen::Vector2d pixel_coords;

        std::istringstream iss(line);
        if (iss >> cam_id >> pt_id >> pixel_coords[0] >> pixel_coords[1]) {
            observations.emplace_back(cam_id, pt_id, pixel_coords);
        } else {
            std::cerr << "Error: Invalid format in observation. Line: " << line_count << std::endl;
            return false;
        }
    }

    // Read Cameras
    for (int i = 0; i < numCameras; ++i) {
        Eigen::Vector3d position;
        Eigen::Vector3d orientation;
        Eigen::Vector2d distortion_coeffs;
        double focal_length;

        // Read position (3 lines)
        for (int j = 0; j < 3; ++j) {
            line_count++;
            if (!std::getline(file, line)) {
                std::cerr << "Error: Could not read camera position. Line: " << line_count << std::endl;
                return false;
            }
            std::istringstream iss(line);
            if (!(iss >> position[j])) {
                std::cerr << "Error: Invalid format in camera position. Line: " << line_count << std::endl;
                return false;
            }
        }

        // Read orientation (3 lines)
        for (int j = 0; j < 3; ++j) {
            line_count++;
            if (!std::getline(file, line)) {
                std::cerr << "Error: Could not read camera orientation. Line: " << line_count << std::endl;
                return false;
            }
            std::istringstream iss(line);
            if (!(iss >> orientation[j])) {
                std::cerr << "Error: Invalid format in camera orientation. Line: " << line_count << std::endl;
                return false;
            }
        }

        // Read distortion coefficients (2 lines)
        for (int j = 0; j < 2; ++j) {
            line_count++;
            if (!std::getline(file, line)) {
                std::cerr << "Error: Could not read camera distortion. Line: " << line_count << std::endl;
                return false;
            }
            std::istringstream iss(line);
            if (!(iss >> distortion_coeffs[j])) {
                std::cerr << "Error: Invalid format in camera distortion. Line: " << line_count << std::endl;
                return false;
            }
        }

        // Read focal length (1 line)
        line_count++;
        if (!std::getline(file, line)) {
            std::cerr << "Error: Could not read camera focal length. Line: " << line_count << std::endl;
            return false;
        }
        std::istringstream iss(line);
        if (!(iss >> focal_length)) {
            std::cerr << "Error: Invalid format in camera focal length. Line: " << line_count << std::endl;
            return false;
        }

        cameras.emplace_back(position, orientation, distortion_coeffs, focal_length);
    }

    // Read Points
    for (int i = 0; i < numPoints; ++i) {
        Eigen::Vector3d point_coords;
        
        for (int j = 0; j < 3; ++j) {
            line_count++;
            if (!std::getline(file, line)) {
                std::cerr << "Error: Could not read point3d coordinates. Line: " << line_count << std::endl;
                return false;
            }
            std::istringstream iss(line);
            if (!(iss >> point_coords[j])) {
                std::cerr << "Error: Invalid format in point3d coordinates. Line: " << line_count << std::endl;
                return false;
            }
        }
        points3d.emplace_back(point_coords);
    }

    // Close the file
    file.close();
    return true; // Data loaded successfully
}
