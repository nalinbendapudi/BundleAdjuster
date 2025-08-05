#include <iostream>
#include "data_loader.h"
#include "camera.h"
#include "point3d.h"
#include "observation.h"
#include "bundle_adjuster.h"
#include "data_writer.h"

int main() {
    // // Create an instance of CheckLibs and run tests for each library    
    // CheckLibs checker;

    // std::cout << "Testing libraries..." << std::endl;
    // checker.testEigen();
    // checker.testOpenCV();
    // checker.testCeres();
    // checker.testPCL();

    // Read data from file
    std::string file_path = "data/problem-49-7776-pre.txt";
    DataLoader data_loader(file_path);
    std::vector<Camera> cameras;
    std::vector<Point3D> points3d;
    std::vector<Observation> observations;  
    if (!data_loader.loadData(cameras, points3d, observations)) {
        std::cerr << "Failed to load data from file: " << file_path << std::endl;
        return 1;
    }
    std::cout << "Data loaded successfully!" << std::endl;

    // // Print loaded data for verification
    // std::cout << "Number of cameras: " << cameras.size() << std::endl;
    // std::cout << "Number of points: " << points3d.size() << std::endl;
    // std::cout << "Number of observations: " << observations.size() << std::endl;
    // for (const auto& camera : cameras) {
    //     std::cout << "Camera Position: " << camera.getPosition().transpose() << std::endl;
    //     std::cout << "Camera Orientation: " << camera.getOrientation().transpose() << std::endl;
    //     std::cout << "Camera Distortion Coefficients: " << camera.getDistortionCoeffs().transpose() << std::endl;
    //     std::cout << "Camera Focal Length: " << camera.getFocalLength() << std::endl;
    // }
    // for (const auto& point : points3d) {
    //     std::cout << "Point Coordinates: " << point.getCoordinates().transpose() << std::endl;
    // }
    // for (const auto& observation : observations) {
    //     std::cout << "Observation Camera ID: " << observation.getCameraId()
    //               << ", Point ID: " << observation.getPointId()
    //               << ", Pixel Coordinates: " << observation.getPixelCoords().transpose() << std::endl;
    // }

    // Perform bundle adjustment optimization
    BundleAdjuster bundle_adjuster(cameras, points3d, observations);
    if (!bundle_adjuster.optimize()) {
        std::cerr << "Bundle adjustment optimization failed!" << std::endl;
        return 1;
    }
    std::cout << "Bundle adjustment optimization completed successfully!" << std::endl;

    // Print optimized data for verification
    std::cout << "Number of cameras: " << cameras.size() << std::endl;
    std::cout << "Number of points: " << points3d.size() << std::endl;
    for (const auto& camera : cameras) {
        std::cout << "Camera Position: " << camera.getPosition().transpose() << std::endl;
        std::cout << "Camera Orientation: " << camera.getOrientation().transpose() << std::endl;
        std::cout << "Camera Distortion Coefficients: " << camera.getDistortionCoeffs().transpose() << std::endl;
        std::cout << "Camera Focal Length: " << camera.getFocalLength() << std::endl;
    }
    for (const auto& point : points3d) {
        std::cout << "Point Coordinates: " << point.getCoordinates().transpose() << std::endl;
    }

    // Write the optimized data into a new file
    std::string output_file_path = "data/problem-49-7776-ba-out.txt";
    DataWriter data_writer(output_file_path);
    if (!data_writer.writeData(cameras, points3d, observations)) {
        std::cerr << "Failed to write optimized data to file: " << output_file_path << std::endl;
        return 1;
    }
    std::cout << "Optimized data written successfully to: " << output_file_path << std::endl;

    return 0;
}
