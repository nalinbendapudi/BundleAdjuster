#include <iostream>
#include "data_loader.h"
#include "camera.h"
#include "point3d.h"
#include "observation.h"

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


    return 0;
}
