#pragma once

#include <string>
#include <vector>

#include "camera.h"
#include "point3d.h"
#include "observation.h"

class DataWriter {
public:
    DataWriter(const std::string& file_path);

    bool writeData(const std::vector<Camera>& cameras,
                   const std::vector<Point3D>& points3d,
                   const std::vector<Observation>& observations);
private:
    std::string file_path_;
};