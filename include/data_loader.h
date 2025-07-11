#pragma once
#include <vector>
#include <string>

#include "camera.h"
#include "point3d.h"
#include "observation.h"

class DataLoader {
public:
    DataLoader(const std::string& file_path);
    bool loadData(std::vector<Camera>& cameras, std::vector<Point3D>& points3d, std::vector<Observation>& observations);

    private:
    std::string file_path_;
};