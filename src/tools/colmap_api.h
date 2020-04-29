#ifndef COLMAP_COLMAP_API_H
#define COLMAP_COLMAP_API_H

#include <string>

namespace colmap {
    int ExtractFeatures(const std::string& project_path);
    int MatchFeaturesExhaustively(const std::string& project_path);
    int ReconstructSparse(const std::string& project_path);
}

#endif //COLMAP_COLMAP_API_H
