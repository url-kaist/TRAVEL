#ifndef TRAVEL_POINT_TYPES_HPP
#define TRAVEL_POINT_TYPES_HPP

// Backwards-compatibility shim. point_types.hpp used to define PointXYZILID
// (with PCL macros) and PointXYZILID2XYZI in the global namespace. The
// algorithm core is now PCL-free; PointXYZILID lives in travel/types.hpp.
// Existing #include "travel/point_types.hpp" call sites keep working
// without code changes.

#include "travel/types.hpp"

#define INVALID_IDX -1

using num_t = float;

inline void PointXYZILID2XYZI(travel::PointCloud<PointXYZILID>&        src,
                              travel::PointCloud<travel::PointXYZI>::Ptr dst) {
    dst->points.clear();
    for (const auto& pt : src.points) {
        travel::PointXYZI pt_xyzi;
        pt_xyzi.x         = pt.x;
        pt_xyzi.y         = pt.y;
        pt_xyzi.z         = pt.z;
        pt_xyzi.intensity = pt.intensity;
        dst->points.push_back(pt_xyzi);
    }
}

#endif  // TRAVEL_POINT_TYPES_HPP
