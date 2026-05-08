#ifndef TRAVEL_POINT_TYPES_HPP
#define TRAVEL_POINT_TYPES_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define INVALID_IDX -1

struct EIGEN_ALIGN16 PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint16_t, id, id))

using PointT = PointXYZILID;
using num_t  = float;

inline void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr dst) {
  dst->points.clear();
  for (const auto& pt : src.points) {
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x         = pt.x;
    pt_xyzi.y         = pt.y;
    pt_xyzi.z         = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }
}

#endif  // TRAVEL_POINT_TYPES_HPP
