#ifndef TRAVEL_TYPES_HPP
#define TRAVEL_TYPES_HPP

// Self-contained replacements for the small slice of PCL the algorithm
// previously borrowed. Keeping the core PCL-free lets `pip install travel-seg`
// produce prebuilt wheels for macOS / Linux / Windows without dragging in a
// PCL system install at every install site.
//
// What we replicate:
//   - travel::PCLHeader            ↔ pcl::PCLHeader   (POD timestamp/frame)
//   - travel::PointCloud<T>        ↔ pcl::PointCloud<T>  (vector-with-header)
//   - travel::PointXYZ / XYZI      ↔ pcl::PointXYZ / pcl::PointXYZI
//   - PointXYZILID                 (custom point type used by TGS/AOS)
//   - travel::computeMeanAndCovarianceMatrix
//                                  ↔ pcl::computeMeanAndCovarianceMatrix
//
// The cov-matrix routine ports PCL's accumulator pattern bit-for-bit, in
// exactly the same float order, so KITTI regression bit-identity is
// preserved. See cpp/tests/data/kitti00_000000_gold.bin.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

// MSVC's <cmath> does not define M_PI by default (POSIX/GNU extension);
// the algorithm uses M_PI in several places, so we define it here once,
// in the header that every other core header transitively includes.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace travel {

// PCL's PCLHeader is essentially this. Some PCL code reads `seq`, `stamp`,
// `frame_id`; we only need it as a passthrough for cloud_header_ inside TGS.
struct PCLHeader {
    std::uint32_t seq    = 0;
    std::uint64_t stamp  = 0;
    std::string   frame_id;
};

// PCL-compatible vector-with-header container. The API is intentionally
// minimal — only the surface our algorithm and tests actually call.
template <typename PointT>
struct PointCloud {
    using value_type = PointT;
    using Ptr        = std::shared_ptr<PointCloud<PointT>>;
    using ConstPtr   = std::shared_ptr<const PointCloud<PointT>>;

    PCLHeader            header;
    std::vector<PointT>  points;
    std::uint32_t        width    = 0;
    std::uint32_t        height   = 1;
    bool                 is_dense = true;

    PointCloud() = default;
    PointCloud(const PointCloud&)     = default;
    PointCloud(PointCloud&&) noexcept = default;
    PointCloud& operator=(const PointCloud&)     = default;
    PointCloud& operator=(PointCloud&&) noexcept = default;

    std::size_t size()  const noexcept { return points.size(); }
    bool        empty() const noexcept { return points.empty(); }

    void clear()          noexcept { points.clear(); width = 0; }
    void reserve(std::size_t n)    { points.reserve(n); }
    void resize(std::size_t n)     { points.resize(n); width = static_cast<std::uint32_t>(points.size()); }

    void push_back(const PointT& p)    { points.push_back(p);    width = static_cast<std::uint32_t>(points.size()); }
    void push_back(PointT&& p)         { points.push_back(std::move(p)); width = static_cast<std::uint32_t>(points.size()); }
    template <typename... Args>
    void emplace_back(Args&&... args)  { points.emplace_back(std::forward<Args>(args)...); width = static_cast<std::uint32_t>(points.size()); }

    PointT&       operator[](std::size_t i)       { return points[i]; }
    const PointT& operator[](std::size_t i) const { return points[i]; }
    PointT&       at(std::size_t i)               { return points.at(i); }
    const PointT& at(std::size_t i) const         { return points.at(i); }

    typename std::vector<PointT>::iterator       begin()       { return points.begin(); }
    typename std::vector<PointT>::iterator       end()         { return points.end(); }
    typename std::vector<PointT>::const_iterator begin() const { return points.begin(); }
    typename std::vector<PointT>::const_iterator end()   const { return points.end(); }

    PointCloud<PointT>& operator+=(const PointCloud<PointT>& other) {
        points.insert(points.end(), other.points.begin(), other.points.end());
        width = static_cast<std::uint32_t>(points.size());
        return *this;
    }
};

// Plain XYZ point — used by KittiLoader's intermediate cloud.
struct PointXYZ {
    float x{0.0f}, y{0.0f}, z{0.0f};
};

// XYZ + intensity — KittiLoader output.
struct PointXYZI {
    float x{0.0f}, y{0.0f}, z{0.0f};
    float intensity{0.0f};
};

// Port of PCL's `pcl::computeMeanAndCovarianceMatrix(cloud, cov, centroid)`.
//
// Same accumulator order, same single-pass formulation, same Eigen storage
// layout (1x9 row-major) as PCL's specialization for pcl::PointCloud<T>.
// Verified bit-identical against the pre-port build via the KITTI gold
// regression in cpp/tests/regression_kitti.cpp.
template <typename PointT>
inline unsigned int computeMeanAndCovarianceMatrix(
    const PointCloud<PointT>& cloud,
    Eigen::Matrix3f&          covariance_matrix,
    Eigen::Vector4f&          centroid)
{
    if (cloud.empty()) return 0u;

    Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
    std::size_t point_count = 0;

    if (cloud.is_dense) {
        point_count = cloud.size();
        for (const auto& p : cloud.points) {
            accu[0] += p.x * p.x;
            accu[1] += p.x * p.y;
            accu[2] += p.x * p.z;
            accu[3] += p.y * p.y;
            accu[4] += p.y * p.z;
            accu[5] += p.z * p.z;
            accu[6] += p.x;
            accu[7] += p.y;
            accu[8] += p.z;
        }
    } else {
        for (const auto& p : cloud.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            accu[0] += p.x * p.x;
            accu[1] += p.x * p.y;
            accu[2] += p.x * p.z;
            accu[3] += p.y * p.y;
            accu[4] += p.y * p.z;
            accu[5] += p.z * p.z;
            accu[6] += p.x;
            accu[7] += p.y;
            accu[8] += p.z;
            ++point_count;
        }
    }

    accu /= static_cast<float>(point_count);

    centroid[0] = accu[6];
    centroid[1] = accu[7];
    centroid[2] = accu[8];
    centroid[3] = 1.0f;

    // Eigen::Matrix3f is column-major by default; coeffRef indexing matches
    // the PCL code we ported from. The mapping: 0=(0,0), 1=(1,0), 2=(2,0),
    // 3=(0,1), 4=(1,1), 5=(2,1), 6=(0,2), 7=(1,2), 8=(2,2).
    covariance_matrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
    covariance_matrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
    covariance_matrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
    covariance_matrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
    covariance_matrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
    covariance_matrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
    covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
    covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
    covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);

    return static_cast<unsigned int>(point_count);
}

}  // namespace travel

// PointXYZILID stays in the global namespace because the algorithm
// templates and downstream call sites have always referred to it bare.
// The PCL macros (PCL_ADD_POINT4D, POINT_CLOUD_REGISTER_POINT_STRUCT) are
// dropped — TRAVEL never relied on PCL's runtime point-field registry,
// only on the field layout matching pcl::PointXYZI. The 16-byte first
// chunk preserves the PCL_ADD_POINT4D layout (3 floats + 1 padding/data[3])
// so any downstream code that read PointXYZILID via a memcpy of XYZ still
// sees the same bytes.
struct alignas(16) PointXYZILID {
    union {
        float data[4];
        struct { float x, y, z; float data_w; };
    };
    float          intensity;
    std::uint16_t  label;
    std::uint16_t  id;
};

// `using PointT = PointXYZILID;` was set in the old utils.hpp. Keep the
// alias here so existing call sites (especially in cpp/examples and ROS
// wrapper) compile unchanged.
using PointT = PointXYZILID;

#endif  // TRAVEL_TYPES_HPP
