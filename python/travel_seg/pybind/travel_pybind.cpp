// pybind11 bindings for TRAVEL ground / object segmentation.
//
// Design notes:
// - The C++ algorithm classes are templated on a custom point type
//   (PointXYZILID) that carries XYZ + intensity + label + id. Python users
//   work with plain numpy arrays of shape (N, 3) or (N, 4), so this module
//   handles the conversion both ways.
// - estimateGround() and segmentObjects() return *new* point clouds, not
//   per-input-index labels. The bindings reconstruct the per-input mapping
//   via a kd-tree nearest-neighbour lookup against the ORIGINAL input cloud
//   (squared-distance epsilon ~ 1e-5). This mirrors what the C++
//   `saveLabels()` helper already does for KITTI .label exports.
// - segmentObjects() mutates its input cloud in-place (sphericalProjection
//   does `*cloud_in = *valid_cloud`). The binding makes a defensive copy of
//   the original input before passing it in, so the lookup is against the
//   pristine cloud.

#define PCL_NO_PRECOMPILE

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


#include "travel/3rdparty/nanoflann.hpp"
#include "travel/3rdparty/nanoflann_utils.hpp"
#include "travel/aos.hpp"
#include "travel/point_types.hpp"
#include "travel/tgs.hpp"

namespace py = pybind11;
using namespace py::literals;

namespace {

using Cloud    = travel::PointCloud<PointXYZILID>;
using CloudPtr = Cloud::Ptr;

// Convert numpy [N,3] (XYZ) or [N,4] (XYZI) float32 array -> PCL cloud.
CloudPtr numpyToCloud(py::array_t<float, py::array::c_style | py::array::forcecast> arr) {
    auto info = arr.request();
    if (info.ndim != 2) {
        throw std::invalid_argument(
            "expected 2D float32 array, got " + std::to_string(info.ndim) + "D");
    }
    if (info.shape[1] != 3 && info.shape[1] != 4) {
        throw std::invalid_argument(
            "expected (N, 3) for XYZ or (N, 4) for XYZI, got shape (" +
            std::to_string(info.shape[0]) + ", " + std::to_string(info.shape[1]) + ")");
    }
    const py::ssize_t N    = info.shape[0];
    const py::ssize_t cols = info.shape[1];
    const float*  data = static_cast<const float*>(info.ptr);

    CloudPtr cloud(new Cloud());
    cloud->reserve(static_cast<size_t>(N));
    for (py::ssize_t i = 0; i < N; ++i) {
        PointXYZILID p{};
        p.x         = data[i * cols + 0];
        p.y         = data[i * cols + 1];
        p.z         = data[i * cols + 2];
        p.intensity = (cols == 4) ? data[i * cols + 3] : 0.0f;
        p.label     = 0;
        p.id        = 0;
        cloud->push_back(p);
    }
    return cloud;
}

// Wrap an XYZ kd-tree built from a PCL cloud's coordinates so we can ask
// "for each query point, was there an identical (eps-close) reference?".
class XyzKdTree {
public:
    explicit XyzKdTree(const Cloud& ref) {
        pts_.pts.resize(ref.size());
        for (size_t i = 0; i < ref.size(); ++i) {
            pts_.pts[i].x = ref[i].x;
            pts_.pts[i].y = ref[i].y;
            pts_.pts[i].z = ref[i].z;
        }
        index_ = std::make_unique<KdTree>(3, pts_, kdtree_params_);
    }

    // Returns (found, ref_index) for the single nearest neighbour.
    bool nearest(float x, float y, float z, float eps_sq, uint32_t& out_idx) const {
        if (pts_.pts.empty()) return false;
        const float pt[3] = {x, y, z};
        size_t   num_results = 1;
        uint32_t ret_idx     = 0;
        float    out_dist_sq = 0.f;
        num_results = index_->knnSearch(pt, num_results, &ret_idx, &out_dist_sq);
        if (num_results > 0 && out_dist_sq < eps_sq) {
            out_idx = ret_idx;
            return true;
        }
        return false;
    }

private:
    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>,
        PointCloud<float>,
        3>;

    PointCloud<float>       pts_;
    nanoflann::KDTreeSingleIndexAdaptorParams kdtree_params_{10};
    std::unique_ptr<KdTree> index_;
};

constexpr float kEpsSq = 1e-5f;  // Same threshold as travel::saveLabels.

// Build a bool[N] mask: true where each query point has a match in `ref`.
py::array_t<bool> maskByNearest(const Cloud& ref, const Cloud& query) {
    XyzKdTree tree(ref);
    py::array_t<bool> result(static_cast<py::ssize_t>(query.size()));
    auto buf = result.mutable_unchecked<1>();
    uint32_t idx = 0;
    for (size_t i = 0; i < query.size(); ++i) {
        buf(i) = tree.nearest(query[i].x, query[i].y, query[i].z, kEpsSq, idx);
    }
    return result;
}

// Build an int32[N] label array: copy `ref[match].id` for matched query
// points, leave 0 otherwise.
py::array_t<int32_t> labelByNearest(const Cloud& ref, const Cloud& query) {
    XyzKdTree tree(ref);
    py::array_t<int32_t> result(static_cast<py::ssize_t>(query.size()));
    auto buf = result.mutable_unchecked<1>();
    uint32_t idx = 0;
    for (size_t i = 0; i < query.size(); ++i) {
        if (tree.nearest(query[i].x, query[i].y, query[i].z, kEpsSq, idx)) {
            buf(i) = static_cast<int32_t>(ref[idx].id);
        } else {
            buf(i) = 0;
        }
    }
    return result;
}

}  // namespace

PYBIND11_MODULE(_travel_seg, m) {
    m.doc() =
        "pybind11 bindings for the TRAVEL traversable-ground / object "
        "segmentation library.";
    m.attr("__version__") = "1.1.0";

    // -- TravelGroundSeg ------------------------------------------------------
    py::class_<travel::TravelGroundSeg<PointXYZILID>>(m, "_TravelGroundSeg")
        .def(py::init<>())
        .def("set_params",
             [](travel::TravelGroundSeg<PointXYZILID>& self,
                double max_range, double min_range, double resolution,
                int    num_iter,  int    num_lpr,    int    num_min_pts,
                double th_seeds,  double th_dist,    double th_outlier,
                double th_normal, double th_weight,
                double th_lcc_normal, double th_lcc_planar, double th_obstacle,
                bool   refine_mode) {
                 self.setParams(max_range, min_range, resolution,
                                num_iter, num_lpr, num_min_pts,
                                th_seeds, th_dist, th_outlier,
                                th_normal, th_weight,
                                th_lcc_normal, th_lcc_planar, th_obstacle,
                                refine_mode, /*viz_mode=*/false);
             },
             "max_range"_a, "min_range"_a, "resolution"_a,
             "num_iter"_a, "num_lpr"_a, "num_min_pts"_a,
             "th_seeds"_a, "th_dist"_a, "th_outlier"_a,
             "th_normal"_a, "th_weight"_a,
             "th_lcc_normal"_a, "th_lcc_planar"_a, "th_obstacle"_a,
             "refine_mode"_a)
        .def("estimate_ground",
             [](travel::TravelGroundSeg<PointXYZILID>& self,
                py::array_t<float, py::array::c_style | py::array::forcecast> arr) {
                 CloudPtr in = numpyToCloud(arr);
                 Cloud    ground;
                 Cloud    nonground;
                 double   elapsed = 0.0;
                 self.estimateGround(*in, ground, nonground, elapsed);

                 auto mask = maskByNearest(ground, *in);
                 return py::make_tuple(mask, elapsed);
             },
             "points"_a,
             "Run TGS on the given points. Returns (ground_mask: bool[N], "
             "elapsed_seconds: float).");

    // -- ObjectCluster --------------------------------------------------------
    py::class_<travel::ObjectCluster<PointXYZILID>>(m, "_ObjectCluster")
        .def(py::init<>())
        .def("set_params",
             [](travel::ObjectCluster<PointXYZILID>& self,
                int   vert_scan,        int   horz_scan,
                float min_range,        float max_range,
                float min_vert_angle,   float max_vert_angle,
                float horz_merge_thres, float vert_merge_thres,
                int   vert_scan_size,   int   horz_scan_size,
                int   horz_extension_size, int horz_skip_size,
                int   downsample,
                int   min_cluster_size, int max_cluster_size) {
                 self.setParams(vert_scan, horz_scan,
                                min_range, max_range,
                                min_vert_angle, max_vert_angle,
                                horz_merge_thres, vert_merge_thres,
                                vert_scan_size, horz_scan_size,
                                horz_extension_size, horz_skip_size,
                                downsample,
                                min_cluster_size, max_cluster_size);
             },
             "vert_scan"_a, "horz_scan"_a,
             "min_range"_a, "max_range"_a,
             "min_vert_angle"_a, "max_vert_angle"_a,
             "horz_merge_thres"_a, "vert_merge_thres"_a,
             "vert_scan_size"_a, "horz_scan_size"_a,
             "horz_extension_size"_a, "horz_skip_size"_a,
             "downsample"_a,
             "min_cluster_size"_a, "max_cluster_size"_a)
        .def("set_seed",
             &travel::ObjectCluster<PointXYZILID>::setSeed,
             "seed"_a,
             "Pin the cluster-id shuffle seed so segment_objects produces "
             "bit-identical output for identical input.")
        .def("clear_seed",
             &travel::ObjectCluster<PointXYZILID>::clearSeed,
             "Restore default behaviour: shuffle cluster ids with "
             "std::random_device on every call.")
        .def("segment_objects",
             [](travel::ObjectCluster<PointXYZILID>& self,
                py::array_t<float, py::array::c_style | py::array::forcecast> arr) {
                 // Make two copies: the algorithm mutates its input pointer
                 // (sphericalProjection() reassigns *cloud_in = *valid_cloud),
                 // so we keep the pristine original around for the
                 // index-back-mapping nearest-neighbour query.
                 CloudPtr original = numpyToCloud(arr);
                 CloudPtr scratch(new Cloud(*original));
                 CloudPtr out(new Cloud());
                 self.segmentObjects(scratch, out);

                 return labelByNearest(*out, *original);
             },
             "points"_a,
             "Run AOS clustering on the given points. Returns int32[N] cluster "
             "labels (0 = unlabeled / outside any cluster, 1..K = cluster id).");
}
