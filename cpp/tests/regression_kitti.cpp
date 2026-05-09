// Regression test: bit-identical comparison against a frozen gold dump.
//
// Loads <input.bin>, runs TravelGroundSeg + ObjectCluster (with the AOS
// shuffle pinned to a known seed) using the KITTI-tuned default parameters,
// and dumps a per-input-point label file:
//
//   uint32  N
//   N * { uint8 is_ground, uint16 cluster_id }
//
// Compares against <gold.bin> if provided. Exit code is non-zero on any
// difference, so this can be wired straight into CI.
//
// Usage:
//   regression_kitti <input.bin> <out.bin> [gold.bin]
//
// Notes on determinism:
// - TGS (estimateGround) is fully deterministic given fixed inputs and
//   parameters.
// - AOS uses std::random_device by default to shuffle cluster ids in
//   labelPointcloud(); that is non-deterministic. ObjectCluster::setSeed()
//   pins the shuffle so the result is deterministic *for a single
//   toolchain*.
// - The shuffle implementation itself is libc++-vs-libstdc++ dependent
//   (std::uniform_int_distribution is not portable), so the raw cluster
//   ids differ between e.g. macOS clang and Linux gcc even with the same
//   seed. To make the gold dump truly portable, we canonicalize each
//   cluster id by its smallest input-point index *before* writing the
//   dump. The canonicalization is deterministic given the partition.

#define PCL_NO_PRECOMPILE

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include "travel/aos.hpp"
#include "travel/point_types.hpp"
#include "travel/tgs.hpp"

namespace {

// Fixed AOS shuffle seed used to produce the reference (gold) dump. Changing
// this value invalidates the gold file and forces a regeneration.
constexpr uint32_t kAosSeed = 42;

bool loadBin(const std::string& path, travel::PointCloud<PointXYZILID>::Ptr& cloud) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) { std::cerr << "open failed: " << path << "\n"; return false; }
    std::streamsize bytes = f.tellg();
    if (bytes <= 0 || bytes % (4 * sizeof(float)) != 0) {
        std::cerr << "bad bin size: " << bytes << "\n"; return false;
    }
    size_t n = bytes / (4 * sizeof(float));
    std::vector<float> buf(n * 4);
    f.seekg(0);
    f.read(reinterpret_cast<char*>(buf.data()), bytes);

    cloud->reserve(n);
    for (size_t i = 0; i < n; ++i) {
        PointXYZILID p{};
        p.x = buf[i*4 + 0];
        p.y = buf[i*4 + 1];
        p.z = buf[i*4 + 2];
        p.intensity = buf[i*4 + 3];
        p.label = 0; p.id = 0;
        cloud->push_back(p);
    }
    return true;
}

// Build {xyz -> output index} via uint32 bit-pattern hashing. Algorithm
// copies x/y/z exactly (no projection or quantization), so this is safe.
std::vector<int> mapToInputIndex(const travel::PointCloud<PointXYZILID>& input,
                                 const travel::PointCloud<PointXYZILID>& out) {
    struct Key { uint32_t x, y, z; };
    struct Hash { size_t operator()(const Key& k) const {
        return (size_t)k.x * 1000003u ^ (size_t)k.y * 7919u ^ (size_t)k.z;
    }};
    struct Eq { bool operator()(const Key& a, const Key& b) const {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }};
    auto toKey = [](float x, float y, float z) {
        Key k; std::memcpy(&k.x, &x, 4); std::memcpy(&k.y, &y, 4); std::memcpy(&k.z, &z, 4);
        return k;
    };

    std::unordered_map<Key, int, Hash, Eq> idx;
    idx.reserve(out.size() * 2);
    for (size_t j = 0; j < out.size(); ++j) {
        idx.emplace(toKey(out[j].x, out[j].y, out[j].z), (int)j);
    }
    std::vector<int> result(input.size(), -1);
    for (size_t i = 0; i < input.size(); ++i) {
        auto it = idx.find(toKey(input[i].x, input[i].y, input[i].z));
        if (it != idx.end()) result[i] = it->second;
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0] << " <input.bin> <out.bin> [gold.bin]\n";
        return 1;
    }
    const std::string in_path   = argv[1];
    const std::string out_path  = argv[2];
    const std::string gold_path = (argc == 4) ? argv[3] : "";

    travel::PointCloud<PointXYZILID>::Ptr cloud_in(new travel::PointCloud<PointXYZILID>());
    if (!loadBin(in_path, cloud_in)) return 2;
    std::cout << "loaded " << cloud_in->size() << " points\n";

    // KITTI defaults from ros/config/kitti_params.yaml.
    const double max_range = 80.0;
    const double min_range = 1.0;

    travel::TravelGroundSeg<PointXYZILID> tgs;
    tgs.setParams(max_range, min_range, 8.0, 3, 5, 10,
                  0.5, 0.125, 0.3, 0.940, 200.0,
                  0.03, 0.1, 1.0, /*refine_mode=*/true, /*viz_mode=*/false);

    travel::PointCloud<PointXYZILID> ground;
    travel::PointCloud<PointXYZILID> nonground;
    double tgs_time = 0.0;
    tgs.estimateGround(*cloud_in, ground, nonground, tgs_time);
    std::cout << "TGS: ground=" << ground.size()
              << " nonground=" << nonground.size() << "\n";

    travel::ObjectCluster<PointXYZILID> aos;
    aos.setParams(64, 4500, (float)min_range, (float)max_range,
                  -24.8f, 2.0f, 0.4f, 0.5f, 3, 5, 5, 5, 1, 10, 30000);
    aos.setSeed(kAosSeed);

    travel::PointCloud<PointXYZILID>::Ptr nonground_ptr(new travel::PointCloud<PointXYZILID>(nonground));
    travel::PointCloud<PointXYZILID>::Ptr labeled_ptr(new travel::PointCloud<PointXYZILID>());
    aos.segmentObjects(nonground_ptr, labeled_ptr);
    std::cout << "AOS: labeled=" << labeled_ptr->size() << " (seed=" << kAosSeed << ")\n";

    const uint32_t N = (uint32_t)cloud_in->size();
    std::vector<uint8_t>  is_ground(N, 0);
    std::vector<uint16_t> cluster_id(N, 0);

    auto ground_map = mapToInputIndex(*cloud_in, ground);
    for (uint32_t i = 0; i < N; ++i) {
        if (ground_map[i] >= 0) is_ground[i] = 1;
    }
    auto label_map = mapToInputIndex(*cloud_in, *labeled_ptr);
    for (uint32_t i = 0; i < N; ++i) {
        if (label_map[i] >= 0) {
            cluster_id[i] = labeled_ptr->points[label_map[i]].id;
        }
    }

    // Canonicalize cluster ids in input-order: the cluster encountered first
    // when scanning input indices 0..N-1 gets canonical id 1, the next new
    // cluster gets id 2, and so on. Result is platform-independent — the
    // partition is the same across toolchains (verified end to end on
    // Melodic + PCL 1.8 vs Ubuntu 20.04 + PCL 1.10), only the shuffled raw
    // ids differ.
    {
        std::unordered_map<uint16_t, uint16_t> raw_to_canonical;
        raw_to_canonical.reserve(1024);
        uint16_t next_canonical = 1;
        for (uint32_t i = 0; i < N; ++i) {
            const uint16_t raw = cluster_id[i];
            if (raw == 0) continue;
            auto it = raw_to_canonical.find(raw);
            if (it == raw_to_canonical.end()) {
                if (next_canonical == std::numeric_limits<uint16_t>::max()) {
                    std::cerr << "more than 65534 distinct clusters; "
                                 "uint16 canonical id space exhausted\n";
                    return 6;
                }
                raw_to_canonical.emplace(raw, next_canonical);
                cluster_id[i] = next_canonical;
                ++next_canonical;
            } else {
                cluster_id[i] = it->second;
            }
        }
        std::cout << "canonicalized " << raw_to_canonical.size()
                  << " distinct clusters\n";
    }

    // Write portable dump.
    std::ofstream of(out_path, std::ios::binary);
    if (!of) { std::cerr << "open out failed: " << out_path << "\n"; return 3; }
    of.write(reinterpret_cast<const char*>(&N), sizeof(N));
    for (uint32_t i = 0; i < N; ++i) {
        of.write(reinterpret_cast<const char*>(&is_ground[i]), 1);
        of.write(reinterpret_cast<const char*>(&cluster_id[i]), 2);
    }
    of.close();
    std::cout << "wrote " << out_path << "\n";

    if (gold_path.empty()) {
        std::cout << "no gold provided; capture mode only.\n";
        return 0;
    }

    // Bit-identical comparison against gold.
    std::ifstream gf(gold_path, std::ios::binary | std::ios::ate);
    if (!gf) { std::cerr << "open gold failed: " << gold_path << "\n"; return 4; }
    std::streamsize gbytes = gf.tellg();
    std::vector<uint8_t> gold_buf(gbytes);
    gf.seekg(0);
    gf.read(reinterpret_cast<char*>(gold_buf.data()), gbytes);

    std::ifstream rf(out_path, std::ios::binary | std::ios::ate);
    std::streamsize rbytes = rf.tellg();
    std::vector<uint8_t> run_buf(rbytes);
    rf.seekg(0);
    rf.read(reinterpret_cast<char*>(run_buf.data()), rbytes);

    if (gold_buf == run_buf) {
        std::cout << "REGRESSION PASS: bit-identical to " << gold_path
                  << " (" << gbytes << " bytes)\n";
        return 0;
    }

    std::cerr << "REGRESSION FAIL: dump differs from " << gold_path << "\n";
    if (gbytes != rbytes) {
        std::cerr << "  size differs: gold=" << gbytes << " run=" << rbytes << "\n";
        return 5;
    }
    int diff_bytes = 0;
    for (std::streamsize k = 0; k < gbytes; ++k) {
        if (gold_buf[k] != run_buf[k]) ++diff_bytes;
    }
    std::cerr << "  " << diff_bytes << " bytes differ (" << (100.0 * diff_bytes / gbytes) << "%)\n";
    return 5;
}
