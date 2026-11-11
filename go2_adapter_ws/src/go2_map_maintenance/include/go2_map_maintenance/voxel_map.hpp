#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace go2_map_maintenance {

struct Point { double x, y, z; };
inline bool finite(const Point& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
struct Key {
  int64_t x, y, z;
  bool operator==(const Key& b) const { return x == b.x && y == b.y && z == b.z; }
  bool operator<(const Key& b) const {
    return std::tie(x, y, z) < std::tie(b.x, b.y, b.z);
  }
};
struct KeyHash {
  size_t operator()(const Key& k) const {
    size_t seed = std::hash<int64_t>{}(k.x);
    seed ^= std::hash<int64_t>{}(k.y) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int64_t>{}(k.z) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};
struct Limits {
  double voxel_size = 0.10;
  double extent_xy = 10.0;  // Half-extents about the first accepted body pose.
  double extent_z = 4.0;
  double candidate_ttl = 5.0;
  size_t min_observations = 3;
  size_t max_voxels = 200000;
  size_t max_input_points = 200000;
};
struct MapLimitError : std::runtime_error { using std::runtime_error::runtime_error; };

// Static-scene accumulator. No free-space carving, TTL deletion of confirmed
// surfaces, localization or loop closure is claimed by this implementation.
class VoxelMap {
 public:
  explicit VoxelMap(Limits limits = Limits{}) : limits_(limits) {
    if (!std::isfinite(limits.voxel_size) || limits.voxel_size < 0.001 ||
        !std::isfinite(limits.extent_xy) || limits.extent_xy <= 0 ||
        !std::isfinite(limits.extent_z) || limits.extent_z <= 0 ||
        !std::isfinite(limits.candidate_ttl) || limits.candidate_ttl <= 0 ||
        !limits.min_observations || !limits.max_voxels || !limits.max_input_points)
      throw std::invalid_argument("invalid voxel-map limits");
  }

  void reset() {
    cells_.clear();
    anchored_ = false;
    version_ = confirmed_ = 0;
    last_stamp_ = 0;
    last_time_ = -std::numeric_limits<double>::infinity();
  }

  void integrate(const std::vector<Point>& points, Point body, int64_t stamp, double now) {
    if (!finite(body) || std::max({std::abs(body.x), std::abs(body.y), std::abs(body.z)}) > 1e9 ||
        stamp <= 0 || !std::isfinite(now) || now < last_time_)
      throw std::invalid_argument("invalid pose, timestamp or monotonic time");
    if (stamp <= last_stamp_)
      throw std::invalid_argument("cloud timestamp did not advance");
    if (points.empty() || points.size() > limits_.max_input_points)
      throw std::invalid_argument("invalid input point count");
    const Point origin = anchored_ ? origin_ : body;
    if (!inside(body, origin))
      throw MapLimitError("robot left mapping region; reset required");

    struct Sum { double x = 0, y = 0, z = 0; size_t n = 0; };
    std::unordered_map<Key, Sum, KeyHash> observations;
    for (const Point& p : points) {
      if (!finite(p) || std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) > 1e9 ||
          !inside(p, origin)) continue;
      Sum& sum = observations[key(p)];
      sum.x += p.x; sum.y += p.y; sum.z += p.z; ++sum.n;
    }
    if (observations.empty())
      throw std::invalid_argument("no finite points inside mapping region");

    // Capacity-check the entire update before mutating any confirmed geometry.
    size_t expired = 0, added = 0;
    for (const auto& entry : cells_)
      if (!entry.second.confirmed && now - entry.second.last_seen > limits_.candidate_ttl)
        ++expired;
    for (const auto& entry : observations) {
      auto old = cells_.find(entry.first);
      if (old == cells_.end() || (!old->second.confirmed &&
          now - old->second.last_seen > limits_.candidate_ttl)) ++added;
    }
    if (cells_.size() - expired + added > limits_.max_voxels)
      throw MapLimitError("voxel capacity exceeded; reset or larger configured limit required");
    for (auto it = cells_.begin(); it != cells_.end();) {
      if (!it->second.confirmed && now - it->second.last_seen > limits_.candidate_ttl)
        it = cells_.erase(it);
      else ++it;
    }

    bool changed = false;
    for (const auto& observation : observations) {
      Cell& cell = cells_[observation.first];
      cell.last_seen = now;
      if (cell.confirmed) continue;
      const Sum& sum = observation.second;
      cell.sum.x += sum.x / sum.n;
      cell.sum.y += sum.y / sum.n;
      cell.sum.z += sum.z / sum.n;
      ++cell.observations;  // At most one vote per voxel per accepted cloud.
      if (cell.observations >= limits_.min_observations) {
        cell.surface = {cell.sum.x / cell.observations,
                        cell.sum.y / cell.observations,
                        cell.sum.z / cell.observations};
        cell.confirmed = changed = true;
        ++confirmed_;
      }
    }
    // Freeze each confirmed representative: repeated noise/count increases do
    // not produce a geometry revision or repeatedly restart the PCT planner.
    if (changed) ++version_;
    origin_ = origin;
    anchored_ = true;
    last_stamp_ = stamp;
    last_time_ = now;
  }

  std::vector<Point> snapshot() const {
    std::vector<std::pair<Key, Point>> entries;
    entries.reserve(confirmed_);
    for (const auto& entry : cells_)
      if (entry.second.confirmed) entries.emplace_back(entry.first, entry.second.surface);
    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });
    std::vector<Point> result;
    result.reserve(entries.size());
    for (const auto& entry : entries) result.push_back(entry.second);
    return result;
  }
  uint64_t version() const { return version_; }
  size_t confirmed() const { return confirmed_; }
  size_t size() const { return cells_.size(); }
  int64_t last_stamp() const { return last_stamp_; }

 private:
  struct Cell {
    Point sum{0, 0, 0}, surface{0, 0, 0};
    size_t observations = 0;
    double last_seen = 0;
    bool confirmed = false;
  };
  Key key(const Point& p) const {
    return {static_cast<int64_t>(std::floor(p.x / limits_.voxel_size)),
            static_cast<int64_t>(std::floor(p.y / limits_.voxel_size)),
            static_cast<int64_t>(std::floor(p.z / limits_.voxel_size))};
  }
  bool inside(const Point& p, const Point& origin) const {
    return std::abs(p.x - origin.x) <= limits_.extent_xy &&
           std::abs(p.y - origin.y) <= limits_.extent_xy &&
           std::abs(p.z - origin.z) <= limits_.extent_z;
  }
  Limits limits_;
  std::unordered_map<Key, Cell, KeyHash> cells_;
  Point origin_{0, 0, 0};
  bool anchored_ = false;
  uint64_t version_ = 0;
  size_t confirmed_ = 0;
  int64_t last_stamp_ = 0;
  double last_time_ = -std::numeric_limits<double>::infinity();
};
}  // namespace go2_map_maintenance
