#include <iostream>
#include <fstream>
#include "go2_map_maintenance/cloud_decode.hpp"

using namespace go2_map_maintenance;
static size_t checks = 0;
static void require(bool value, const char* description) {
  ++checks;
  if (!value) throw std::runtime_error(description);
}
template<class F> static void rejects(F function, const char* description) {
  bool rejected = false;
  try { function(); } catch (const std::exception&) { rejected = true; }
  require(rejected, description);
}

int main(int argc, char** argv) {
  Limits limits;
  limits.min_observations = 3;
  VoxelMap map(limits);
  std::vector<Point> repeated(1000, Point{0.02, 0.03, 0.01});
  map.integrate(repeated, {0, 0, 0.3}, 1, 1.0);
  require(map.size() == 1 && map.confirmed() == 0, "one frame is one vote, not 1000");
  rejects([&]() { map.integrate(repeated, {0, 0, 0.3}, 1, 1.1); }, "duplicate cloud rejected");
  map.integrate(repeated, {0, 0, 0.3}, 2, 1.2);
  require(map.confirmed() == 0, "surface not yet confirmed");
  map.integrate(repeated, {0, 0, 0.3}, 3, 1.4);
  require(map.confirmed() == 1 && map.version() == 1, "three independent clouds confirm surface");
  const auto original = map.snapshot();
  map.integrate({{0.08, 0.09, 0.09}}, {0, 0, 0.3}, 4, 1.6);
  require(map.version() == 1 && map.snapshot()[0].x == original[0].x,
          "stable geometry and version do not jitter");
  // Not seeing an old surface is not free-space evidence.
  map.integrate({{1.02, 0.02, 0.02}}, {0, 0, 0.3}, 5, 20.0);
  require(map.confirmed() == 1, "confirmed surface survives long occlusion");
  map.integrate({{2.02, 0.02, 0.02}}, {0, 0, 0.3}, 6, 26.0);
  require(map.size() == 2, "unconfirmed candidates expire but confirmed geometry remains");
  rejects([&]() { map.integrate(repeated, {11, 0, 0.3}, 7, 27.0); }, "body outside region rejected");
  require(map.last_stamp() == 6 && map.confirmed() == 1, "invalid update is not accepted");
  rejects([&]() { map.integrate(repeated, {0, 0, 0.3}, 7, 25.0); }, "steady time regression rejected");
  rejects([&]() { map.integrate({{1e300, 0, 0}}, {0, 0, 0.3}, 7, 27.0); }, "huge coordinates rejected");
  map.reset();
  require(map.size() == 0 && map.confirmed() == 0 && map.version() == 0, "explicit reset clears map");

  limits.min_observations = 1;
  limits.max_voxels = 2;
  VoxelMap bounded(limits);
  bounded.integrate({{0, 0, 0}, {0, 0, 2.5}}, {0, 0, 0.3}, 1, 1.0);
  require(bounded.confirmed() == 2, "floor and ceiling preserved");
  rejects([&]() { bounded.integrate({{1, 0, 0}, {2, 0, 0}}, {0, 0, 0.3}, 2, 2.0); },
          "capacity overflow rejected");
  require(bounded.size() == 2 && bounded.version() == 1 && bounded.last_stamp() == 1,
          "overflow does not partially mutate or refresh map");
  VoxelMap negative(limits);
  negative.integrate({{-0.01, 0, 0}, {0.01, 0, 0}}, {0, 0, 0}, 1, 1.0);
  require(negative.size() == 2, "voxel indexing uses floor for negative positions");
  require(negative.snapshot()[0].x < 0, "snapshot ordering is deterministic");
  Limits invalid = limits; invalid.voxel_size = 0;
  rejects([&]() { VoxelMap bad(invalid); }, "bad resolution rejected");

  // Padded organized FLOAT32 records and opposite byte order.
  std::vector<Field> fields{{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}};
  for (bool big : {false, true}) {
    std::vector<uint8_t> bytes(80, 0);
    for (size_t row = 0; row < 2; ++row) for (size_t col = 0; col < 2; ++col)
      for (size_t component = 0; component < 3; ++component) {
        float value = static_cast<float>(100*row + 10*col + component);
        std::array<uint8_t, 4> raw;
        std::memcpy(raw.data(), &value, 4);
        if (big != native_big_endian()) std::reverse(raw.begin(), raw.end());
        std::memcpy(bytes.data() + row*40 + col*16 + component*4, raw.data(), 4);
      }
    const auto points = decode_xyz(bytes.data(), bytes.size(), 2, 2, 16, 40, big, fields, 10);
    require(points.size() == 4 && points[3].x == 110 && points[3].z == 112,
            "row padding, record padding and byte order decoded correctly");
    rejects([&]() { decode_xyz(bytes.data(), 79, 2, 2, 16, 40, big, fields, 10); },
            "truncated payload rejected");
    rejects([&]() { decode_xyz(bytes.data(), 80, 2, 2, 16, 40, big, fields, 3); },
            "oversized input rejected");
    auto bad = fields; bad[0].offset = 15;
    rejects([&]() { decode_xyz(bytes.data(), 80, 2, 2, 16, 40, big, bad, 10); },
            "field outside record rejected");
    bad = fields; bad[0].datatype = 5;
    rejects([&]() { decode_xyz(bytes.data(), 80, 2, 2, 16, 40, big, bad, 10); },
            "non-floating point XYZ rejected");
  }
  double doubles[3] = {0.125, -0.25, 1.5};
  const auto fp64 = decode_xyz(reinterpret_cast<uint8_t*>(doubles), 24, 1, 1, 24, 24,
    native_big_endian(), {{"x",0,8,1},{"y",8,8,1},{"z",16,8,1}}, 10);
  require(fp64.size() == 1 && fp64[0].y == -0.25, "FLOAT64 XYZ supported");
  if (argc == 2) {
    // Emit a real native accumulator snapshot for the CPU tomography/A* smoke.
    std::vector<Point> room;
    for (int x = 0; x <= 60; ++x) for (int y = 0; y <= 40; ++y) {
      room.push_back({x*0.1+0.025, y*0.1+0.025, 0.025});
      room.push_back({x*0.1+0.025, y*0.1+0.025, 2.525});
    }
    VoxelMap accumulated;
    for (int frame = 1; frame <= 3; ++frame)
      accumulated.integrate(room, {1, 1, 0.34}, frame, frame*0.2);
    require(accumulated.confirmed() == room.size(), "synthetic room surfaces accumulated");
    std::ofstream output(argv[1]);
    for (const auto& p : accumulated.snapshot()) output << p.x << ' ' << p.y << ' ' << p.z << '\n';
    if (!output) throw std::runtime_error("cannot write synthetic snapshot");
  }
  std::cout << "voxel map / PointCloud2 decoder: " << checks << " checks passed\n";
  return 0;
}
