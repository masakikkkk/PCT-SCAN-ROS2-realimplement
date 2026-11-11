#pragma once
#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include "go2_map_maintenance/voxel_map.hpp"

namespace go2_map_maintenance {
struct Field { std::string name; uint32_t offset; uint8_t datatype; uint32_t count; };
inline bool native_big_endian() {
  const uint16_t value = 0x0102;
  return reinterpret_cast<const uint8_t*>(&value)[0] == 1;
}
inline double read_coordinate(const uint8_t* bytes, uint8_t datatype, bool big) {
  std::array<uint8_t, 8> storage{};
  const size_t size = datatype == 7 ? 4 : 8;
  std::memcpy(storage.data(), bytes, size);
  if (big != native_big_endian()) std::reverse(storage.begin(), storage.begin() + size);
  if (datatype == 7) {
    float value; std::memcpy(&value, storage.data(), 4); return value;
  }
  double value; std::memcpy(&value, storage.data(), 8); return value;
}
inline std::vector<Point> decode_xyz(const uint8_t* data, size_t data_size,
    uint32_t width, uint32_t height, uint32_t point_step, uint32_t row_step,
    bool big, const std::vector<Field>& fields, size_t max_points) {
  const uint64_t count = uint64_t(width) * height;
  if (!count || count > max_points || point_step < 12 ||
      uint64_t(row_step) < uint64_t(width) * point_step ||
      uint64_t(row_step) * height > data_size)
    throw std::invalid_argument("invalid/truncated/oversized PointCloud2 layout");
  std::array<Field, 3> xyz_fields{};
  for (size_t i = 0; i < 3; ++i) {
    const std::string name(1, "xyz"[i]);
    size_t found = 0;
    for (const auto& field : fields) if (field.name == name) { xyz_fields[i] = field; ++found; }
    const auto& field = xyz_fields[i];
    if (found != 1 || field.count != 1 || (field.datatype != 7 && field.datatype != 8) ||
        uint64_t(field.offset) + (field.datatype == 7 ? 4 : 8) > point_step)
      throw std::invalid_argument("XYZ must be unique scalar FLOAT32/FLOAT64 fields");
  }
  std::vector<Point> result;
  result.reserve(static_cast<size_t>(count));
  for (uint32_t row = 0; row < height; ++row)
    for (uint32_t column = 0; column < width; ++column) {
      const uint8_t* record = data + size_t(row) * row_step + size_t(column) * point_step;
      const Point point{
        read_coordinate(record + xyz_fields[0].offset, xyz_fields[0].datatype, big),
        read_coordinate(record + xyz_fields[1].offset, xyz_fields[1].datatype, big),
        read_coordinate(record + xyz_fields[2].offset, xyz_fields[2].datatype, big)};
      if (finite(point)) result.push_back(point);
    }
  return result;
}
}  // namespace go2_map_maintenance
