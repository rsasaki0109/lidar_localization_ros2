#pragma once

#include <cstddef>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace glim_prior_map_localizer
{
namespace detail
{

struct PlyProperty
{
  std::string type;
  std::string name;
  std::size_t offset{0};
  std::size_t size{0};
};

inline std::size_t plyScalarSize(const std::string & type)
{
  if (type == "char" || type == "uchar" || type == "int8" || type == "uint8") {
    return 1;
  }
  if (type == "short" || type == "ushort" || type == "int16" || type == "uint16") {
    return 2;
  }
  if (type == "int" || type == "uint" || type == "int32" || type == "uint32" ||
    type == "float" || type == "float32")
  {
    return 4;
  }
  if (type == "double" || type == "float64") {
    return 8;
  }
  throw std::runtime_error("unsupported PLY scalar type: " + type);
}

template<typename T>
inline double copiedScalar(const char * data)
{
  T value;
  std::memcpy(&value, data, sizeof(value));
  return static_cast<double>(value);
}

inline double binaryPlyScalar(const char * data, const std::string & type)
{
  if (type == "char" || type == "int8") {
    return copiedScalar<signed char>(data);
  }
  if (type == "uchar" || type == "uint8") {
    return copiedScalar<unsigned char>(data);
  }
  if (type == "short" || type == "int16") {
    return copiedScalar<short>(data);
  }
  if (type == "ushort" || type == "uint16") {
    return copiedScalar<unsigned short>(data);
  }
  if (type == "int" || type == "int32") {
    return copiedScalar<int>(data);
  }
  if (type == "uint" || type == "uint32") {
    return copiedScalar<unsigned int>(data);
  }
  if (type == "float" || type == "float32") {
    return copiedScalar<float>(data);
  }
  if (type == "double" || type == "float64") {
    return copiedScalar<double>(data);
  }
  throw std::runtime_error("unsupported PLY scalar type: " + type);
}

}  // namespace detail

inline std::vector<Eigen::Vector3f> loadPlyXyz(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to open prior-map PLY: " + path);
  }

  std::string line;
  std::getline(stream, line);
  if (line != "ply") {
    throw std::runtime_error("not a PLY file: " + path);
  }

  bool ascii = false;
  bool binary_little_endian = false;
  bool reading_vertices = false;
  std::size_t vertex_count = 0;
  std::size_t stride = 0;
  std::vector<detail::PlyProperty> properties;
  while (std::getline(stream, line)) {
    std::istringstream fields(line);
    std::string keyword;
    fields >> keyword;
    if (keyword == "format") {
      std::string format;
      fields >> format;
      ascii = format == "ascii";
      binary_little_endian = format == "binary_little_endian";
    } else if (keyword == "element") {
      std::string element;
      std::size_t count = 0;
      fields >> element >> count;
      reading_vertices = element == "vertex";
      if (reading_vertices) {
        vertex_count = count;
      }
    } else if (keyword == "property" && reading_vertices) {
      detail::PlyProperty property;
      fields >> property.type;
      if (property.type == "list") {
        throw std::runtime_error("list property in PLY vertex element is unsupported");
      }
      fields >> property.name;
      property.offset = stride;
      property.size = detail::plyScalarSize(property.type);
      stride += property.size;
      properties.push_back(property);
    } else if (keyword == "end_header") {
      break;
    }
  }
  if ((!ascii && !binary_little_endian) || vertex_count == 0 || properties.empty()) {
    throw std::runtime_error("unsupported or empty PLY header: " + path);
  }

  int xyz_indices[3] = {-1, -1, -1};
  for (std::size_t index = 0; index < properties.size(); ++index) {
    xyz_indices[0] = properties[index].name == "x" ? static_cast<int>(index) : xyz_indices[0];
    xyz_indices[1] = properties[index].name == "y" ? static_cast<int>(index) : xyz_indices[1];
    xyz_indices[2] = properties[index].name == "z" ? static_cast<int>(index) : xyz_indices[2];
  }
  if (xyz_indices[0] < 0 || xyz_indices[1] < 0 || xyz_indices[2] < 0) {
    throw std::runtime_error("PLY vertex element must contain x, y, and z");
  }

  std::vector<Eigen::Vector3f> points;
  points.reserve(vertex_count);
  if (ascii) {
    for (std::size_t row = 0; row < vertex_count && std::getline(stream, line); ++row) {
      std::istringstream values(line);
      std::vector<double> record(properties.size());
      for (double & value : record) {
        if (!(values >> value)) {
          throw std::runtime_error("truncated ASCII PLY vertex data");
        }
      }
      Eigen::Vector3f point(
        static_cast<float>(record[xyz_indices[0]]),
        static_cast<float>(record[xyz_indices[1]]),
        static_cast<float>(record[xyz_indices[2]]));
      if (point.allFinite()) {
        points.push_back(point);
      }
    }
  } else {
    std::vector<char> record(stride);
    for (std::size_t row = 0; row < vertex_count; ++row) {
      stream.read(record.data(), static_cast<std::streamsize>(record.size()));
      if (!stream) {
        throw std::runtime_error("truncated binary PLY vertex data");
      }
      const auto value = [&](int index) {
          const auto & property = properties[static_cast<std::size_t>(index)];
          return detail::binaryPlyScalar(record.data() + property.offset, property.type);
        };
      Eigen::Vector3f point(
        static_cast<float>(value(xyz_indices[0])),
        static_cast<float>(value(xyz_indices[1])),
        static_cast<float>(value(xyz_indices[2])));
      if (point.allFinite()) {
        points.push_back(point);
      }
    }
  }
  if (points.empty()) {
    throw std::runtime_error("prior-map PLY has no finite XYZ points: " + path);
  }
  return points;
}

}  // namespace glim_prior_map_localizer
