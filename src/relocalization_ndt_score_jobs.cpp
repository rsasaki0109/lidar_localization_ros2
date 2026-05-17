#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  bool in_quotes = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        in_quotes = !in_quotes;
      }
    } else if (c == ',' && !in_quotes) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  fields.push_back(field);
  return fields;
}

std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(c);
    }
  }
  escaped.push_back('"');
  return escaped;
}

std::vector<std::map<std::string, std::string>> read_csv(
  const std::string & path,
  std::vector<std::string> * header_out)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open input CSV: " + path);
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("empty input CSV: " + path);
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::vector<std::string> header = split_csv_line(line);
  std::vector<std::map<std::string, std::string>> rows;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> values = split_csv_line(line);
    std::map<std::string, std::string> row;
    for (std::size_t i = 0; i < header.size(); ++i) {
      row[header[i]] = i < values.size() ? values[i] : "";
    }
    rows.push_back(row);
  }
  *header_out = header;
  return rows;
}

void write_csv(
  const std::string & path,
  const std::vector<std::string> & header,
  const std::vector<std::map<std::string, std::string>> & rows)
{
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open output CSV: " + path);
  }
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (i > 0) {
      stream << ',';
    }
    stream << csv_escape(header[i]);
  }
  stream << '\n';
  for (const auto & row : rows) {
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (i > 0) {
        stream << ',';
      }
      const auto iter = row.find(header[i]);
      if (iter != row.end()) {
        stream << csv_escape(iter->second);
      }
    }
    stream << '\n';
  }
}

double as_double(const std::map<std::string, std::string> & row, const std::string & key)
{
  const auto iter = row.find(key);
  if (iter == row.end() || iter->second.empty()) {
    throw std::runtime_error("missing numeric field: " + key);
  }
  return std::stod(iter->second);
}

std::string get_value(const std::map<std::string, std::string> & row, const std::string & key)
{
  const auto iter = row.find(key);
  return iter == row.end() ? "" : iter->second;
}

bool has_field(const std::vector<pcl::PCLPointField> & fields, const std::string & name)
{
  return std::any_of(fields.begin(), fields.end(), [&](const auto & field) {
    return field.name == name;
  });
}

CloudPtr load_cloud_xyzi(const std::string & path)
{
  pcl::PCLPointCloud2 raw;
  const std::string lower = [&]() {
    std::string copy = path;
    std::transform(copy.begin(), copy.end(), copy.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return copy;
  }();
  int result = -1;
  if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".pcd") {
    result = pcl::io::loadPCDFile(path, raw);
  } else if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".ply") {
    result = pcl::io::loadPLYFile(path, raw);
  } else {
    throw std::runtime_error("unsupported point cloud suffix: " + path);
  }
  if (result != 0) {
    throw std::runtime_error("failed to load point cloud: " + path);
  }

  CloudPtr cloud(new Cloud());
  if (has_field(raw.fields, "intensity")) {
    pcl::fromPCLPointCloud2(raw, *cloud);
  } else {
    pcl::PointCloud<pcl::PointXYZ> xyz;
    pcl::fromPCLPointCloud2(raw, xyz);
    cloud->reserve(xyz.size());
    for (const auto & point : xyz.points) {
      pcl::PointXYZI xyzi;
      xyzi.x = point.x;
      xyzi.y = point.y;
      xyzi.z = point.z;
      xyzi.intensity = 0.0F;
      cloud->push_back(xyzi);
    }
  }
  return cloud;
}

CloudPtr voxel_downsample(const CloudPtr & input, double leaf_size)
{
  if (leaf_size <= 0.0 || input->empty()) {
    return input;
  }
  CloudPtr filtered(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(input);
  voxel.filter(*filtered);
  return filtered;
}

CloudPtr crop_map(const CloudPtr & map, double x, double y, double radius)
{
  if (radius <= 0.0) {
    return map;
  }
  const double r2 = radius * radius;
  CloudPtr cropped(new Cloud());
  cropped->reserve(map->size() / 10);
  for (const auto & point : map->points) {
    const double dx = static_cast<double>(point.x) - x;
    const double dy = static_cast<double>(point.y) - y;
    if (dx * dx + dy * dy <= r2) {
      cropped->push_back(point);
    }
  }
  return cropped;
}

Eigen::Matrix4f pose_matrix(double x, double y, double z, double yaw)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(yaw));
  const float s = static_cast<float>(std::sin(yaw));
  matrix(0, 0) = c;
  matrix(0, 1) = -s;
  matrix(1, 0) = s;
  matrix(1, 1) = c;
  matrix(0, 3) = static_cast<float>(x);
  matrix(1, 3) = static_cast<float>(y);
  matrix(2, 3) = static_cast<float>(z);
  return matrix;
}

double yaw_from_matrix(const Eigen::Matrix4f & matrix)
{
  return std::atan2(static_cast<double>(matrix(1, 0)), static_cast<double>(matrix(0, 0)));
}

void ensure_header_field(std::vector<std::string> * header, const std::string & field)
{
  if (std::find(header->begin(), header->end(), field) == header->end()) {
    header->push_back(field);
  }
}

struct Args
{
  std::string input_csv;
  std::string output_csv;
  int max_jobs{0};
  double ndt_resolution{1.0};
  double ndt_step_size{0.1};
  double transform_epsilon{0.01};
  int max_iterations{30};
  int num_threads{1};
  double scan_voxel_leaf_size{1.0};
  double target_voxel_leaf_size{1.0};
  double local_map_radius{150.0};
  std::size_t min_target_points{100};
  double score_gate_threshold{6.0};
  double refinement_delta_gate_m{2.0};
  double refinement_yaw_gate_rad{0.7853981633974483};
};

Args parse_args(int argc, char ** argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto require_value = [&](const std::string & option) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + option);
      }
      return argv[++i];
    };
    if (key == "--input-csv") {
      args.input_csv = require_value(key);
    } else if (key == "--output-csv") {
      args.output_csv = require_value(key);
    } else if (key == "--max-jobs") {
      args.max_jobs = std::stoi(require_value(key));
    } else if (key == "--ndt-resolution") {
      args.ndt_resolution = std::stod(require_value(key));
    } else if (key == "--ndt-step-size") {
      args.ndt_step_size = std::stod(require_value(key));
    } else if (key == "--transform-epsilon") {
      args.transform_epsilon = std::stod(require_value(key));
    } else if (key == "--max-iterations") {
      args.max_iterations = std::stoi(require_value(key));
    } else if (key == "--num-threads") {
      args.num_threads = std::stoi(require_value(key));
    } else if (key == "--scan-voxel-leaf-size") {
      args.scan_voxel_leaf_size = std::stod(require_value(key));
    } else if (key == "--target-voxel-leaf-size") {
      args.target_voxel_leaf_size = std::stod(require_value(key));
    } else if (key == "--local-map-radius") {
      args.local_map_radius = std::stod(require_value(key));
    } else if (key == "--min-target-points") {
      args.min_target_points = static_cast<std::size_t>(std::stoul(require_value(key)));
    } else if (key == "--score-gate-threshold") {
      args.score_gate_threshold = std::stod(require_value(key));
    } else if (key == "--refinement-delta-gate-m") {
      args.refinement_delta_gate_m = std::stod(require_value(key));
    } else if (key == "--refinement-yaw-gate-rad") {
      args.refinement_yaw_gate_rad = std::stod(require_value(key));
    } else {
      throw std::runtime_error("unknown argument: " + key);
    }
  }
  if (args.input_csv.empty() || args.output_csv.empty()) {
    throw std::runtime_error("--input-csv and --output-csv are required");
  }
  return args;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Args args = parse_args(argc, argv);
    std::vector<std::string> header;
    auto rows = read_csv(args.input_csv, &header);
    for (const std::string & field : {
        "target_point_count", "source_point_count", "final_pose_x", "final_pose_y", "final_pose_z",
        "final_yaw_rad", "score_gate_threshold", "refinement_delta_gate_m",
        "refinement_yaw_gate_rad", "registration_gate_passed", "registration_gate_reason"}) {
      ensure_header_field(&header, field);
    }

    std::unordered_map<std::string, CloudPtr> map_cache;
    std::unordered_map<std::string, CloudPtr> scan_cache;
    int processed = 0;
    std::map<std::string, int> status_counts;

    for (auto & row : rows) {
      if (args.max_jobs > 0 && processed >= args.max_jobs) {
        continue;
      }
      if (get_value(row, "status") != "scan_resolved_no_registration") {
        continue;
      }
      const std::string scan_path = get_value(row, "scan_pcd_path");
      const std::string map_path = get_value(row, "map_path");
      if (scan_path.empty()) {
        row["status"] = "registration_skipped";
        row["rejection_reason"] = "scan_pcd_path_missing";
        status_counts[row["status"]]++;
        continue;
      }

      const auto started = std::chrono::steady_clock::now();
      CloudPtr full_map;
      if (map_cache.count(map_path) == 0) {
        map_cache[map_path] = load_cloud_xyzi(map_path);
      }
      full_map = map_cache[map_path];
      if (scan_cache.count(scan_path) == 0) {
        scan_cache[scan_path] = load_cloud_xyzi(scan_path);
      }
      CloudPtr source = voxel_downsample(scan_cache[scan_path], args.scan_voxel_leaf_size);

      const double x = as_double(row, "initial_pose_x");
      const double y = as_double(row, "initial_pose_y");
      const double z = as_double(row, "initial_pose_z");
      const double yaw = as_double(row, "initial_yaw_rad");
      CloudPtr target = crop_map(full_map, x, y, args.local_map_radius);
      if (target->size() < args.min_target_points) {
        row["status"] = "registration_target_too_small";
        row["rejection_reason"] = "local_map_crop_too_small";
        row["target_point_count"] = std::to_string(target->size());
        row["source_point_count"] = std::to_string(source->size());
        status_counts[row["status"]]++;
        continue;
      }
      target = voxel_downsample(target, args.target_voxel_leaf_size);

      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
      ndt.setResolution(args.ndt_resolution);
      ndt.setStepSize(args.ndt_step_size);
      ndt.setTransformationEpsilon(args.transform_epsilon);
      ndt.setMaximumIterations(args.max_iterations);
      ndt.setNumThreads(std::max(1, args.num_threads));
      ndt.setInputTarget(target);
      ndt.setInputSource(source);

      const Eigen::Matrix4f init = pose_matrix(x, y, z, yaw);
      Cloud output;
      ndt.align(output, init);
      const Eigen::Matrix4f final = ndt.getFinalTransformation();
      const Eigen::Matrix4f delta = init.inverse() * final;
      const double fitness_score = ndt.getFitnessScore();
      const double refinement_delta_m = delta.block<3, 1>(0, 3).norm();
      const double refinement_delta_yaw_rad = std::abs(yaw_from_matrix(delta));
      bool gate_passed = false;
      std::string gate_reason;
      if (!ndt.hasConverged()) {
        gate_reason = "not_converged";
      } else if (!std::isfinite(fitness_score) || fitness_score > args.score_gate_threshold) {
        gate_reason = "score_above_threshold";
      } else if (refinement_delta_m > args.refinement_delta_gate_m) {
        gate_reason = "refinement_delta_above_threshold";
      } else if (refinement_delta_yaw_rad > args.refinement_yaw_gate_rad) {
        gate_reason = "refinement_yaw_above_threshold";
      } else {
        gate_passed = true;
        gate_reason = "passed_reset_disabled";
      }
      const auto ended = std::chrono::steady_clock::now();
      const double runtime_sec =
        std::chrono::duration<double>(ended - started).count();

      row["status"] = "registration_scored_no_reset";
      row["rejection_reason"] = ndt.hasConverged() ? "reset_disabled" : "registration_not_converged";
      row["converged"] = ndt.hasConverged() ? "true" : "false";
      row["score"] = std::to_string(fitness_score);
      row["overlap"] = "";
      row["refinement_delta_m"] = std::to_string(refinement_delta_m);
      row["refinement_delta_yaw_rad"] = std::to_string(refinement_delta_yaw_rad);
      row["runtime_sec"] = std::to_string(runtime_sec);
      row["target_point_count"] = std::to_string(target->size());
      row["source_point_count"] = std::to_string(source->size());
      row["final_pose_x"] = std::to_string(final(0, 3));
      row["final_pose_y"] = std::to_string(final(1, 3));
      row["final_pose_z"] = std::to_string(final(2, 3));
      row["final_yaw_rad"] = std::to_string(yaw_from_matrix(final));
      row["score_gate_threshold"] = std::to_string(args.score_gate_threshold);
      row["refinement_delta_gate_m"] = std::to_string(args.refinement_delta_gate_m);
      row["refinement_yaw_gate_rad"] = std::to_string(args.refinement_yaw_gate_rad);
      row["registration_gate_passed"] = gate_passed ? "true" : "false";
      row["registration_gate_reason"] = gate_reason;
      status_counts[row["status"]]++;
      ++processed;
    }

    write_csv(args.output_csv, header, rows);
    std::cout << "{\"output_csv\":\"" << args.output_csv << "\",\"processed_jobs\":"
              << processed << ",\"row_count\":" << rows.size() << ",\"status_counts\":{";
    bool first = true;
    for (const auto & [status, count] : status_counts) {
      if (!first) {
        std::cout << ',';
      }
      first = false;
      std::cout << "\"" << status << "\":" << count;
    }
    std::cout << "}}" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << std::endl;
    return 1;
  }
}
