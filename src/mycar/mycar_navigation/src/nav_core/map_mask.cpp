#include "mycar_navigation/nav_core/map_mask.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace mycar_navigation::nav_core
{
namespace
{
constexpr double kProbabilityScale = 1.0 / 255.0;

std::string readTokenSkippingComments(std::istream & stream)
{
  std::string token;
  while (stream >> token) {
    if (!token.empty() && token.front() == '#') {
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    return token;
  }
  throw std::runtime_error("Unexpected end of PGM file");
}

int parseIntToken(const std::string & token, const char * field_name)
{
  std::size_t parsed_characters = 0;
  const int value = std::stoi(token, &parsed_characters);
  if (parsed_characters != token.size()) {
    throw std::runtime_error(std::string("Invalid PGM ") + field_name);
  }
  return value;
}

int parsePositiveIntToken(const std::string & token, const char * field_name)
{
  const int value = parseIntToken(token, field_name);
  if (value <= 0) {
    throw std::runtime_error(std::string("Invalid PGM ") + field_name);
  }
  return value;
}

double parseProbability(std::uint8_t pixel, int negate) noexcept
{
  const double normalized = static_cast<double>(pixel) * kProbabilityScale;
  return negate ? normalized : 1.0 - normalized;
}
}  // namespace

MapMask MapMask::loadFromYaml(const std::string & yaml_path)
{
  return fromFiles(loadConfigFromYaml(yaml_path));
}

MapMask MapMask::fromFiles(const Config & config)
{
  if (config.image_path.empty()) {
    throw std::invalid_argument("MapMask image path must not be empty");
  }
  if (!(config.resolution > 0.0)) {
    throw std::invalid_argument("MapMask resolution must be positive");
  }
  if (!(config.thresholds.free >= 0.0) || !(config.thresholds.free <= 1.0) ||
    !(config.thresholds.occupied >= 0.0) || !(config.thresholds.occupied <= 1.0) ||
    !(config.thresholds.free < config.thresholds.occupied))
  {
    throw std::invalid_argument("MapMask thresholds must satisfy 0 <= free < occupied <= 1");
  }
  if (config.thresholds.negate != 0 && config.thresholds.negate != 1) {
    throw std::invalid_argument("MapMask negate must be 0 or 1");
  }

  return MapMask(config, loadPgm(config.image_path));
}

MapMask MapMask::fromCells(
  std::size_t width, std::size_t height, double resolution, double origin_x,
  double origin_y, std::vector<MapClass> cells)
{
  if (width == 0U || height == 0U) {
    throw std::invalid_argument("MapMask dimensions must be non-zero");
  }
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("MapMask resolution must be positive");
  }
  if (cells.size() != width * height) {
    throw std::invalid_argument("MapMask cell count must equal width * height");
  }

  Config config;
  config.resolution = resolution;
  config.origin.x = origin_x;
  config.origin.y = origin_y;
  config.origin.yaw = 0.0;

  GrayImage image;
  image.width = static_cast<int>(width);
  image.height = static_cast<int>(height);
  image.pixels.resize(cells.size());

  MapMask mask(config, std::move(image));
  mask.classes_ = std::move(cells);
  return mask;
}

int MapMask::width() const noexcept
{
  return image_.width;
}

int MapMask::height() const noexcept
{
  return image_.height;
}

double MapMask::resolution() const noexcept
{
  return config_.resolution;
}

const Pose2D & MapMask::origin() const noexcept
{
  return config_.origin;
}

const MapMask::PixelThresholds & MapMask::thresholds() const noexcept
{
  return config_.thresholds;
}

bool MapMask::hasSoftCostLayer() const noexcept
{
  return config_.has_soft_cost_layer;
}

bool MapMask::isInBounds(const GridIndex & index) const noexcept
{
  return index.i >= 0 && index.j >= 0 && index.i < image_.width && index.j < image_.height;
}

std::optional<GridIndex> MapMask::worldToMap(const Point2D & world_point) const noexcept
{
  const double relative_x = (world_point.x - config_.origin.x) / config_.resolution;
  const double relative_y = (world_point.y - config_.origin.y) / config_.resolution;

  const int i = static_cast<int>(std::floor(relative_x));
  const int row_from_bottom = static_cast<int>(std::floor(relative_y));
  const GridIndex index{i, image_.height - 1 - row_from_bottom};

  if (!isInBounds(index)) {
    return std::nullopt;
  }
  return index;
}

Point2D MapMask::mapToWorld(const GridIndex & index) const
{
  if (!isInBounds(index)) {
    throw std::out_of_range("MapMask map index out of bounds");
  }

  const double world_x = config_.origin.x + (static_cast<double>(index.i) + 0.5) * config_.resolution;
  const double row_from_bottom = static_cast<double>(image_.height - 1 - index.j);
  const double world_y = config_.origin.y + (row_from_bottom + 0.5) * config_.resolution;
  return Point2D{world_x, world_y};
}

MapClass MapMask::classify(const GridIndex & index) const noexcept
{
  if (!isInBounds(index)) {
    return MapClass::UNKNOWN;
  }
  const std::size_t offset = static_cast<std::size_t>(index.j) * static_cast<std::size_t>(image_.width) +
    static_cast<std::size_t>(index.i);
  return classes_[offset];
}

MapClass MapMask::classifyWorld(const Point2D & world_point) const noexcept
{
  const auto index = worldToMap(world_point);
  if (!index.has_value()) {
    return MapClass::UNKNOWN;
  }
  return classify(*index);
}

MapMask::MapMask(Config config, GrayImage image)
: config_(std::move(config)), image_(std::move(image))
{
  classes_.reserve(image_.pixels.size());
  for (const std::uint8_t pixel : image_.pixels) {
    classes_.push_back(classifyPixel(pixel, config_.thresholds, config_.has_soft_cost_layer));
  }
}

MapMask::GrayImage MapMask::loadPgm(const std::string & image_path)
{
  std::ifstream stream(image_path, std::ios::binary);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to open PGM image: " + image_path);
  }

  const std::string magic = readTokenSkippingComments(stream);
  const bool binary = magic == "P5";
  const bool ascii = magic == "P2";
  if (!binary && !ascii) {
    throw std::runtime_error("Unsupported PGM format: " + magic);
  }

  const int width = parsePositiveIntToken(readTokenSkippingComments(stream), "width");
  const int height = parsePositiveIntToken(readTokenSkippingComments(stream), "height");
  const int max_value = parsePositiveIntToken(readTokenSkippingComments(stream), "max value");
  if (max_value != 255) {
    throw std::runtime_error("Only 8-bit PGM files with max value 255 are supported");
  }

  stream >> std::ws;

  GrayImage image;
  image.width = width;
  image.height = height;
  image.pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));

  if (binary) {
    stream.read(reinterpret_cast<char *>(image.pixels.data()), static_cast<std::streamsize>(image.pixels.size()));
    if (stream.gcount() != static_cast<std::streamsize>(image.pixels.size())) {
      throw std::runtime_error("PGM pixel data size mismatch");
    }
  } else {
    for (std::uint8_t & pixel : image.pixels) {
      const int value = parseIntToken(readTokenSkippingComments(stream), "pixel value");
      if (value < 0 || value > 255) {
        throw std::runtime_error("PGM pixel value exceeds 255");
      }
      pixel = static_cast<std::uint8_t>(value);
    }
  }

  return image;
}

MapMask::Config MapMask::loadConfigFromYaml(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  Config config;
  config.image_path = resolveImagePath(yaml_path, root["image"].as<std::string>());
  config.resolution = root["resolution"].as<double>();

  const YAML::Node origin = root["origin"];
  if (!origin || !origin.IsSequence() || origin.size() < 3) {
    throw std::runtime_error("Map YAML origin must be a 3-element sequence");
  }
  config.origin.x = origin[0].as<double>();
  config.origin.y = origin[1].as<double>();
  config.origin.yaw = origin[2].as<double>();

  if (root["occupied_thresh"]) {
    config.thresholds.occupied = root["occupied_thresh"].as<double>();
  }
  if (root["free_thresh"]) {
    config.thresholds.free = root["free_thresh"].as<double>();
  }
  if (root["negate"]) {
    config.thresholds.negate = root["negate"].as<int>();
  }
  if (root["has_soft_cost_layer"]) {
    config.has_soft_cost_layer = root["has_soft_cost_layer"].as<bool>();
  }
  return config;
}

std::string MapMask::resolveImagePath(const std::string & yaml_path, const std::string & image_path)
{
  const std::filesystem::path image = image_path;
  if (image.is_absolute()) {
    return image.lexically_normal().string();
  }
  return (std::filesystem::path(yaml_path).parent_path() / image).lexically_normal().string();
}

MapClass MapMask::classifyPixel(
  std::uint8_t pixel, const PixelThresholds & thresholds, bool has_soft_cost_layer) noexcept
{
  const double occupancy = parseProbability(pixel, thresholds.negate);
  if (occupancy >= thresholds.occupied) {
    return MapClass::HARD_FORBIDDEN;
  }
  if (occupancy <= thresholds.free) {
    return MapClass::DRIVABLE;
  }
  if (has_soft_cost_layer) {
    return MapClass::SOFT_COST;
  }
  return MapClass::UNKNOWN;
}

}  // namespace mycar_navigation::nav_core
