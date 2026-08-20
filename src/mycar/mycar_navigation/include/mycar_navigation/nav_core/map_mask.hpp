#ifndef MYCAR_NAVIGATION_NAV_CORE_MAP_MASK_HPP_
#define MYCAR_NAVIGATION_NAV_CORE_MAP_MASK_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::nav_core
{

class MapMask
{
public:
  struct PixelThresholds
  {
    double occupied{0.65};
    double free{0.196};
    int negate{0};
  };

  struct Config
  {
    std::string image_path;
    double resolution{0.0};
    Pose2D origin{};
    PixelThresholds thresholds{};
    bool has_soft_cost_layer{false};
  };

  static MapMask loadFromYaml(const std::string & yaml_path);
  static MapMask fromFiles(const Config & config);
  static MapMask fromCells(
    std::size_t width, std::size_t height, double resolution, double origin_x,
    double origin_y, std::vector<MapClass> cells);

  int width() const noexcept;
  int height() const noexcept;
  double resolution() const noexcept;
  const Pose2D & origin() const noexcept;
  const PixelThresholds & thresholds() const noexcept;
  bool hasSoftCostLayer() const noexcept;

  bool isInBounds(const GridIndex & index) const noexcept;
  std::optional<GridIndex> worldToMap(const Point2D & world_point) const noexcept;
  Point2D mapToWorld(const GridIndex & index) const;
  MapClass classify(const GridIndex & index) const noexcept;
  MapClass classifyWorld(const Point2D & world_point) const noexcept;

private:
  struct GrayImage
  {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> pixels;
  };

  MapMask(Config config, GrayImage image);

  static GrayImage loadPgm(const std::string & image_path);
  static Config loadConfigFromYaml(const std::string & yaml_path);
  static std::string resolveImagePath(const std::string & yaml_path, const std::string & image_path);
  static MapClass classifyPixel(
    std::uint8_t pixel, const PixelThresholds & thresholds, bool has_soft_cost_layer) noexcept;

  Config config_;
  GrayImage image_;
  std::vector<MapClass> classes_;
};

}  // namespace mycar_navigation::nav_core

#endif  // MYCAR_NAVIGATION_NAV_CORE_MAP_MASK_HPP_
