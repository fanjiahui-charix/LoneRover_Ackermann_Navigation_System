#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include "mycar_navigation/nav_core/map_mask.hpp"

namespace mycar_navigation::nav_core
{
namespace
{
std::filesystem::path makeTempDir()
{
  const auto base = std::filesystem::temp_directory_path() / "mycar_navigation_map_mask_tests";
  std::filesystem::create_directories(base);
  return base;
}

std::filesystem::path writeTextFile(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream stream(path, std::ios::binary);
  stream << content;
  stream.close();
  return path;
}
}  // namespace

TEST(MapMaskTest, LoadsYamlAndAppliesMapServerThresholdSemantics)
{
  const auto temp_dir = makeTempDir();
  const auto yaml_path = writeTextFile(
    temp_dir / "mask.yaml",
    "image: mask.pgm\n"
    "resolution: 0.5\n"
    "origin: [1.0, 2.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n");
  writeTextFile(
    temp_dir / "mask.pgm",
    "P2\n"
    "3 2\n"
    "255\n"
    "0 205 254\n"
    "255 100 50\n");

  const MapMask mask = MapMask::loadFromYaml(yaml_path.string());

  EXPECT_EQ(mask.width(), 3);
  EXPECT_EQ(mask.height(), 2);
  EXPECT_DOUBLE_EQ(mask.resolution(), 0.5);
  EXPECT_FALSE(mask.hasSoftCostLayer());
  EXPECT_EQ(mask.classify(GridIndex{0, 0}), MapClass::HARD_FORBIDDEN);
  EXPECT_EQ(mask.classify(GridIndex{1, 0}), MapClass::UNKNOWN);
  EXPECT_EQ(mask.classify(GridIndex{2, 0}), MapClass::DRIVABLE);
  EXPECT_EQ(mask.classify(GridIndex{0, 1}), MapClass::DRIVABLE);
  EXPECT_EQ(mask.classify(GridIndex{1, 1}), MapClass::UNKNOWN);
  EXPECT_EQ(mask.classify(GridIndex{2, 1}), MapClass::HARD_FORBIDDEN);
  EXPECT_EQ(mask.classify(GridIndex{-1, 0}), MapClass::UNKNOWN);
}

TEST(MapMaskTest, NegateFlipsOccupancyInterpretation)
{
  const auto temp_dir = makeTempDir();
  const auto yaml_path = writeTextFile(
    temp_dir / "negated.yaml",
    "image: negated.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n"
    "negate: 1\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n");
  writeTextFile(
    temp_dir / "negated.pgm",
    "P2\n"
    "2 1\n"
    "255\n"
    "255 0\n");

  const MapMask mask = MapMask::loadFromYaml(yaml_path.string());

  EXPECT_EQ(mask.classify(GridIndex{0, 0}), MapClass::HARD_FORBIDDEN);
  EXPECT_EQ(mask.classify(GridIndex{1, 0}), MapClass::DRIVABLE);
}

TEST(MapMaskTest, SoftLayerMapsIntermediatePixelsToSoftCost)
{
  const auto temp_dir = makeTempDir();
  const auto yaml_path = writeTextFile(
    temp_dir / "soft.yaml",
    "image: soft.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n"
    "has_soft_cost_layer: true\n");
  writeTextFile(
    temp_dir / "soft.pgm",
    "P2\n"
    "3 1\n"
    "255\n"
    "254 170 0\n");

  const MapMask mask = MapMask::loadFromYaml(yaml_path.string());

  EXPECT_TRUE(mask.hasSoftCostLayer());
  EXPECT_EQ(mask.classify(GridIndex{0, 0}), MapClass::DRIVABLE);
  EXPECT_EQ(mask.classify(GridIndex{1, 0}), MapClass::SOFT_COST);
  EXPECT_EQ(mask.classify(GridIndex{2, 0}), MapClass::HARD_FORBIDDEN);
}

TEST(MapMaskTest, WorldMapRoundTripUsesCellCentersAndRejectsOutOfBounds)
{
  const auto temp_dir = makeTempDir();
  const auto yaml_path = writeTextFile(
    temp_dir / "roundtrip.yaml",
    "image: roundtrip.pgm\n"
    "resolution: 0.25\n"
    "origin: [1.0, -0.5, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n");
  writeTextFile(
    temp_dir / "roundtrip.pgm",
    "P2\n"
    "4 3\n"
    "255\n"
    "255 255 255 255\n"
    "255 255 255 255\n"
    "255 255 255 255\n");

  const MapMask mask = MapMask::loadFromYaml(yaml_path.string());
  const GridIndex index{2, 1};

  const Point2D world = mask.mapToWorld(index);
  const auto recovered = mask.worldToMap(world);

  ASSERT_TRUE(recovered.has_value());
  EXPECT_EQ(recovered->i, index.i);
  EXPECT_EQ(recovered->j, index.j);
  EXPECT_EQ(mask.classifyWorld(world), MapClass::DRIVABLE);
  EXPECT_FALSE(mask.worldToMap(Point2D{0.99, -0.5}).has_value());
  EXPECT_EQ(mask.classifyWorld(Point2D{5.0, 5.0}), MapClass::UNKNOWN);
}

TEST(MapMaskTest, SoftCostLayerRemainsExplicitlyUnconfiguredWhenAbsent)
{
  const auto temp_dir = makeTempDir();
  const auto yaml_path = writeTextFile(
    temp_dir / "soft_placeholder.yaml",
    "image: soft_placeholder.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n");
  writeTextFile(
    temp_dir / "soft_placeholder.pgm",
    "P2\n"
    "1 1\n"
    "255\n"
    "205\n");

  const MapMask mask = MapMask::loadFromYaml(yaml_path.string());

  EXPECT_FALSE(mask.hasSoftCostLayer());
  EXPECT_EQ(mask.classify(GridIndex{0, 0}), MapClass::UNKNOWN);
}

TEST(MapMaskTest, FromCellsBuildsEquivalentInMemoryMap)
{
  const MapMask mask = MapMask::fromCells(
    3U, 2U, 0.5, 1.0, -0.5,
    {
      MapClass::HARD_FORBIDDEN, MapClass::UNKNOWN, MapClass::DRIVABLE,
      MapClass::SOFT_COST, MapClass::DRIVABLE, MapClass::HARD_FORBIDDEN,
    });

  EXPECT_EQ(mask.width(), 3);
  EXPECT_EQ(mask.height(), 2);
  EXPECT_DOUBLE_EQ(mask.resolution(), 0.5);
  EXPECT_EQ(mask.origin().x, 1.0);
  EXPECT_EQ(mask.origin().y, -0.5);
  EXPECT_EQ(mask.origin().yaw, 0.0);
  EXPECT_FALSE(mask.hasSoftCostLayer());
  EXPECT_EQ(mask.classify(GridIndex{0, 0}), MapClass::HARD_FORBIDDEN);
  EXPECT_EQ(mask.classify(GridIndex{1, 0}), MapClass::UNKNOWN);
  EXPECT_EQ(mask.classify(GridIndex{2, 0}), MapClass::DRIVABLE);
  EXPECT_EQ(mask.classify(GridIndex{0, 1}), MapClass::SOFT_COST);
  EXPECT_EQ(mask.classify(GridIndex{1, 1}), MapClass::DRIVABLE);
  EXPECT_EQ(mask.classify(GridIndex{2, 1}), MapClass::HARD_FORBIDDEN);

  const Point2D world = mask.mapToWorld(GridIndex{1, 1});
  const auto round_trip = mask.worldToMap(world);
  ASSERT_TRUE(round_trip.has_value());
  EXPECT_EQ(round_trip->i, 1);
  EXPECT_EQ(round_trip->j, 1);
  EXPECT_EQ(mask.classifyWorld(world), MapClass::DRIVABLE);
}

}  // namespace mycar_navigation::nav_core
