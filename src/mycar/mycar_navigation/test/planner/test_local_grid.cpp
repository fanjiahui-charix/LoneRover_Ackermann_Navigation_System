#include <gtest/gtest.h>

#include <vector>

#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/planner/local_grid.hpp"

namespace mycar_navigation::planner
{
namespace
{
constexpr double kResolution = 0.05;
constexpr double kHalfExtent = 0.5;
}  // namespace

TEST(LocalGridTest, MarksScanCellsWithInflation)
{
  LocalGrid grid(kResolution, kHalfExtent);
  grid.addScanPoints({Point2D{0.10, 0.0}}, 0.05);

  EXPECT_TRUE(grid.isScanOccupied(GridIndex{2, 0}));
  EXPECT_TRUE(grid.isScanOccupied(GridIndex{1, 0}));
  EXPECT_FALSE(grid.isScanOccupied(GridIndex{2, 1}));
  EXPECT_FALSE(grid.isScanOccupied(GridIndex{-5, -5}));
}

TEST(LocalGridTest, MapForbiddenTreatsUnknownAsConfigured)
{
  std::vector<MapClass> cells(20U * 20U, MapClass::DRIVABLE);
  cells[9U * 20U + 11U] = MapClass::UNKNOWN;
  const nav_core::MapMask map = nav_core::MapMask::fromCells(20U, 20U, 0.05, -0.5, -0.5, std::move(cells));
  const nav_core::FieldOdomTransform tf(Pose2D{0.0, 0.0, 0.0});

  LocalGrid conservative(kResolution, kHalfExtent);
  conservative.addMapForbidden(map, tf, Pose2D{0.0, 0.0, 0.0}, true);
  EXPECT_TRUE(conservative.isMapForbidden(GridIndex{1, 0}));

  LocalGrid permissive(kResolution, kHalfExtent);
  permissive.addMapForbidden(map, tf, Pose2D{0.0, 0.0, 0.0}, false);
  EXPECT_FALSE(permissive.isMapForbidden(GridIndex{1, 0}));
}

TEST(LocalGridTest, OutOfBoundsMapQueryIsConservative)
{
  LocalGrid grid(kResolution, kHalfExtent);
  EXPECT_TRUE(grid.isMapForbidden(GridIndex{100, 0}));
  EXPECT_FALSE(grid.isScanOccupied(GridIndex{100, 0}));
}

}  // namespace mycar_navigation::planner
