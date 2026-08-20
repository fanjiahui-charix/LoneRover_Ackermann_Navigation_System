#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "mycar_navigation/nav_core/footprint.hpp"

namespace mycar_navigation::nav_core
{
namespace
{
std::set<std::pair<int, int>> toCellSet(const std::vector<GridIndex> & cells)
{
  std::set<std::pair<int, int>> result;
  for (const GridIndex & cell : cells) {
    result.emplace(cell.i, cell.j);
  }
  return result;
}

// Reference "cell-center inside rotated rectangle" predicate — the weaker test
// the conservative coverage must out-perform at clipped corners.
bool cellCenterInside(
  int i, int j, double resolution, const Footprint & fp, const Pose2D & pose)
{
  const double cx = (static_cast<double>(i) + 0.5) * resolution;
  const double cy = (static_cast<double>(j) + 0.5) * resolution;
  const double dx = cx - pose.x;
  const double dy = cy - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double local_x = dx * c + dy * s;
  const double local_y = -dx * s + dy * c;
  return std::abs(local_x) <= fp.halfLength() && std::abs(local_y) <= fp.halfWidth();
}

struct Span
{
  int min_i, max_i, min_j, max_j;
};

Span spanOf(const std::vector<GridIndex> & cells)
{
  Span span{std::numeric_limits<int>::max(), std::numeric_limits<int>::min(),
    std::numeric_limits<int>::max(), std::numeric_limits<int>::min()};
  for (const GridIndex & c : cells) {
    span.min_i = std::min(span.min_i, c.i);
    span.max_i = std::max(span.max_i, c.i);
    span.min_j = std::min(span.min_j, c.j);
    span.max_j = std::max(span.max_j, c.j);
  }
  return span;
}
}  // namespace

// Axis-aligned footprint covers exactly the cells its rectangle overlaps, and
// nothing outside that bounding box (chosen so edges fall inside cells, not on
// grid lines, to keep the count unambiguous).
TEST(FootprintTest, AxisAlignedFootprintCoversFullBoundingBox)
{
  const Footprint footprint(0.22, 0.14);
  const Pose2D pose{1.0, 1.0, 0.0};

  const auto cells = footprint.coveredCells(pose, 0.05);
  const auto set = toCellSet(cells);

  // Rectangle spans x in [0.78,1.22] -> i in [15,24]; y in [0.86,1.14] -> j in [17,22].
  EXPECT_EQ(set.size(), 60U);
  EXPECT_TRUE(set.count({15, 17}) > 0);
  EXPECT_TRUE(set.count({24, 22}) > 0);
  EXPECT_TRUE(set.count({20, 20}) > 0);
  EXPECT_FALSE(set.count({14, 17}) > 0);  // cell left of the body, no overlap
  EXPECT_FALSE(set.count({25, 17}) > 0);  // cell right of the body, no overlap
  EXPECT_FALSE(set.count({20, 16}) > 0);  // cell below the body, no overlap
}

// THE key spec §4.5 property: a cell the rotated body merely clips at a corner
// (its center lies OUTSIDE the footprint) must still be reported. A sparse
// cell-center predicate misses it; conservative polygon coverage must not.
TEST(FootprintTest, ConservativeCoverageIncludesCornerClippedCell)
{
  const Footprint footprint(0.22, 0.14);
  const Pose2D pose{1.0, 1.0, M_PI / 4.0};
  const double resolution = 0.05;

  const auto set = toCellSet(footprint.coveredCells(pose, resolution));

  // The (+half_length,-half_width) corner lands at ~(1.2546,1.0566), inside
  // cell (25,21) whose center (1.275,1.075) is outside the rotated rectangle.
  EXPECT_TRUE(set.count({25, 21}) > 0) << "corner-clipped cell must be covered";
  EXPECT_FALSE(cellCenterInside(25, 21, resolution, footprint, pose))
    << "this cell is exactly the case a cell-center test would wrongly skip";
}

// Coverage tracks the rectangle's orientation: long axis along x at yaw=0,
// along y at yaw=pi/2; the two coverage sets have equal size.
TEST(FootprintTest, RotationReorientsCoverageExtent)
{
  const Footprint footprint(0.22, 0.14);

  const auto aligned = footprint.coveredCells(Pose2D{1.0, 1.0, 0.0}, 0.05);
  const auto rotated = footprint.coveredCells(Pose2D{1.0, 1.0, M_PI_2}, 0.05);

  const Span a = spanOf(aligned);
  const Span r = spanOf(rotated);

  EXPECT_GT(a.max_i - a.min_i, a.max_j - a.min_j);  // wider in x when aligned
  EXPECT_GT(r.max_j - r.min_j, r.max_i - r.min_i);  // wider in y when rotated 90
  EXPECT_EQ(aligned.size(), rotated.size());
}

// Finer resolution yields strictly denser coverage of the same metric body.
TEST(FootprintTest, FinerResolutionProducesDenserCoverage)
{
  const Footprint footprint(0.22, 0.14);
  const Pose2D pose{1.0, 1.0, 0.0};

  const auto coarse = toCellSet(footprint.coveredCells(pose, 0.05));
  const auto fine = toCellSet(footprint.coveredCells(pose, 0.025));

  EXPECT_LT(coarse.size(), fine.size());
  EXPECT_TRUE(coarse.count({20, 20}) > 0);
  EXPECT_TRUE(fine.count({40, 40}) > 0);
}

// Zero / negative resolution is rejected.
TEST(FootprintTest, NonPositiveResolutionThrows)
{
  const Footprint footprint(0.22, 0.14);
  EXPECT_THROW(footprint.coveredCells(Pose2D{0.0, 0.0, 0.0}, 0.0), std::invalid_argument);
}

}  // namespace mycar_navigation::nav_core
