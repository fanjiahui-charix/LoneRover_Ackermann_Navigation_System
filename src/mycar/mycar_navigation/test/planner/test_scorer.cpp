#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "mycar_navigation/planner/scorer.hpp"

namespace mycar_navigation::planner
{
namespace
{

PlannerConfig makeConfig()
{
  PlannerConfig cfg;
  cfg.w_goal = 1.0;
  cfg.w_clear = 0.3;
  cfg.w_smooth = 0.2;
  cfg.w_soft_cost = 0.4;
  return cfg;
}

std::vector<Pose2D> rolloutTo(double x, double y)
{
  return {{0.0, 0.0, 0.0}, {x, y, 0.0}};
}

}  // namespace

TEST(ScoreArcTest, CloserEndpointScoresHigherWhenOthersEqual)
{
  const PlannerConfig cfg = makeConfig();
  const Point2D goal_base{2.0, 0.0};

  const double near_score = scoreArc(0.1, 0.1, 1.0, 0.0, rolloutTo(1.8, 0.0), goal_base, cfg);
  const double far_score = scoreArc(0.1, 0.1, 1.0, 0.0, rolloutTo(1.2, 0.0), goal_base, cfg);

  EXPECT_GT(near_score, far_score);
}

TEST(ScoreArcTest, LargerFreeLengthScoresHigherWhenOthersEqual)
{
  const PlannerConfig cfg = makeConfig();
  const Point2D goal_base{1.0, 0.0};
  const auto rollout = rolloutTo(0.8, 0.0);

  const double shorter_clearance = scoreArc(0.0, 0.0, 0.5, 0.0, rollout, goal_base, cfg);
  const double longer_clearance = scoreArc(0.0, 0.0, 1.5, 0.0, rollout, goal_base, cfg);

  EXPECT_GT(longer_clearance, shorter_clearance);
}

TEST(ScoreArcTest, LargerCurvatureChangeGetsLowerScore)
{
  const PlannerConfig cfg = makeConfig();
  const Point2D goal_base{1.0, 0.0};
  const auto rollout = rolloutTo(0.8, 0.0);
  const double prev_kappa = 0.2;

  const double smoother = scoreArc(0.25, prev_kappa, 1.0, 0.0, rollout, goal_base, cfg);
  const double sharper = scoreArc(0.8, prev_kappa, 1.0, 0.0, rollout, goal_base, cfg);

  EXPECT_GT(smoother, sharper);
}

TEST(ScoreArcTest, RearGoalPrefersArcThatTurnsTowardGoal)
{
  const PlannerConfig cfg = makeConfig();
  const Point2D goal_base{-1.0, 1.0};

  const double turning_score = scoreArc(0.8, 0.0, 1.0, 0.0, rolloutTo(-0.7, 0.8), goal_base, cfg);
  const double straight_score = scoreArc(0.0, 0.0, 1.0, 0.0, rolloutTo(0.8, 0.0), goal_base, cfg);

  EXPECT_GT(turning_score, straight_score);
}

TEST(ScoreArcTest, EmptyRolloutReturnsVeryLowScore)
{
  const PlannerConfig cfg = makeConfig();
  const Point2D goal_base{0.0, 0.0};

  const double score = scoreArc(0.0, 0.0, 0.0, 0.0, {}, goal_base, cfg);

  EXPECT_EQ(score, -std::numeric_limits<double>::max());
}

TEST(ScoreArcTest, SoftCostLowersScore)
{
  const PlannerConfig cfg = makeConfig();
  const Point2D goal_base{1.0, 0.0};
  const auto rollout = rolloutTo(0.8, 0.0);

  const double clean_score = scoreArc(0.0, 0.0, 1.0, 0.0, rollout, goal_base, cfg);
  const double soft_score = scoreArc(0.0, 0.0, 1.0, 3.0, rollout, goal_base, cfg);

  EXPECT_GT(clean_score, soft_score);
}

}  // namespace mycar_navigation::planner
