#ifndef MYCAR_NAVIGATION_PLANNER_PLANNER_HPP_
#define MYCAR_NAVIGATION_PLANNER_PLANNER_HPP_

#include "mycar_navigation/nav_core/arc.hpp"
#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/footprint.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/planner/planner_types.hpp"

namespace mycar_navigation::planner
{

using nav_core::GridIndex;
using nav_core::MapClass;
using nav_core::Point2D;
using nav_core::Pose2D;

class Planner
{
public:
  Planner(
    const PlannerConfig & cfg, const nav_core::Footprint & fp,
    const nav_core::MapMask & map, const nav_core::FieldOdomTransform & tf);

  PlannerResult computeCommand(const PlannerInput & in) const;
  PlannerResult computeRecoveryCommand(const PlannerInput & in) const;

private:
  PlannerConfig cfg_;
  nav_core::Footprint fp_;
  nav_core::MapMask map_;
  nav_core::FieldOdomTransform tf_;
  nav_core::ArcIntegrator integrator_;
  nav_core::CurvatureSampler sampler_;
};

}  // namespace mycar_navigation::planner

#endif  // MYCAR_NAVIGATION_PLANNER_PLANNER_HPP_
