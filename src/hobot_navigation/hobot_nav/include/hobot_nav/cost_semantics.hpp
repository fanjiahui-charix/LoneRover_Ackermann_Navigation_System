#ifndef HOBOT_NAV__COST_SEMANTICS_HPP_
#define HOBOT_NAV__COST_SEMANTICS_HPP_

#include <cstdint>

namespace hobot_nav
{

constexpr bool isCenterForbidden(std::uint8_t cost) noexcept
{
  return cost >= 253U;
}

constexpr bool isHardObstacle(std::uint8_t cost) noexcept
{
  return cost >= 254U;
}

}  // namespace hobot_nav

#endif  // HOBOT_NAV__COST_SEMANTICS_HPP_
