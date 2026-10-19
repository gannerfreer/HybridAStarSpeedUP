// Minimal stub of nav2_core planner exceptions for the SmacPlanner2D decoupling probe.
// (family C / X-S S-07 ad-hoc feasibility probe; NOT upstream code)
//
// The extracted a_star_impl.hpp throws nav2_core::GoalOccupied and
// nav2_core::PlannerCancelled; both derive from PlannerException in upstream nav2_core.

#ifndef NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
#define NAV2_CORE__PLANNER_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace nav2_core
{

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string & description)
  : std::runtime_error(description) {}
};

class StartOccupied : public PlannerException
{
public:
  explicit StartOccupied(const std::string & description)
  : PlannerException(description) {}
};

class GoalOccupied : public PlannerException
{
public:
  explicit GoalOccupied(const std::string & description)
  : PlannerException(description) {}
};

class NoValidPathCouldBeFound : public PlannerException
{
public:
  explicit NoValidPathCouldBeFound(const std::string & description)
  : PlannerException(description) {}
};

class TimedOut : public PlannerException
{
public:
  explicit TimedOut(const std::string & description)
  : PlannerException(description) {}
};

class PlannerCancelled : public PlannerException
{
public:
  explicit PlannerCancelled(const std::string & description)
  : PlannerException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
