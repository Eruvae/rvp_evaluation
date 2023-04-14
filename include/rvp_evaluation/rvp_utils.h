#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <type_traits>

#define ENUM_FLAG_OPERATORS(T) \
  template<class T> constexpr inline T operator~ (T a) { \
    using U = typename std::underlying_type<T>::type; \
    return static_cast<T>(~static_cast<U>(a)); \
  } \
\
  template<class T> constexpr inline T operator| (T a, T b) \
  { \
    using U = typename std::underlying_type<T>::type; \
    return static_cast<T>(static_cast<U>(a) | static_cast<U>(b)); \
  } \
\
  template<class T> constexpr inline T operator& (T a, T b) \
  { \
    using U = typename std::underlying_type<T>::type; \
    return static_cast<T>(static_cast<U>(a) & static_cast<U>(b)); \
  } \
\
  template<class T> constexpr inline T operator^ (T a, T b) \
  { \
    using U = typename std::underlying_type<T>::type; \
    return static_cast<T>(static_cast<U>(a) ^ static_cast<U>(b)); \
  } \
\
  template<class T> constexpr inline T& operator|= (T& a, T b) \
  { \
    return a = a | b; \
  } \
\
  template<class T> constexpr inline T& operator&= (T& a, T b) \
  { \
    return a = a & b; \
  } \
\
  template<class T> constexpr inline T& operator^= (T& a, T b) \
  { \
    return a = a ^ b; \
  } \
\
  template<class T> constexpr inline bool test(T a, T b) \
  { \
    return (a & b) == b; \
  }

namespace rvp_evaluation
{

static double doubleVecDiff(const std::vector<double> &v1, const std::vector<double> &v2)
{
  double res = 0;
  for (size_t i=0; i < v1.size(); i++)
  {
    res += std::abs(v2[i] - v1[i]);
  }
  return res;
}

static double computeTrajectoryLength(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
  double traj_length = 0;
  const std::vector<double> *last_pos = &plan.start_state_.joint_state.position;
  for (const trajectory_msgs::JointTrajectoryPoint &point : plan.trajectory_.joint_trajectory.points)
  {
    const std::vector<double> *point_pos = &point.positions;
    traj_length += doubleVecDiff(*last_pos, *point_pos);
    last_pos = point_pos;
  }
  return traj_length;
}

inline static double getTrajectoryDuration(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
  return plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
}

template<typename T>
std::ostream& writeVector(std::ostream &os, double passed_time, const std::vector<T> &vec)
{
  os << passed_time << ",";
  for (size_t i = 0; i < vec.size(); i++)
  {
    os << vec[i];
    if (i < vec.size() - 1)
      os << ",";
  }
  return os;
}

} // namespace rvp_evaluation
