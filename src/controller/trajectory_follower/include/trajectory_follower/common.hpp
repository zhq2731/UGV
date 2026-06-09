
#ifndef TRAJECTORY_FOLLOWER__COMMON_HPP_
#define TRAJECTORY_FOLLOWER__COMMON_HPP_

#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Pose.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <autoware_msgs/TrajectoryPoint.h>
#include <algorithm>
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "eigen3/Eigen/Core"


constexpr double pi = 3.14159265358979323846;

inline geometry_msgs::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

inline geometry_msgs::Point getPoint(const geometry_msgs::Point & p)
{
  return p;
}

inline geometry_msgs::Point getPoint(const geometry_msgs::Pose & p)
{
  return p.position;
}

inline geometry_msgs::Point getPoint(const geometry_msgs::PoseStamped & p)
{
  return p.pose.position;
}

inline geometry_msgs::Point getPoint(const geometry_msgs::PoseWithCovarianceStamped & p)
{
  return p.pose.pose.position;
}


inline geometry_msgs::Point getPoint(
  const autoware_msgs::TrajectoryPoint & p)
{
  return p.pose.position;
}
  
template <class T>
geometry_msgs::Pose getPose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}

inline geometry_msgs::Pose getPose(const geometry_msgs::Pose & p)
{
  return p;
}

inline geometry_msgs::Pose getPose(const geometry_msgs::PoseStamped & p)
{
  return p.pose;
}


inline geometry_msgs::Pose getPose(const autoware_msgs::TrajectoryPoint & p)
{
  return p.pose;
}


template <class T>
void validateNonEmpty(const T & points)
{
  if (points.empty()) {
    throw std::invalid_argument("Points is empty.");
  }
}


template <class Point1, class Point2>
double calcSquaredDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}



inline double normalizeDegree(const double deg, const double min_deg = -180)
{
  const auto max_deg = min_deg + 360.0;

  const auto value = std::fmod(deg, 360.0);
  if (min_deg <= value && value < max_deg) {
    return value;
  }

  return value - std::copysign(360.0, value);
}

inline double normalizeRadian(const double rad, const double min_rad = -pi)
{
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}

inline double calcYawDeviation(
  const geometry_msgs::Pose & base_pose, const geometry_msgs::Pose & target_pose)
{
  const auto base_yaw = tf2::getYaw(base_pose.orientation);
  const auto target_yaw = tf2::getYaw(target_pose.orientation);
  return normalizeRadian(target_yaw - base_yaw);
}


template <class T>
size_t findNearestIndex(const T & points, const geometry_msgs::Point & point)
{
  validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = ::calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}


  

template <class T>
size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  {  // with dist and yaw thresholds
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        ::calcSquaredDistance2d(points.at(i), pose.position);
      const auto yaw =
        ::calcYawDeviation(::getPose(points.at(i)), pose);

      if (squared_dist_threshold < squared_dist || yaw_threshold < std::abs(yaw)) {
        if (is_within_constraints) {
          break;
        } else {
          continue;
        }
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  {  // with dist threshold
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        ::calcSquaredDistance2d(points.at(i), pose.position);

      if (squared_dist_threshold < squared_dist) {
        if (is_within_constraints) {
          break;
        } else {
          continue;
        }
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  // without any threshold
  return findNearestIndex(points, pose.position);
}


namespace detail{

constexpr auto kDoublePi = 2.0 * M_PI;
}  // namespace detail

///
/// @brief      Wrap angle to the [-pi, pi] range.
///
/// @details    This method uses the formula suggested in the paper [On wrapping the Kalman filter
///             and estimating with the SO(2) group](https://arxiv.org/pdf/1708.05551.pdf) and
///             implements the following formula:
///             \f$\mathrm{mod}(\alpha + \pi, 2 \pi) - \pi\f$.
///
/// @param[in]  angle  The input angle
///
/// @tparam     T      Type of scalar
///
/// @return     Angle wrapped to the chosen range.
///
template <typename T>
constexpr T wrap_angle(T angle) noexcept
{
  auto help_angle = angle + T(M_PI);
  while (help_angle < T{}) {
    help_angle += T(detail::kDoublePi);
  }
  while (help_angle >= T(detail::kDoublePi)) {
    help_angle -= T(detail::kDoublePi);
  }
  return help_angle - T(M_PI);
}



/// \brief Gets the x value for a point
/// \return The x value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline double x_(const PointT & pt)
{
  return pt.x;
}
/// \brief Gets the x value for a TrajectoryPoint message
/// \return The x value of the point
/// \param[in] pt The point
inline double x_(const autoware_msgs::TrajectoryPoint & pt)
{
  return pt.pose.position.x;
}
/// \brief Gets the y value for a point
/// \return The y value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline double y_(const PointT & pt)
{
  return pt.y;
}
/// \brief Gets the y value for a TrajectoryPoint message
/// \return The y value of the point
/// \param[in] pt The point
inline double y_(const autoware_msgs::TrajectoryPoint & pt)
{
  return pt.pose.position.y;
}

template <typename OUT = float, typename T1, typename T2>
inline OUT squared_distance_2d(const T1 & a, const T2 & b)
{
  const auto x = static_cast<OUT>(x_(a)) - static_cast<OUT>(x_(b));
  const auto y = static_cast<OUT>(y_(a)) - static_cast<OUT>(y_(b));
  return (x * x) + (y * y);
}

template <typename OUT = float, typename T1, typename T2>
inline OUT distance_2d(const T1 & a, const T2 & b)
{
  return std::sqrt(squared_distance_2d<OUT>(a, b));
}

inline double calcAzimuthAngle(
  const geometry_msgs::Point & p_from, const geometry_msgs::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}



template <class Pose1, class Pose2>
bool isDrivingForward(const Pose1 & src_pose, const Pose2 & dst_pose)
{
  // check the first point direction
  const double src_yaw = tf2::getYaw(getPose(src_pose).orientation);
  const double pose_direction_yaw = calcAzimuthAngle(getPoint(src_pose), getPoint(dst_pose));
  return std::fabs(normalizeRadian(src_yaw - pose_direction_yaw)) < pi / 2.0;
}



template <class T>
boost::optional<bool> isDrivingForward(const T points)
{
  if (points.size() < 2) {
    return boost::none;
  }

  // check the first point direction
  const auto & first_pose = getPose(points.at(0));
  const auto & second_pose = getPose(points.at(1));

  return  isDrivingForward(first_pose, second_pose);
}


template <class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class T>
T removeOverlapPoints(const T & points, const size_t & start_idx = 0)
{
  if (points.size() < start_idx + 1) {
    return points;
  }

  T dst;

  for (size_t i = 0; i <= start_idx; ++i) {
    dst.push_back(points.at(i));
  }

  constexpr double eps = 1.0E-08;
  for (size_t i = start_idx + 1; i < points.size(); ++i) {
    const auto prev_p = getPoint(dst.back());
    const auto curr_p = getPoint(points.at(i));
    const double dist = calcDistance2d(prev_p, curr_p);
    if (dist < eps) {
      continue;
    }
    dst.push_back(points.at(i));
  }

  return dst;
}

template <class T>
double calcLongitudinalOffsetToSegment(
  const T & points, const size_t seg_idx, const geometry_msgs::Point & p_target,
  const bool throw_exception = false)
{
  if (seg_idx >= points.size() - 1) {
    const std::out_of_range e("Segment index is invalid.");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return std::nan("");
  }

  //就是把距离很近的点给刷掉，没毛用
  const auto overlap_removed_points = removeOverlapPoints(points, seg_idx);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
      return std::nan("");
    }
  }

  if (seg_idx >= overlap_removed_points.size() - 1) {
    const std::runtime_error e("Same points are given.");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return std::nan("");
  }

  const auto p_front = getPoint(overlap_removed_points.at(seg_idx));
  const auto p_back = getPoint(overlap_removed_points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};
  //target_vec向量在segment_vec向量上的投影
  return segment_vec.dot(target_vec) / segment_vec.norm();
}



template <class T>
size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const T & points, const geometry_msgs::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  // find first nearest index with soft constraints (not segment index)
  const size_t nearest_idx =
    findFirstNearestIndexWithSoftConstraints(points, pose, dist_threshold, yaw_threshold);

  // calculate segment index
  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, pose.position);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}


template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  double dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += calcDistance2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

#endif  // TRAJECTORY_FOLLOWER__MPC_LATERAL_CONTROLLER_HPP_

