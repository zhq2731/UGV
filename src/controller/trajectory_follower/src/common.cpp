
template <class T>
geometry_msgs::Point getPoint(const T & p)
{
  return geometry_msgs::build<geometry_msgs::Point>().x(p.x).y(p.y).z(p.z);
}

template <>
inline geometry_msgs::Point getPoint(const geometry_msgs::Point & p)
{
  return p;
}

template <>
inline geometry_msgs::Point getPoint(const geometry_msgs::Pose & p)
{
  return p.position;
}

template <>
inline geometry_msgs::Point getPoint(const geometry_msgs::PoseStamped & p)
{
  return p.pose.position;
}

template <>
inline geometry_msgs::Point getPoint(const geometry_msgs::PoseWithCovarianceStamped & p)
{
  return p.pose.pose.position;
}


template <>
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

template <>
inline geometry_msgs::Pose getPose(const geometry_msgs::Pose & p)
{
  return p;
}

template <>
inline geometry_msgs::Pose getPose(const geometry_msgs::PoseStamped & p)
{
  return p.pose;
}


template <>
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
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  const auto base_yaw = tf2::getYaw(base_pose.orientation);
  const auto target_yaw = tf2::getYaw(target_pose.orientation);
  return normalizeRadian(target_yaw - base_yaw);
}


template <class T>
size_t findNearestIndex(const T & points, const geometry_msgs::msg::Point & point)
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

