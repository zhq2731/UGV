/******************************************************************************
 * Copyright 2017 The ugv Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "trajectory/reference_line.h"

namespace ugv {
namespace planning {


using namespace ugv::planning;
using namespace ugv::common::math;

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points) {

	 if (0 == reference_points.size())
	 	return;
	 
	 discrete_path_points = ToDiscretizedReferenceLine(reference_points);
	 for (int i = 0; i < discrete_path_points.size();i++)
	 {
		 accumulated_s_.push_back(discrete_path_points[i].s());
	 }
	 length_ = discrete_path_points.back().s();
	 planning_config = PlanningConfig::get_instance();
}


size_t ReferenceLine::GetNearestReferenceIndex(
    const Vec2d& xy) const {
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < reference_points_.size(); ++i) {
    const double distance = amathutils::DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return min_index;
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(
    const Vec2d& xy) const {
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < reference_points_.size(); ++i) {
    const double distance = amathutils::DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}


/*
FrenetFramePoint ReferenceLine::GetFrenetPoint(
    const PathPoint& path_point) const {
  if (reference_points_.empty()) {
    return FrenetFramePoint();
  }

  common::SLPoint sl_point;
  XYToSL(path_point, &sl_point);
  FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();
  const double kappa = path_point.kappa();
  const double l = frenet_frame_point.l();

  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}


*/

std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(const TrajectoryPoint& traj_point) const {
  std::array<double, 3> s_condition;
  std::array<double, 3> l_condition;
  auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_points()));
  PathPoint ref_point = PathMatcher::MatchToPath(*ptr_reference_line, traj_point.path_point().x(),
	traj_point.path_point().y());

  CartesianFrenetConverter::cartesian_to_frenet(
      ref_point.s(), ref_point.x(), ref_point.y(), ref_point.theta(),
      ref_point.kappa(), ref_point.dkappa(), traj_point.path_point().x(),
      traj_point.path_point().y(), traj_point.v(), traj_point.a(),
      traj_point.path_point().theta(), traj_point.path_point().kappa(),
      &s_condition, &l_condition);

  return std::make_pair(s_condition, l_condition);
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const {

  if (s < accumulated_s_.front() - 1e-2) {
    std::cout << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }
  if (s > accumulated_s_.back() + 1e-2) {
    std::cout << "The requested s: " << s
          << " > reference line length: " << accumulated_s_.back();
    return reference_points_.back();
  }
  auto it_lower =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (it_lower == accumulated_s_.begin()) {
    return reference_points_.front();
  }
  auto index = std::distance(accumulated_s_.begin(), it_lower);
  if (std::fabs(accumulated_s_[index - 1] - s) <
      std::fabs(accumulated_s_[index] - s)) {
    return reference_points_[index - 1];
  }
  return reference_points_[index];
}


size_t ReferenceLine::GetNearestReferenceIndex(const double s) const {

  if (s < accumulated_s_.front() - 1e-2) {
    std::cout << "The requested s: " << s << " < 0.";
    return 0;
  }
  if (s > accumulated_s_.back() + 1e-2) {
    std::cout << "The requested s: " << s << " > reference line length "
          << accumulated_s_.back();
    return reference_points_.size() - 1;
  }
  auto it_lower =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  return std::distance(accumulated_s_.begin(), it_lower);
}

std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(
    double start_s, double end_s) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > Length()) {
    end_s = Length();
  }
  std::vector<ReferencePoint> ref_points;
  std::cout <<"GetNearestReferenceIndex "<<std::endl;
  auto start_index = GetNearestReferenceIndex(start_s);
  
  std::cout <<"GetNearestReferenceIndex2 "<<std::endl;
  auto end_index = GetNearestReferenceIndex(end_s);
  
  std::cout <<"GetNearestReferenceIndex3 "<<std::endl;
  if (start_index < end_index) {
    ref_points.assign(reference_points_.begin() + start_index,
                      reference_points_.begin() + end_index);
  }
  return ref_points;
}


bool ReferenceLine::SLToXY(const SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const {

	PathPoint matched_ref_point = PathMatcher::MatchToPath(discrete_path_points, sl_point.s());
	const double rs = matched_ref_point.s();
	const double rx = matched_ref_point.x();
	const double ry = matched_ref_point.y();
	const double rtheta = matched_ref_point.theta();
	const double rkappa = matched_ref_point.kappa();
	const double rdkappa = matched_ref_point.dkappa();						   
	Vec2d xy = CartesianFrenetConverter::CalculateCartesianPoint(rtheta,Vec2d(rx,ry),sl_point.l());
    xy_point->set_x(xy.x());
    xy_point->set_y(xy.y());
	return true;
}


//输出 s ds/dt l,     输入 x y theta v						   
std::array<double,3> ReferenceLine::XYToS_SD_L(std::array<double,4> trajectoryPoint) const{

    double x  = trajectoryPoint[0];
    double y  = trajectoryPoint[1];
    double theta = trajectoryPoint[2];
    double v = trajectoryPoint[3];
	
	auto matched_path_point = PathMatcher::MatchToPath(discrete_path_points, x, y);
	double rs = matched_path_point.s();
	double rtheta = matched_path_point.theta();
	double rx = matched_path_point.x();
	double ry = matched_path_point.y();
	const double rkappa = matched_path_point.kappa();
	const double rdkappa = matched_path_point.dkappa();	

	double delta_x = x - rx;
	double delta_y = y - ry;
	double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;

	std::array<double,3> s_sd_l;
	s_sd_l[0] = matched_path_point.s();
	s_sd_l[2] =
		std::copysign(std::hypot(delta_x, delta_y), side);

	const double delta_theta = theta - rtheta;
	const double cos_delta_theta = std::cos(delta_theta);
	const double one_minus_kappa_r_d = 1 - rkappa * s_sd_l[2];
	s_sd_l[1] = v * cos_delta_theta / one_minus_kappa_r_d;
	return s_sd_l;
}



bool ReferenceLine::XYToSL(const Vec2d& xy_point,
                           SLPoint* const sl_point) const {

  auto sl_point_ = PathMatcher::GetPathFrenetCoordinate(
		discrete_path_points,xy_point.x(),xy_point.y());

  sl_point->set_s(sl_point_.first) ;
  sl_point->set_l(sl_point_.second) ;
  
  return true;
}

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}


bool ReferenceLine::GetLaneWidth(const double s, double* const lane_left_width,
                                 double* const lane_right_width) const {
     
     *lane_left_width  = planning_config->kDefaultLaneWidth / 2.0;
     *lane_right_width = planning_config->kDefaultLaneWidth / 2.0;
     return true;
}



double ReferenceLine::GetDrivingWidth(const SLBoundary& sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s(), &lane_left_width, &lane_right_width);

  //可供通行的宽度
  double driving_width = std::max(lane_left_width - sl_boundary.end_l(),
                                  lane_right_width + sl_boundary.start_l());
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  ////ADEBUG << "Driving width [" << driving_width << "].";
  return driving_width;
}

bool ReferenceLine::IsOnLane(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
     return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  return sl_boundary.start_l() <= lane_left_width &&
         sl_boundary.end_l() >= -lane_right_width;
}

bool ReferenceLine::IsOnLane(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > Length()) {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s(), &left_width, &right_width)) {
    return false;
  }

  return sl_point.l() >= -right_width && sl_point.l() <= left_width;
}


bool ReferenceLine::IsOnLane(const Vec2d& vec2d_point) const {
  SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnLane(sl_point);
}


bool ReferenceLine::GetSLBoundary(const Polygon2d &polygon,
									SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<Vec2d> corners;
  corners = polygon.GetAllVertices();

  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      std::cout << "Failed to get projection for point: " << " on reference line."<<std::endl;;
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}



bool ReferenceLine::GetSLBoundary(const Box2d& box,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);

  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      std::cout << "Failed to get projection for point: " 
             << " on reference line.";
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % corners.size();
    const auto& p0 = corners[index0];
    const auto& p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    SLPoint sl_point_mid;
    if (!XYToSL(p_mid, &sl_point_mid)) {
      std::cout << "Failed to get projection for point: " 
             << " on reference line.";
      return false;
    }

    Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(),
             sl_corners[index1].l() - sl_corners[index0].l());

    Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(),
             sl_point_mid.l() - sl_corners[index0].l());

    sl_boundary->boundary_point.push_back(sl_corners[index0]) ;

    // sl_point is outside of polygon; add to the vertex list
    if (v0.CrossProd(v1) < 0.0) {
		
	  sl_boundary->boundary_point.push_back(sl_point_mid) ;
    }
  }

  for (const auto& sl_point : sl_boundary->boundary_point ){
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }


  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

/*
ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  const auto& accumulated_s = accumulated_s_;
  if (s < accumulated_s.front() - 1e-2) {
    std::cout << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    std::cout << "The requested s: " << s
          << " > reference line length: " << accumulated_s.back();
    return reference_points_.back();
  }

  PathPoint mach_point = PathMatcher::MatchToPath(getDiscretizedPathPoint(),s);

  size_t index = interpolate_index.id;
  size_t next_index = index + 1;
  if (next_index >= reference_points_.size()) {
    next_index = reference_points_.size() - 1;
  }

  const auto& p0 = reference_points_[index];
  const auto& p1 = reference_points_[next_index];

  const double s0 = accumulated_s[index];
  const double s1 = accumulated_s[next_index];
  return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}
*/

void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  std::vector<SpeedLimit> new_speed_limit;
  for (const auto& limit : speed_limit_) {
    if (start_s >= limit.end_s || end_s <= limit.start_s) {
      new_speed_limit.emplace_back(limit);
    } else {
      // start_s < speed_limit.end_s && end_s > speed_limit.start_s
      double min_speed = std::min(limit.speed_limit, speed_limit);
      if (start_s >= limit.start_s) {
        new_speed_limit.emplace_back(limit.start_s, start_s, min_speed);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed);
        }
      } else {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed);
        }
      }
      start_s = limit.end_s;
      end_s = std::max(end_s, limit.end_s);
    }
  }
  speed_limit_.clear();
  if (end_s > start_s) {
    new_speed_limit.emplace_back(start_s, end_s, speed_limit);
  }
  for (const auto& limit : new_speed_limit) {
    if (limit.start_s < limit.end_s) {
      speed_limit_.emplace_back(limit);
    }
  }
  std::sort(speed_limit_.begin(), speed_limit_.end(),
            [](const SpeedLimit& a, const SpeedLimit& b) {
              if (a.start_s != b.start_s) {
                return a.start_s < b.start_s;
              }
              if (a.end_s != b.end_s) {
                return a.end_s < b.end_s;
              }
              return a.speed_limit < b.speed_limit;
            });
}


std::vector<PathPoint> ReferenceLine::ToDiscretizedReferenceLine (
	const std::vector<ReferencePoint>& ref_points)const {
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
	PathPoint path_point;
	path_point.set_x(ref_point.x());
	path_point.set_y(ref_point.y());
	path_point.set_theta(ref_point.heading());
	path_point.set_kappa(ref_point.kappa());
	path_point.set_dkappa(ref_point.dkappa());

	if (!path_points.empty()) {
	  double dx = path_point.x() - path_points.back().x();
	  double dy = path_point.y() - path_points.back().y();
	  s += std::sqrt(dx * dx + dy * dy);
	}
	path_point.set_s(s);
	path_points.push_back(std::move(path_point));
  }
  return path_points;
}


}  // namespace planning
}  // namespace ugv
