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
 * @file path_data.cc
 **/

#include "trajectory/path_data.h"



namespace ugv {
namespace planning {
using namespace ugv::common::math;
using namespace ugv::common::util;

/*
bool PathData::SetDiscretizedPath(DiscretizedPath path) {
  if (reference_line_ == nullptr) {
    std::cout << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  discretized_path_ = std::move(path);
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    std::cout << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  assert(discretized_path_.size() == frenet_path_.size());
  return true;
}
*/

bool PathData::SetFrenetPath(FrenetFramePath frenet_path) {
  if (reference_line_ == nullptr) {
    std::cout << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first."<<std::endl;
    return false;
  }
  frenet_path_ = std::move(frenet_path);
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    std::cout << "Fail to transfer frenet path to discretized path."<<std::endl;
    return false;
  }
  assert(discretized_path_.size()== frenet_path_.size());
  return true;
}

bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathPointType, double>>
        path_point_decision_guide) {
  if (reference_line_ == nullptr) {
    std::cout << "Should NOT set path_point_decision_guide when reference line is "
              "nullptr. "<<std::endl;
    return false;
  }
  if (frenet_path_.empty() || discretized_path_.empty()) {
    std::cout << "Should NOT set path_point_decision_guide when frenet_path or "
              "world frame trajectory is empty. "<<std::endl;
    return false;
  }
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

const std::vector<std::tuple<double, PathData::PathPointType, double>>
    &PathData::path_point_decision_guide() const {
  return path_point_decision_guide_;
}

bool PathData::Empty() const {
  return discretized_path_.empty() && frenet_path_.empty();
}

void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}

PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    PathPoint *const path_point) const {
  assert(reference_line_);
  assert(discretized_path_.size() == frenet_path_.size());
  if (ref_s < 0) {
    std::cout << "ref_s[" << ref_s << "] should be > 0"<<std::endl;
    return false;
  }
  if (ref_s > frenet_path_.back().s()) {
    std::cout << "ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.back().s() << "]."<<std::endl;
    return false;
  }

  uint32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i) {
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      *path_point = (discretized_path_.at(i));
      return true;
    }
    if (frenet_path_.at(i).s() < ref_s && ref_s <= frenet_path_.at(i + 1).s()) {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());

  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());
  *path_point = discretized_path_.Evaluate(discretized_path_s);

  return true;
}

void PathData::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  path_point_decision_guide_.clear();
  path_reference_.clear();
  reference_line_ = nullptr;
}

bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  std::vector<PathPoint> path_points;
  for (const FrenetFramePoint &frenet_point : frenet_path) {
    SLPoint sl_point =
        PointFactory::ToSLPoint(frenet_point.s(), frenet_point.l());
    Vec2d cartesian_point;
	if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      std::cout  << "Fail to convert sl point to xy point"<<std::endl;
      return false;
    }
	
	PathPoint matched_ref_point = PathMatcher::MatchToPath(reference_line_->getDiscretizedPathPoint(), sl_point.s());

	const double rs = matched_ref_point.s();
	const double rx = matched_ref_point.x();
	const double ry = matched_ref_point.y();
	const double rtheta = matched_ref_point.theta();
	const double rkappa = matched_ref_point.kappa();
	const double rdkappa = matched_ref_point.dkappa();	

	const double theta = CartesianFrenetConverter::CalculateTheta(
        rtheta, rkappa, frenet_point.l(),frenet_point.dl());
	
   // ADEBUG << "frenet_point: " << frenet_point.ShortDebugString();
    const double kappa = CartesianFrenetConverter::CalculateKappa(
        rkappa, rdkappa, frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    double s = 0.0;
    double dkappa = 0.0;
    if (!path_points.empty()) {
      common::math::Vec2d last = PointFactory::ToVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      s = path_points.back().s() + distance;
      dkappa = (kappa - path_points.back().kappa()) / distance;
    }
    path_points.push_back(PointFactory::ToPathPoint(cartesian_point.x(),
                                                    cartesian_point.y(), 0.0, s,
                                                    theta, kappa, dkappa));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

/*
bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  assert(reference_line_);
  std::vector<FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path) {
    FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL(path_point, &sl_point)) {
        std::cout << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

bool PathData::LeftTrimWithRefS(const FrenetFramePoint &frenet_point) {
  assert(reference_line_);
  std::vector<FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.emplace_back(frenet_point);

  for (const FrenetFramePoint fp : frenet_path_) {
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    if (fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(fp);
    }
  }
  SetFrenetPath(FrenetFramePath(std::move(frenet_frame_points)));
  return true;
}

bool PathData::UpdateFrenetFramePath(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}
*/
void PathData::set_path_label(const std::string &label) { path_label_ = label; }

const std::string &PathData::path_label() const { return path_label_; }

const std::vector<PathPoint> &PathData::path_reference() const {
  return path_reference_;
}

void PathData::set_path_reference(
    const std::vector<PathPoint> &path_reference) {
  path_reference_ = std::move(path_reference);
}
	


}  // namespace planning
}  // namespace ugv
