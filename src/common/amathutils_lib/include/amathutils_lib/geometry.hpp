#pragma once

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <array>
#include <vector>
#include <limits>
#include <cfloat>
#include <geometry_msgs/Point.h>

namespace amathutils {

template <class T>
inline T minus_2d(T point1,T point2)
{
    T  point;
	point.x = point1.x - point2.x;
	point.y = point1.y - point2.y;
	return point;
}


template <class T>
inline double distance2D(const T &p1, const T &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}


template <typename U, typename V> 
inline double DistanceXY(const U& u, const V& v) {
  return std::hypot(u.x() - v.x(), u.y() - v.y());
}

template <typename U, typename V>
inline double distance2D(const U& u, const V& v) {
  return std::hypot(u.x - v.x, u.y - v.y);
}


inline double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

template <class T>
inline const T  rotate(const T &in, const double theta) {
  T  out;
  double s = sin(theta);
  double c = cos(theta);

  out.x = in.x * c - in.y * s;
  out.y = in.x * s + in.y * c;

  return out;
}

template <class T>
inline const T  globalToLocal(const T &center, const double theta,
                            const T &p) {
  T delta;
  delta.x = p.x - center.x;
  delta.y = p.y - center.y;
  return rotate(delta, -theta);
}


template <class T>
inline const T localToGlobal(const T &center, const double theta,
                            const T &p) {
  T out = rotate(p, theta);
  out.x += center.x;
  out.y += center.y;
  return out;
}



							
template <class T>
inline double dot(const T &a, const T &b) {
  return a.x * b.x + a.y * b.y;
}


template <class T>
inline T getProjectedPointOnLine(const T &line_start,
                                const T &line_end, const T &p) {
  // get dot product of e1, e2
  T e1(line_end.x - line_start.x, line_end.y - line_start.y);
  T e2(p.x - line_start.x, p.y - line_start.y);
  double val = dot(e1, e2);
  double len2 = e1.x * e1.x + e1.y * e1.y;
  return T((line_start.x + (val * e1.x) / len2),
                 (line_start.y + (val * e1.y) / len2));
}

								
template <class T>
inline double distanceToLine(const T start, const T end,
                      const T point) {
  double normalLength = std::hypot(end.x - start.x, end.y - start.y);
  double distance = (double)((point.x - start.x) * (end.y - start.y) -
                             (point.y - start.y) * (end.x - start.x)) /
                    normalLength;
  return fabs(distance);
}


template <class T>
inline double distanceToTrajectory(std::vector<T> ref_path, double x, double y) {
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, x, y);
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  return min_dist;
}

template <typename T,typename V>
inline double distanceToTrajectory(std::vector<T> ref_path, V &point) {
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p, point);
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  return min_dist;
}


template <class T>
inline int closestPoint(std::vector<T> ref_path, double x, double y) {
  int index = 0;
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, x, y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}


template <class T>
inline int closestPoint(std::vector<T> ref_path, T query) {
  int index = 0;
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, query.x, query.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}


template <typename T,typename V>
inline int closestPoint(std::vector<T> ref_path, V query) {
  int index = 0;
  int closest_index = 0;
  double min_dist = DBL_MAX;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, query.x, query.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}


template <class T>
inline T getIntersectionPointWithCurve(const T &line_start,
                                      const T &line_end,
                                      std::vector<T> ref_path) {
  int n = 100;
  double min_dist = std::numeric_limits<double>::max();
  T intersection = line_start;
  double distance = distance2D(line_start, line_end);
  for (int i = 0; i <= n; i++) {
    double cur_x = line_start.x + (line_end.x - line_start.x) / n * i;
    double cur_y = line_start.y + (line_end.y - line_start.y) / n * i;
    T cur_p(cur_x, cur_y);
    int idx = closestPoint(ref_path, cur_x, cur_y);
    if (min_dist > distance2D(cur_p, ref_path[idx])) {
      min_dist = distance2D(cur_p, ref_path[idx]);
      intersection = cur_p;
    }
  }
  return intersection;
}


template <class T>
inline bool isOnPath(std::vector<T> path, T check_p, double thresh,double &dis) {
  int closest_idx = closestPoint(path, check_p.x, check_p.y);
  T pa, pb;
  if (closest_idx == path.size() - 1) {
    pa = path[closest_idx - 1];
    pb = path[closest_idx];
  } else {
    pa = path[closest_idx];
    pb = path[closest_idx + 1];
  }
  dis = distanceToLine(pa, pb, check_p);
  return dis < thresh;
}


//a*b*c = 4 * R * S
template <class T>
void caculateKappa(
  const size_t curvature_smoothing_num,   std::vector<T> &points)
{
  if (points.size() < 3)
      return ;
  /* calculate curvature by circle fitting from three points */
  T p1, p2, p3;
  const size_t max_smoothing_num =
    static_cast<size_t>(std::floor(0.5 * (static_cast<double>(points.size() - 1))));
  const size_t L = std::min(curvature_smoothing_num, max_smoothing_num);
  for (size_t i = L; i < points.size() - L; ++i) {
    const size_t curr_idx = i;
    const size_t prev_idx = curr_idx - L;
    const size_t next_idx = curr_idx + L;
    p1.x = points[prev_idx].x;
    p2.x = points[curr_idx].x;
    p3.x = points[next_idx].x;
    p1.y = points[prev_idx].y;
    p2.y = points[curr_idx].y;
    p3.y = points[next_idx].y;
    const double den = std::max(
      distance2D(p1, p2) * distance2D(p2, p3) * distance2D(p3, p1),
      std::numeric_limits<double>::epsilon());
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
      points.at(curr_idx).kappa = curvature;
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, points.size()); ++i) {
    points.at(i).kappa = points.at(std::min(L, points.size() - 1)).kappa;
    points.at(points.size() - i - 1).kappa =
      points.at(std::max(points.size() - L - 1, size_t(0))).kappa;
  }
 }


template <class T>
void caculateAccumulated_s(    std::vector<T> &points)
{
    if (0 == points.size())
		return ;
	
    double s = 0.0;
    points[0].s = s;
	
	for (std::size_t i = 1; i < points.size(); ++i) {
		s += distance2D(points[i],points[i-1]);
	    points[i].s =s;
	}
}


template <class T>
void caculateDkappa(  std::vector<T> &points)
{
	// Dkappa calculation
	for (std::size_t i = 0; i < points.size(); ++i) {
	    T p1,p2,p3;
	    double dkappa = 0.0;
	    if (i == 0) {
		    dkappa = (points[i+1].kappa - points[i].kappa) /
				 (points[i+1].s - points[i].s);
	    } else if (i == points.size() - 1) {
		    dkappa = (points[i].kappa - points[i-1].kappa) /
				 (points[i].s - points[i-1].s);
	    } else {
		    dkappa = (points[i+1].kappa - points[i-1].kappa) /
				 (points[i+1].s - points[i-1].s);
	  }
	  points[i].dkappa = dkappa;
	}
}

template <class T>
void caculateHeading(  std::vector<T> &points)
{
	if (points.size() < 3)
		return ;
	
	for (size_t i = 0; i < points.size(); i++) {
		float angle = 0;
		auto & pt = points.at(i);
		if (i + 1 < points.size()) {
		  const auto & next_pt = points.at(i + 1);
		  angle = std::atan2(
		    next_pt.y - pt.y,
		    next_pt.x - pt.x);
		} else if (i != 0) {
		  const auto & prev_pt = points.at(i - 1);
		  angle = std::atan2(
		    pt.y - prev_pt.y,
		    pt.x - prev_pt.x);
		}
		//std::cout <<"angle "<<angle/3.1415926 * 180<<std::endl;
	    //pt.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(angle);
		//std::cout <<"orientation "<<pt.pose.orientation.w<<std::endl;
		points.at(i).theta = angle;
	}
}


template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

template <typename T>
inline T add_2d(T  point1,T   point2)
{
    T  point;
	point.x = point1.x + point2.x;
	point.y = point1.y + point2.y;
	return point;
}

template <typename T> 
inline T scaled_2d(T point1,double scale)
{
    T  point;
	point.x = point1.x*scale;
	point.y = point1.y*scale;
	return point;
}



template <typename T> 
inline std::pair<size_t, size_t> findNearestIndexPair(
  const std::vector<T> & accumulated_lengths, const T target_length)
{
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (size_t i = 1; i < N; ++i) {
    if (
      accumulated_lengths.at(i - 1) <= target_length &&
      target_length <= accumulated_lengths.at(i))
    {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error(
          "findNearestIndexPair(): No nearest point found.");
}



template <typename T, typename V>
int getClosestIndex(const int referIndex, const std::vector<T> &points,const V pos)
{
	double closest_distance = std::numeric_limits<double>::max();
	int closest_index = 0;
	for (int i = referIndex; i < points.size(); i++) {
		double temp = amathutils::distance2D(pos,points[i]);
		if (temp <= closest_distance) {
		     closest_distance = temp;
		     closest_index = i;
		     continue;
		}
		
		if (0 != referIndex) //如果referIndex = 0，则查询整条轨迹
		    break;
	}

	
	for (int i = referIndex-1; i >= 0; i--) {
	   double temp = amathutils::distance2D(pos,points[i]);
	   if (temp <= closest_distance) {
		   closest_distance = temp;
		   closest_index = i;
		   continue;
	   }
	   break;
	}
	
	if(closest_index == 0)
	    closest_index = 1;

	//if(closest_index == (points.size()-1)) //为了在调试的时候方便，轨迹绕一圈的情况
	 // closest_index = 0;

	return closest_index;
}  

template <class T>
inline double trajectoryLength(const std::vector<T> &path,int firstIndex,int secondIndex) {

  assert(firstIndex >= 0);
  assert(secondIndex <= (path.size()-1));
  assert(firstIndex <= secondIndex);
  double length = 0.0;
  for (int i = firstIndex; i < secondIndex; i++) {
      length += distance2D(path[i], path[i+1]);
  }
  return length;
}


template <typename T> 
void resamplePoints(double resolution,std::vector<T>  &pts) 
{
     double acculate_s = 0;
	 std::vector<double> accumulated_lengths;
	 accumulated_lengths.push_back(0.0);
	 for (auto i  = 1;i < pts.size();i++){
	     acculate_s += distance2D(pts[i],pts[i-1]); 
		 accumulated_lengths.push_back(acculate_s);
	 }
	 
	 const int32_t num_segments =
	    std::max(static_cast<int32_t>(ceil(acculate_s / resolution)), 1);

	std::vector<T> resampled_points;
	for (auto i = 0; i <= num_segments; ++i) {
		// Find two nearest points
		const double target_length = (static_cast<double>(i) / num_segments) * acculate_s;
		const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

		// Apply linear interpolation
		const T back_point = pts[index_pair.first];
		const T front_point = pts[index_pair.second];


		const auto direction_vector = minus_2d(front_point,back_point);

		const auto back_length = accumulated_lengths.at(index_pair.first);
		const auto front_length = accumulated_lengths.at(index_pair.second);
		const auto segment_length = front_length - back_length;
		const auto target_point = add_2d(back_point,scaled_2d(direction_vector,(target_length - back_length) / segment_length));
		resampled_points.push_back(target_point);
	}
    std::vector<T>().swap(pts);
	pts = resampled_points;
	
}


template <typename U, typename V>
inline double cross(const U& u, const V& v) {
  return u.x * v.y - u.y *v.x;
}

template <typename T>
inline double cross(const T& u, const T& v) {
  return u.x * v.y - u.y *v.x;
}

template <typename U, typename V>
inline double abs_cross(const U& u, const V& v)
{
   return fabs(cross(u,v));
}

template <typename T>
inline double abs_cross(const T& u, const T& v)
{
   return fabs(cross(u,v));
}

template <class T>
inline double trajectoryLength(const std::vector<T> &path) {

  double length = 0.0;
  for (int i = 0; i< path.size()-1;i++) {
    length += distance2D(path[i], path[i+1]);
  }
  return length;
}


template <class T>
inline double printTraj(const std::vector<T> &path) {

  double length = 0.0;
  for (int i = 0; i< path.size()-1;i++) {
      std::cout <<"index  "<<i <<" x: "<<path[i].x <<" y: "<<path[i].y <<std::endl;
  }
  return length;
}


/*
inline bool isOnPath(std::vector<Pose2D> path, Point2D check_p, double thresh,double &dis) {
  std::vector<Point2D> points(path.size());
  for (int i = 0; i < path.size(); i++) {
    points[i] = path[i].position;
  }
  return isOnPath(points, check_p, thresh,dis);
}
*/
} // namespace geometry

#endif // GEOMETRY_H
