// Created for OpenMower: coverage-feedback re-mow.
// Copyright (c) 2024 OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
// coverage_feedback
// -----------------
// Two responsibilities in one node:
//
//   A. Coverage tracker (always-on). Subscribes to the body pose and the low-level mower status and,
//      while the blade is enabled (which already encodes "actively mowing under acceptable position
//      accuracy" - see ticket), stamps a disc of radius tool_width/2 along the path into a map-frame
//      occupancy grid. The grid is published for RViz.
//
//   B. Gap detect + refill (on demand, via the GetFillPaths service). Rasterizes the area polygon
//      minus its obstacles, erodes it by erosion_margin (so the outline/keepout rim is excluded),
//      subtracts the covered grid, declutters the difference (morphological opening + connected
//      components with min width/area thresholds), and for each surviving gap builds a polygon and
//      requests a linear fill from slic3r along the gap's major axis (PCA). The aggregated fill paths
//      are returned so MowingBehavior can run them before docking.

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/Status.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <slic3r_coverage_planner/PlanPath.h>
#include <xbot_msgs/AbsolutePose.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "coverage_feedback/GetFillPaths.h"

namespace {

// Marker values for the single-channel coverage grid (CV_8UC1).
constexpr unsigned char CELL_COVERED = 255;

class CoverageFeedback {
 public:
  explicit CoverageFeedback(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
    // --- parameters (defaults mirror the ticket's conservative starting dial) ---
    private_nh.param("grid_resolution", res_, 0.05);
    private_nh.param("tool_width", tool_width_, 0.14);
    // min_gap_width / erosion_margin default to tool_width-derived values when not set explicitly.
    private_nh.param("min_gap_width", min_gap_width_, tool_width_);
    private_nh.param("min_gap_area", min_gap_area_, 0.04);
    private_nh.param("residual_stop_area", residual_stop_area_, 0.05);
    private_nh.param("erosion_margin", erosion_margin_, tool_width_ / 2.0);
    private_nh.param("max_connect_gap", max_connect_gap_, 0.5);
    private_nh.param<std::string>("map_frame", map_frame_, "map");
    private_nh.param<std::string>("pose_topic", pose_topic_, "/xbot_positioning/xb_pose");
    private_nh.param<std::string>("status_topic", status_topic_, "/ll/mower_status");
    private_nh.param<std::string>("state_topic", state_topic_, "/mower_logic/current_state");
    private_nh.param("publish_grid", publish_grid_, true);

    plan_client_ = nh.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");

    pose_sub_ = nh.subscribe(pose_topic_, 50, &CoverageFeedback::onPose, this);
    status_sub_ = nh.subscribe(status_topic_, 10, &CoverageFeedback::onStatus, this);
    // Reset the coverage grid when a new mowing job starts so stale coverage from a previous job
    // (e.g. yesterday's mow, with the node still running) is not mistaken for freshly cut ground.
    state_sub_ = nh.subscribe(state_topic_, 10, &CoverageFeedback::onState, this);
    fill_service_ = private_nh.advertiseService("get_fill_paths", &CoverageFeedback::onGetFillPaths, this);

    if (publish_grid_) {
      grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("coverage_grid", 1, true);
      grid_timer_ = nh.createTimer(ros::Duration(1.0), &CoverageFeedback::publishGrid, this);
    }

    ROS_INFO_STREAM("coverage_feedback: res=" << res_ << "m tool_width=" << tool_width_ << "m min_gap_width="
                                              << min_gap_width_ << "m min_gap_area=" << min_gap_area_
                                              << "m^2 erosion_margin=" << erosion_margin_ << "m");
  }

 private:
  // -------------------------------------------------------------------------
  // Coverage tracker
  // -------------------------------------------------------------------------
  void onStatus(const mower_msgs::Status::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool was_mowing = mowing_active_;
    mowing_active_ = msg->mow_enabled;
    // Dropping the blade (pause / transit / area boundary) breaks the swath: don't draw a
    // connecting line across the gap when mowing resumes somewhere else.
    if (was_mowing && !mowing_active_) {
      have_prev_ = false;
    }
  }

  void onState(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // A new, non-empty job id means a fresh mow: drop accumulated coverage. Resuming the same job
    // after a dock keeps the same id, so the grid (and thus per-area progress) is preserved.
    if (!msg->job_id.empty() && msg->job_id != last_job_id_) {
      if (initialized_) {
        ROS_INFO_STREAM("coverage_feedback: new job '" << msg->job_id << "' - resetting coverage grid.");
      }
      resetGrid();
    }
    last_job_id_ = msg->job_id;
  }

  void resetGrid() {
    grid_.release();
    initialized_ = false;
    have_prev_ = false;
  }

  void onPose(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!mowing_active_) {
      have_prev_ = false;
      return;
    }
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    stamp(x, y);
  }

  // Ensure the grid contains world point (x, y) with a margin of slack, growing the cv::Mat and
  // shifting the origin if needed. Row index increases with +y so the Mat maps 1:1 onto a ROS
  // OccupancyGrid (data row-major from the origin corner, +x then +y).
  void ensureContains(double x, double y) {
    constexpr int kMargin = 32;  // cells of slack kept on every side to amortise reallocations
    if (!initialized_) {
      origin_x_ = x - kMargin * res_;
      origin_y_ = y - kMargin * res_;
      grid_ = cv::Mat::zeros(2 * kMargin + 1, 2 * kMargin + 1, CV_8UC1);
      initialized_ = true;
    }

    int c = static_cast<int>(std::round((x - origin_x_) / res_));
    int r = static_cast<int>(std::round((y - origin_y_) / res_));

    int add_left = c < kMargin ? kMargin - c : 0;
    int add_right = c >= grid_.cols - kMargin ? c - (grid_.cols - kMargin) + 1 : 0;
    int add_bottom = r < kMargin ? kMargin - r : 0;
    int add_top = r >= grid_.rows - kMargin ? r - (grid_.rows - kMargin) + 1 : 0;

    if (add_left || add_right || add_bottom || add_top) {
      cv::Mat bigger = cv::Mat::zeros(grid_.rows + add_bottom + add_top, grid_.cols + add_left + add_right, CV_8UC1);
      grid_.copyTo(bigger(cv::Rect(add_left, add_bottom, grid_.cols, grid_.rows)));
      grid_ = bigger;
      origin_x_ -= add_left * res_;
      origin_y_ -= add_bottom * res_;
    }
  }

  cv::Point toCell(double x, double y) const {
    return cv::Point(static_cast<int>(std::round((x - origin_x_) / res_)),
                     static_cast<int>(std::round((y - origin_y_) / res_)));
  }

  // Stamp the swath at (x, y): a disc of radius tool_width/2, plus a thick segment back to the
  // previous sample so a sparse pose stream still paints a continuous strip.
  void stamp(double x, double y) {
    ensureContains(x, y);
    const int radius = std::max(1, static_cast<int>(std::round((tool_width_ / 2.0) / res_)));
    const cv::Point p = toCell(x, y);
    cv::circle(grid_, p, radius, cv::Scalar(CELL_COVERED), cv::FILLED);
    if (have_prev_ && std::hypot(x - prev_x_, y - prev_y_) <= max_connect_gap_) {
      cv::line(grid_, toCell(prev_x_, prev_y_), p, cv::Scalar(CELL_COVERED), 2 * radius);
    }
    prev_x_ = x;
    prev_y_ = y;
    have_prev_ = true;
  }

  void publishGrid(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = res_;
    msg.info.width = grid_.cols;
    msg.info.height = grid_.rows;
    // OccupancyGrid origin is the real-world pose of the (0,0) cell corner; our origin_ is the cell centre.
    msg.info.origin.position.x = origin_x_ - res_ / 2.0;
    msg.info.origin.position.y = origin_y_ - res_ / 2.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(static_cast<size_t>(grid_.cols) * grid_.rows);
    for (int r = 0; r < grid_.rows; ++r) {
      const unsigned char* row = grid_.ptr<unsigned char>(r);
      for (int c = 0; c < grid_.cols; ++c) {
        msg.data[static_cast<size_t>(r) * grid_.cols + c] = row[c] ? 100 : 0;
      }
    }
    grid_pub_.publish(msg);
  }

  // -------------------------------------------------------------------------
  // Gap detect + refill
  // -------------------------------------------------------------------------
  bool onGetFillPaths(coverage_feedback::GetFillPaths::Request& req, coverage_feedback::GetFillPaths::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    res.uncovered_area = 0.0;
    res.gap_count = 0;

    const double tool_width = req.tool_width > 0.0 ? req.tool_width : tool_width_;

    if (!initialized_) {
      ROS_WARN_STREAM("coverage_feedback: no coverage accumulated yet - cannot assess gaps, skipping refill.");
      return true;
    }
    if (req.area.area.points.size() < 3) {
      ROS_WARN_STREAM("coverage_feedback: area polygon has < 3 points, skipping refill.");
      return true;
    }

    // --- window aligned to the global grid so the covered region copies by integer offset ---
    double min_x = req.area.area.points.front().x, max_x = min_x;
    double min_y = req.area.area.points.front().y, max_y = min_y;
    for (const auto& pt : req.area.area.points) {
      min_x = std::min<double>(min_x, pt.x);
      max_x = std::max<double>(max_x, pt.x);
      min_y = std::min<double>(min_y, pt.y);
      max_y = std::max<double>(max_y, pt.y);
    }
    const double pad = tool_width + 4 * res_;
    min_x -= pad;
    min_y -= pad;
    max_x += pad;
    max_y += pad;

    const int c0 = static_cast<int>(std::floor((min_x - origin_x_) / res_));
    const int r0 = static_cast<int>(std::floor((min_y - origin_y_) / res_));
    const double win_origin_x = origin_x_ + c0 * res_;
    const double win_origin_y = origin_y_ + r0 * res_;
    const int cols = static_cast<int>(std::ceil((max_x - win_origin_x) / res_)) + 1;
    const int rows = static_cast<int>(std::ceil((max_y - win_origin_y) / res_)) + 1;
    if (cols <= 0 || rows <= 0 || static_cast<long>(cols) * rows > 50'000'000L) {
      ROS_WARN_STREAM("coverage_feedback: unreasonable gap window " << cols << "x" << rows << ", skipping refill.");
      return true;
    }

    auto worldToWin = [&](double x, double y) {
      return cv::Point(static_cast<int>(std::round((x - win_origin_x) / res_)),
                       static_cast<int>(std::round((y - win_origin_y) / res_)));
    };

    // --- target = area minus obstacles, eroded by erosion_margin ---
    cv::Mat target = cv::Mat::zeros(rows, cols, CV_8UC1);
    fillPolygon(target, req.area.area, worldToWin, CELL_COVERED);
    for (const auto& hole : req.area.obstacles) {
      fillPolygon(target, hole, worldToWin, 0);
    }
    const int erode_px = std::max(1, static_cast<int>(std::round(erosion_margin_ / res_)));
    cv::erode(target, target,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erode_px + 1, 2 * erode_px + 1)));

    // --- covered window, copied from the global grid by integer offset ---
    cv::Mat covered = cv::Mat::zeros(rows, cols, CV_8UC1);
    const cv::Rect global_rect(0, 0, grid_.cols, grid_.rows);
    const cv::Rect want_rect(c0, r0, cols, rows);
    const cv::Rect inter = global_rect & want_rect;
    if (inter.area() > 0) {
      grid_(inter).copyTo(covered(cv::Rect(inter.x - c0, inter.y - r0, inter.width, inter.height)));
    }

    // --- uncovered = target AND NOT covered ---
    cv::Mat uncovered;
    cv::bitwise_and(target, ~covered, uncovered);

    // Declutter: opening removes connected regions narrower than min_gap_width everywhere.
    const int open_px = std::max(1, static_cast<int>(std::round((min_gap_width_ / 2.0) / res_)));
    cv::morphologyEx(uncovered, uncovered, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * open_px + 1, 2 * open_px + 1)));

    cv::Mat labels, stats, centroids;
    const int num = cv::connectedComponentsWithStats(uncovered, labels, stats, centroids, 8, CV_32S);

    const double cell_area = res_ * res_;
    std::vector<int> kept;
    double total_uncovered = 0.0;
    for (int label = 1; label < num; ++label) {
      const double area_m2 = stats.at<int>(label, cv::CC_STAT_AREA) * cell_area;
      if (area_m2 < min_gap_area_) continue;
      kept.push_back(label);
      total_uncovered += area_m2;
    }
    res.uncovered_area = total_uncovered;

    if (kept.empty() || total_uncovered < residual_stop_area_) {
      ROS_INFO_STREAM("coverage_feedback: " << kept.size() << " gap(s), " << total_uncovered
                                            << " m^2 uncovered < residual_stop_area (" << residual_stop_area_
                                            << ") - no refill.");
      return true;
    }

    // --- one slic3r linear fill per kept gap, aligned to the gap's major axis ---
    const int dilate_px = std::max(1, static_cast<int>(std::round((tool_width / 2.0) / res_)));
    const cv::Mat dilate_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilate_px + 1, 2 * dilate_px + 1));
    for (const int label : kept) {
      cv::Mat mask = (labels == label);  // CV_8U, 255 where this component

      const double angle = majorAxisAngle(mask);

      // Dilate by tool_width/2 so the fill overlaps the existing coverage (no new sliver at the seam).
      cv::Mat dilated;
      cv::dilate(mask, dilated, dilate_kernel);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (contours.empty()) continue;
      const auto& contour = *std::max_element(
          contours.begin(), contours.end(),
          [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) { return a.size() < b.size(); });

      geometry_msgs::Polygon poly;
      poly.points.reserve(contour.size());
      for (const auto& cp : contour) {
        geometry_msgs::Point32 wp;
        wp.x = static_cast<float>(win_origin_x + cp.x * res_);
        wp.y = static_cast<float>(win_origin_y + cp.y * res_);
        wp.z = 0.0f;
        poly.points.push_back(wp);
      }

      slic3r_coverage_planner::PlanPath plan;
      plan.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
      plan.request.angle = angle;
      plan.request.distance = tool_width;
      plan.request.outer_offset = 0.0;
      plan.request.outline_count = 0;
      plan.request.outline_overlap_count = 0;
      plan.request.skip_area_outline = true;  // fill only, no perimeter pass around the gap
      plan.request.skip_obstacle_outlines = true;
      plan.request.skip_fill = false;
      plan.request.outline = poly;
      // holes left empty
      if (!plan_client_.call(plan)) {
        ROS_WARN_STREAM("coverage_feedback: slic3r plan_path call failed for a gap, skipping it.");
        continue;
      }
      if (plan.response.paths.empty()) continue;
      res.paths.insert(res.paths.end(), plan.response.paths.begin(), plan.response.paths.end());
      res.gap_count++;
    }

    ROS_INFO_STREAM("coverage_feedback: refill plan ready - " << res.gap_count << " gap(s), " << total_uncovered
                                                              << " m^2 uncovered, " << res.paths.size()
                                                              << " fill path(s).");
    return true;
  }

  // Fill a polygon (world coords) into a window mask with the given value.
  template <typename Proj>
  static void fillPolygon(cv::Mat& mask, const geometry_msgs::Polygon& poly, Proj worldToWin, unsigned char value) {
    if (poly.points.size() < 3) return;
    std::vector<cv::Point> pts;
    pts.reserve(poly.points.size());
    for (const auto& p : poly.points) pts.push_back(worldToWin(p.x, p.y));
    std::vector<std::vector<cv::Point>> polys{pts};
    cv::fillPoly(mask, polys, cv::Scalar(value));
  }

  // Major-axis orientation (radians, map frame) of a binary blob via PCA. Window column index maps
  // to +x and row index to +y (both scaled by res_), so the cell-space angle equals the world angle.
  static double majorAxisAngle(const cv::Mat& mask) {
    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);
    if (pts.size() < 2) return 0.0;
    cv::Mat data(static_cast<int>(pts.size()), 2, CV_32F);
    for (size_t i = 0; i < pts.size(); ++i) {
      data.at<float>(static_cast<int>(i), 0) = static_cast<float>(pts[i].x);
      data.at<float>(static_cast<int>(i), 1) = static_cast<float>(pts[i].y);
    }
    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    const float vx = pca.eigenvectors.at<float>(0, 0);
    const float vy = pca.eigenvectors.at<float>(0, 1);
    return std::atan2(vy, vx);
  }

  // --- ROS plumbing ---
  ros::Subscriber pose_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceServer fill_service_;
  ros::ServiceClient plan_client_;
  ros::Publisher grid_pub_;
  ros::Timer grid_timer_;

  // --- params ---
  double res_ = 0.05;
  double tool_width_ = 0.14;
  double min_gap_width_ = 0.14;
  double min_gap_area_ = 0.04;
  double residual_stop_area_ = 0.05;
  double erosion_margin_ = 0.07;
  double max_connect_gap_ = 0.5;
  bool publish_grid_ = true;
  std::string map_frame_ = "map";
  std::string pose_topic_;
  std::string status_topic_;
  std::string state_topic_;

  // --- coverage grid state (map frame; cell (0,0) centre at origin_) ---
  std::mutex mutex_;
  cv::Mat grid_;
  bool initialized_ = false;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  bool mowing_active_ = false;
  bool have_prev_ = false;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  std::string last_job_id_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_feedback");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  CoverageFeedback node(nh, private_nh);
  ROS_INFO_STREAM("coverage_feedback: started.");
  ros::spin();
  return 0;
}
