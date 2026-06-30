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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/Status.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <slic3r_coverage_planner/PlanPath.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <xbot_msgs/AbsolutePose.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <system_error>
#include <vector>

#include "coverage_feedback/CoverageStatus.h"
#include "coverage_feedback/GetFillPaths.h"

namespace {

using json = nlohmann::ordered_json;

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
    // Latest slic3r mow plan (latched), kept so pass 0's planned_path.json mirrors what was actually mowed.
    planned_path_sub_ = nh.subscribe("mower_logic/planned_path", 1, &CoverageFeedback::onPlannedPath, this);
    fill_service_ = private_nh.advertiseService("get_fill_paths", &CoverageFeedback::onGetFillPaths, this);

    if (publish_grid_) {
      grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("coverage_grid", 1, true);
      grid_timer_ = nh.createTimer(ros::Duration(1.0), &CoverageFeedback::publishGrid, this);
    }

    // Telemetry (latched so `rosbag record -a` always captures the latest state).
    status_pub_ = private_nh.advertise<coverage_feedback::CoverageStatus>("status", 1, true);
    gaps_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("gaps", 1, true);
    fill_paths_pub_ = private_nh.advertise<nav_msgs::Path>("fill_paths", 1, true);

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
    // When the mow ends (high-level state leaves AUTONOMOUS, e.g. docking/idle), persist the current
    // job's FINAL coverage as a pass even if it never reached an area-completion/refill. This is what
    // lands every real mow in coverage history (refill passes are still persisted per get_fill_paths).
    const bool left_autonomous = prev_state_ == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS &&
                                 msg->state != mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
    prev_state_ = msg->state;
    if (left_autonomous && initialized_ && !last_job_id_.empty()) {
      ROS_INFO_STREAM("coverage_feedback: job '" << last_job_id_ << "' ended - persisting final coverage (pass "
                                                 << pass_index_ << ").");
      persistPass({}, {}, finalStatus());
    }
    // A new, non-empty job id means we switched jobs. Because this node has respawn=true, a crash or
    // charge-reboot mid-mow restarts it with empty in-RAM state; on the next state message it sees the
    // current (unchanged) job id as "new" and would wipe coverage that was already cut. To stay correct
    // we persist the grid to disk per job and, on a job-id change, prefer resuming from disk over reset.
    // Resuming the same job after a dock keeps the same id, so nothing changes there either.
    if (!msg->job_id.empty() && msg->job_id != last_job_id_) {
      last_job_id_ = msg->job_id;
      // The per-pass track buffer and the cached pass-0 plan belong to the previous job: drop them so
      // the new job records its own. The new job's plan re-publishes when slic3r planning runs.
      track_.clear();
      planned_path_plan_ = json::object();
      have_planned_path_ = false;
      if (loadLatest(msg->job_id)) {
        ROS_INFO_STREAM("coverage_feedback: resuming job '" << msg->job_id << "' from disk (pass " << pass_index_
                                                            << ", grid " << grid_.cols << "x" << grid_.rows << ").");
      } else {
        ROS_INFO_STREAM("coverage_feedback: new job '" << msg->job_id << "' - starting fresh coverage grid.");
        resetGrid();
        publishEmptyGrid();  // clear the app's retained overlay so the previous job's coverage doesn't linger
      }
      return;
    }
    last_job_id_ = msg->job_id;
  }

  void resetGrid() {
    grid_.release();
    initialized_ = false;
    have_prev_ = false;
    pass_index_ = 0;
  }

  // Cache the latest slic3r mow plan (JSON {job_id, paths:[{is_outline, points}]}) for pass-0 replay.
  void onPlannedPath(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      planned_path_plan_ = json::parse(msg->data);
      have_planned_path_ = true;
    } catch (const json::exception& e) {
      ROS_WARN_STREAM("coverage_feedback: ignoring malformed planned_path: " << e.what());
    }
  }

  void onPose(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    if (!mowing_active_) {  // idle/transit: don't track or stamp (position_history has the full path)
      have_prev_ = false;
      return;
    }
    if (have_prev_ && std::hypot(x - prev_x_, y - prev_y_) > kMaxJump_) {  // GPS/EKF outlier
      ROS_WARN_STREAM_THROTTLE(5.0, "coverage_feedback: rejecting outlier pose jump > " << kMaxJump_ << " m");
      return;
    }
    track_.push_back({x, y, true});
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
    grid_dirty_ = true;
  }

  // Publish a zero-size grid so the MQTT bridge clears the retained coverage layer: on a new job the
  // app drops the previous job's overlay immediately, instead of showing it until new cells are stamped.
  void publishEmptyGrid() {
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = res_;
    msg.info.width = 0;
    msg.info.height = 0;
    msg.info.origin.orientation.w = 1.0;
    grid_pub_.publish(msg);
  }

  void publishGrid(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;
    if (!grid_dirty_) return;
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
    grid_dirty_ = false;

    // Periodically refresh latest.yml.gz so a respawn after a crash resumes with near-current coverage
    // even if no GetFillPaths call happened since the last save. Throttled to avoid gzip churn at 1 Hz.
    constexpr int kSaveEverySeconds = 10;
    if (++save_tick_ >= kSaveEverySeconds) {
      save_tick_ = 0;
      saveLatest();
    }
  }

  // -------------------------------------------------------------------------
  // Disk persistence (per-job grid + per-pass snapshots, rooted at ROS_HOME)
  // -------------------------------------------------------------------------
  // <ROS_HOME or $HOME/.ros>/coverage
  std::string coverageRoot() const {
    const char* ros_home = std::getenv("ROS_HOME");
    if (ros_home && ros_home[0] != '\0') return std::string(ros_home) + "/coverage";
    const char* home = std::getenv("HOME");
    const std::string base = (home && home[0] != '\0') ? std::string(home) : std::string(".");
    return base + "/.ros/coverage";
  }

  std::string jobDir(const std::string& job_id) const {
    return coverageRoot() + "/" + job_id;
  }

  // Serialize the live grid + georeferencing + pass counter via cv::FileStorage. Must hold mutex_.
  static void writeGrid(cv::FileStorage& fs, const cv::Mat& grid, double origin_x, double origin_y, double res,
                        int pass_index) {
    fs << "grid" << grid;
    fs << "origin_x" << origin_x;
    fs << "origin_y" << origin_y;
    fs << "res" << res;
    fs << "pass_index" << pass_index;
  }

  // Atomically (write-to-temp + rename) refresh <job>/latest.yml.gz from the current grid. Must hold mutex_.
  void saveLatest() {
    if (!initialized_ || last_job_id_.empty()) return;
    const std::string dir = jobDir(last_job_id_);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot create job dir '" << dir << "': " << ec.message());
      return;
    }
    const std::string final_path = dir + "/latest.yml.gz";
    const std::string tmp_path = dir + "/latest.tmp.yml.gz";  // keep .yml.gz so OpenCV gzips it
    try {
      cv::FileStorage fs(tmp_path, cv::FileStorage::WRITE);
      if (!fs.isOpened()) {
        ROS_WARN_STREAM("coverage_feedback: cannot open '" << tmp_path << "' for writing.");
        return;
      }
      writeGrid(fs, grid_, origin_x_, origin_y_, res_, pass_index_);
      fs.release();
    } catch (const cv::Exception& e) {
      ROS_WARN_STREAM("coverage_feedback: failed to write latest grid: " << e.what());
      return;
    }
    std::filesystem::rename(tmp_path, final_path, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot finalize '" << final_path << "': " << ec.message());
    }
  }

  // Restore grid_ + georeferencing + pass counter from <job>/latest.yml.gz. Returns false (leaving state
  // untouched) if there is no usable saved data. Must hold mutex_.
  bool loadLatest(const std::string& job_id) {
    const std::string path = jobDir(job_id) + "/latest.yml.gz";
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) return false;
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      if (!fs.isOpened()) return false;
      cv::Mat g;
      fs["grid"] >> g;
      if (g.empty() || g.type() != CV_8UC1) {
        ROS_WARN_STREAM("coverage_feedback: '" << path << "' has no usable grid - starting fresh.");
        return false;
      }
      double ox = origin_x_, oy = origin_y_, rr = res_;
      int pass = 0;
      fs["origin_x"] >> ox;
      fs["origin_y"] >> oy;
      fs["res"] >> rr;
      fs["pass_index"] >> pass;
      fs.release();
      grid_ = g;
      origin_x_ = ox;
      origin_y_ = oy;
      if (rr > 0.0) res_ = rr;  // keep the grid self-consistent with the resolution it was built at
      pass_index_ = pass;
      initialized_ = true;
      have_prev_ = false;
      return true;
    } catch (const cv::Exception& e) {
      ROS_WARN_STREAM("coverage_feedback: failed to load '" << path << "': " << e.what());
      return false;
    }
  }

  // -------------------------------------------------------------------------
  // Telemetry (gap visualization + fill paths), captured by `rosbag record -a`
  // -------------------------------------------------------------------------
  struct GapViz {
    geometry_msgs::Polygon polygon;  // gap outline (dilated), map frame
    double angle = 0.0;              // PCA major axis / fill direction (rad)
    double area_m2 = 0.0;            // detected gap area
    double cx = 0.0;                 // centroid (map frame)
    double cy = 0.0;
  };

  // Drop any previously published gap markers and fill path (gaps were closed or none found).
  void clearGaps(const std_msgs::Header& hdr) {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header = hdr;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    gaps_pub_.publish(arr);
    nav_msgs::Path empty;
    empty.header = hdr;
    fill_paths_pub_.publish(empty);
  }

  // Publish, per detected gap: a red outline, a cyan fill-direction arrow, and an area label.
  void publishGaps(const std::vector<GapViz>& gaps, const std_msgs::Header& hdr) {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header = hdr;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    for (const auto& g : gaps) {
      visualization_msgs::Marker outline;
      outline.header = hdr;
      outline.ns = "gap_outline";
      outline.id = id;
      outline.type = visualization_msgs::Marker::LINE_STRIP;
      outline.action = visualization_msgs::Marker::ADD;
      outline.scale.x = 0.03;
      outline.color.r = 1.0;
      outline.color.a = 1.0;
      outline.pose.orientation.w = 1.0;
      for (const auto& p : g.polygon.points) {
        geometry_msgs::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.05;
        outline.points.push_back(pt);
      }
      if (!g.polygon.points.empty()) {
        geometry_msgs::Point pt;
        pt.x = g.polygon.points.front().x;
        pt.y = g.polygon.points.front().y;
        pt.z = 0.05;
        outline.points.push_back(pt);  // close the loop
      }
      arr.markers.push_back(outline);

      visualization_msgs::Marker arrow;
      arrow.header = hdr;
      arrow.ns = "gap_direction";
      arrow.id = id;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.scale.x = 0.03;
      arrow.scale.y = 0.07;
      arrow.color.g = 1.0;
      arrow.color.b = 1.0;
      arrow.color.a = 1.0;
      arrow.pose.orientation.w = 1.0;
      const double half = 0.25;
      geometry_msgs::Point a;
      geometry_msgs::Point b;
      a.x = g.cx - half * std::cos(g.angle);
      a.y = g.cy - half * std::sin(g.angle);
      a.z = 0.05;
      b.x = g.cx + half * std::cos(g.angle);
      b.y = g.cy + half * std::sin(g.angle);
      b.z = 0.05;
      arrow.points.push_back(a);
      arrow.points.push_back(b);
      arr.markers.push_back(arrow);

      visualization_msgs::Marker text;
      text.header = hdr;
      text.ns = "gap_label";
      text.id = id;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = g.cx;
      text.pose.position.y = g.cy;
      text.pose.position.z = 0.2;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.12;
      text.color.r = text.color.g = text.color.b = 1.0;
      text.color.a = 1.0;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.2f m^2", g.area_m2);
      text.text = buf;
      arr.markers.push_back(text);

      id++;
    }
    gaps_pub_.publish(arr);
  }

  // Publish the queued re-mow geometry as a single Path so the re-mowed strips are visible.
  void publishFillPaths(const std::vector<slic3r_coverage_planner::Path>& paths, const std_msgs::Header& hdr) {
    nav_msgs::Path path;
    path.header = hdr;
    for (const auto& p : paths) {
      for (const auto& ps : p.path.poses) path.poses.push_back(ps);
    }
    fill_paths_pub_.publish(path);
  }

  // Dump the detected gap clusters (map-frame) for a future app timeline view. Best-effort.
  void writeGapsJson(const std::string& path, const std::vector<GapViz>& gaps) const {
    std::ofstream os(path);
    if (!os) return;
    os.precision(9);
    os << "[\n";
    for (size_t i = 0; i < gaps.size(); ++i) {
      const auto& g = gaps[i];
      os << "  {\"angle\": " << g.angle << ", \"area_m2\": " << g.area_m2 << ", \"cx\": " << g.cx
         << ", \"cy\": " << g.cy << ", \"polygon\": [";
      for (size_t j = 0; j < g.polygon.points.size(); ++j) {
        const auto& p = g.polygon.points[j];
        os << "[" << p.x << ", " << p.y << "]";
        if (j + 1 < g.polygon.points.size()) os << ", ";
      }
      os << "]}";
      if (i + 1 < gaps.size()) os << ",";
      os << "\n";
    }
    os << "]\n";
  }

  // Dump the slic3r fill paths (map-frame polylines) for that pass. Best-effort.
  void writeFillPathsJson(const std::string& path, const std::vector<slic3r_coverage_planner::Path>& paths) const {
    std::ofstream os(path);
    if (!os) return;
    os.precision(9);
    os << "[\n";
    for (size_t i = 0; i < paths.size(); ++i) {
      const auto& poses = paths[i].path.poses;
      os << "  [";
      for (size_t j = 0; j < poses.size(); ++j) {
        os << "[" << poses[j].pose.position.x << ", " << poses[j].pose.position.y << "]";
        if (j + 1 < poses.size()) os << ", ";
      }
      os << "]";
      if (i + 1 < paths.size()) os << ",";
      os << "\n";
    }
    os << "]\n";
  }

  // Atomically (write-to-temp + rename) write a JSON document to `path`. Best-effort. Must hold mutex_.
  static void writeJsonAtomic(const std::string& path, const json& j) {
    const std::string tmp = path + ".tmp";
    {
      std::ofstream os(tmp);
      if (!os) return;
      os << j.dump();
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot finalize '" << path << "': " << ec.message());
    }
  }

  // Run-length encode the live grid as the app's coverage layer (the same {res, w, h, ox, oy, rle} scheme
  // xbot_monitoring emits for map_layers/coverage/json: row-major, 100 = covered, 0 = uncovered). The app
  // can render the pass without OpenCV. Must hold mutex_.
  json coverageJson() const {
    json j;
    j["res"] = res_;
    j["w"] = grid_.cols;
    j["h"] = grid_.rows;
    // ox/oy are the real-world pose of the (0,0) cell corner, matching the published OccupancyGrid.
    j["ox"] = origin_x_ - res_ / 2.0;
    j["oy"] = origin_y_ - res_ / 2.0;
    json rle = json::array();
    int cur = 0;
    uint32_t cnt = 0;
    bool first = true;
    for (int r = 0; r < grid_.rows; ++r) {
      const unsigned char* row = grid_.ptr<unsigned char>(r);
      for (int c = 0; c < grid_.cols; ++c) {
        const int v = row[c] ? 100 : 0;
        if (first) {
          cur = v;
          cnt = 1;
          first = false;
        } else if (v == cur) {
          cnt++;
        } else {
          rle.push_back(cur);
          rle.push_back(static_cast<int>(cnt));
          cur = v;
          cnt = 1;
        }
      }
    }
    if (!first) {
      rle.push_back(cur);
      rle.push_back(static_cast<int>(cnt));
    }
    j["rle"] = rle;
    return j;
  }

  // planned_path.json: pass 0 replays the slic3r mow plan; pass N>=1 replays this node's fill paths.
  // Must hold mutex_.
  void writePlannedPathJson(const std::string& path, const std::vector<slic3r_coverage_planner::Path>& paths) const {
    json j;
    j["job_id"] = last_job_id_;
    j["pass"] = pass_index_;
    if (pass_index_ == 0) {
      j["paths"] =
          (have_planned_path_ && planned_path_plan_.contains("paths")) ? planned_path_plan_["paths"] : json::array();
    } else {
      json arr = json::array();
      for (const auto& p : paths) {
        json pts = json::array();
        for (const auto& ps : p.path.poses) pts.push_back({ps.pose.position.x, ps.pose.position.y});
        arr.push_back({{"is_outline", false}, {"points", pts}});
      }
      j["paths"] = arr;
    }
    writeJsonAtomic(path, j);
  }

  // actual_track.json: the accumulated EKF track segmented into runs of constant blade state. Must hold mutex_.
  void writeActualTrackJson(const std::string& path) const {
    json j;
    j["job_id"] = last_job_id_;
    j["pass"] = pass_index_;
    json segments = json::array();
    size_t i = 0;
    while (i < track_.size()) {
      const bool blades = track_[i].blades;
      json pts = json::array();
      size_t k = i;
      for (; k < track_.size() && track_[k].blades == blades; ++k) pts.push_back({track_[k].x, track_[k].y});
      segments.push_back({{"blades", blades}, {"points", pts}});
      i = k;
    }
    j["segments"] = segments;
    writeJsonAtomic(path, j);
  }

  // meta.json: the small per-pass summary (epoch timestamp + stats the node already computed). Must hold mutex_.
  void writeMetaJson(const std::string& path, const coverage_feedback::CoverageStatus& status) const {
    json j;
    j["job_id"] = last_job_id_;
    j["pass"] = pass_index_;
    j["timestamp"] = static_cast<long>(std::time(nullptr));
    j["coverage_percent"] = status.coverage_percent;
    j["gap_count"] = status.gap_count;
    writeJsonAtomic(path, j);
  }

  // Lightweight CoverageStatus for an end-of-job snapshot. The full gap analysis (and the area mask
  // it needs) only exists inside get_fill_paths, so here we report the covered area straight from the
  // grid and an approximate percent over the grid extent. The grid snapshot itself is exact. Hold mutex_.
  coverage_feedback::CoverageStatus finalStatus() const {
    coverage_feedback::CoverageStatus s;
    if (initialized_) {
      const double cell_area = res_ * res_;
      const double covered = static_cast<double>(cv::countNonZero(grid_)) * cell_area;
      const double total = static_cast<double>(grid_.cols) * static_cast<double>(grid_.rows) * cell_area;
      s.covered_area = covered;
      s.target_area = total;
      s.uncovered_area = total > covered ? total - covered : 0.0;
      s.coverage_percent = total > 0.0 ? 100.0 * covered / total : 0.0;
      s.gap_count = 0;
    }
    return s;
  }

  // Write an immutable snapshot of the just-completed pass into <job>/pass<N>/ (grid + meta + JSON),
  // then advance the pass counter and refresh latest.yml.gz so a resume continues at the next pass.
  // Must hold mutex_.
  void persistPass(const std::vector<GapViz>& gaps, const std::vector<slic3r_coverage_planner::Path>& paths,
                   const coverage_feedback::CoverageStatus& status) {
    if (!initialized_ || last_job_id_.empty()) return;
    const std::string dir = jobDir(last_job_id_) + "/pass" + std::to_string(pass_index_);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot create pass dir '" << dir << "': " << ec.message());
    } else {
      try {
        cv::FileStorage fs(dir + "/grid.yml.gz", cv::FileStorage::WRITE);
        if (fs.isOpened()) {
          writeGrid(fs, grid_, origin_x_, origin_y_, res_, pass_index_);
          fs << "covered_area" << status.covered_area;
          fs << "uncovered_area" << status.uncovered_area;
          fs << "gap_count" << status.gap_count;
          fs.release();
        }
      } catch (const cv::Exception& e) {
        ROS_WARN_STREAM("coverage_feedback: failed to write pass grid: " << e.what());
      }
      writeGapsJson(dir + "/gaps.json", gaps);
      writeFillPathsJson(dir + "/fill_paths.json", paths);
      // Per-pass triplet (+ meta) so every mow attempt can be visualized identically to the first.
      writeJsonAtomic(dir + "/coverage.json", coverageJson());
      writePlannedPathJson(dir + "/planned_path.json", paths);
      writeActualTrackJson(dir + "/actual_track.json");
      writeMetaJson(dir + "/meta.json", status);
    }
    track_.clear();  // start the next pass's track fresh, whether or not the snapshot wrote cleanly
    ++pass_index_;   // latest now points at the *next* pass index, so a resume won't overwrite this one
    saveLatest();
  }

  // -------------------------------------------------------------------------
  // Gap detect + refill
  // -------------------------------------------------------------------------
  bool onGetFillPaths(coverage_feedback::GetFillPaths::Request& req, coverage_feedback::GetFillPaths::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    res.uncovered_area = 0.0;
    res.gap_count = 0;

    std_msgs::Header hdr;
    hdr.stamp = ros::Time::now();
    hdr.frame_id = map_frame_;
    coverage_feedback::CoverageStatus status;
    status.header = hdr;
    status.area_id = req.area.id;
    status.area_name = req.area.name;
    auto emitStatus = [&](bool refill) {
      status.refill_requested = refill;
      status_pub_.publish(status);
    };

    const double tool_width = req.tool_width > 0.0 ? req.tool_width : tool_width_;

    if (!initialized_) {
      ROS_WARN_STREAM("coverage_feedback: no coverage accumulated yet - cannot assess gaps, skipping refill.");
      clearGaps(hdr);
      emitStatus(false);
      return true;
    }
    if (req.area.area.points.size() < 3) {
      ROS_WARN_STREAM("coverage_feedback: area polygon has < 3 points, skipping refill.");
      clearGaps(hdr);
      emitStatus(false);
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
      clearGaps(hdr);
      emitStatus(false);
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

    // Coverage stats over the eroded target (telemetry).
    {
      cv::Mat covered_in_target;
      cv::bitwise_and(target, covered, covered_in_target);
      status.target_area = cv::countNonZero(target) * (res_ * res_);
      status.covered_area = cv::countNonZero(covered_in_target) * (res_ * res_);
      status.coverage_percent = status.target_area > 0.0 ? 100.0 * status.covered_area / status.target_area : 100.0;
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
    status.gap_count = static_cast<int>(kept.size());
    status.uncovered_area = total_uncovered;

    if (kept.empty() || total_uncovered < residual_stop_area_) {
      ROS_INFO_STREAM("coverage_feedback: " << kept.size() << " gap(s), " << total_uncovered
                                            << " m^2 uncovered < residual_stop_area (" << residual_stop_area_
                                            << ") - no refill.");
      clearGaps(hdr);
      emitStatus(false);
      persistPass({}, {}, status);
      return true;
    }

    // --- one slic3r linear fill per kept gap, aligned to the gap's major axis ---
    std::vector<GapViz> gap_viz;
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

      GapViz gv;
      gv.polygon = poly;
      gv.angle = angle;
      gv.area_m2 = stats.at<int>(label, cv::CC_STAT_AREA) * (res_ * res_);
      gv.cx = win_origin_x + centroids.at<double>(label, 0) * res_;
      gv.cy = win_origin_y + centroids.at<double>(label, 1) * res_;
      gap_viz.push_back(gv);

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

    publishGaps(gap_viz, hdr);
    publishFillPaths(res.paths, hdr);
    emitStatus(true);
    persistPass(gap_viz, res.paths, status);

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
  ros::Subscriber planned_path_sub_;
  ros::ServiceServer fill_service_;
  ros::ServiceClient plan_client_;
  ros::Publisher grid_pub_;
  ros::Publisher status_pub_;
  ros::Publisher gaps_pub_;
  ros::Publisher fill_paths_pub_;
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
  bool grid_dirty_ = false;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double kMaxJump_ = 2.0;  // metres; reject EKF/GPS teleports

  // --- disk persistence ---
  int pass_index_ = 0;      // per-job pass counter (pass<N> snapshot dir); restored on resume
  int save_tick_ = 0;       // publishGrid timer ticks since the last periodic saveLatest()
  uint8_t prev_state_ = 0;  // last HighLevelStatus.state (NULL); detects AUTONOMOUS->IDLE = job end

  bool mowing_active_ = false;
  bool have_prev_ = false;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  std::string last_job_id_;

  // --- per-pass replay capture (written into pass<N>/ alongside the grid) ---
  struct TrackPoint {
    double x;
    double y;
    bool blades;
  };
  std::vector<TrackPoint> track_;  // EKF track for the in-progress pass; segmented + cleared at each write
  json planned_path_plan_;         // latest mower_logic/planned_path payload (pass-0 plan)
  bool have_planned_path_ = false;
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
