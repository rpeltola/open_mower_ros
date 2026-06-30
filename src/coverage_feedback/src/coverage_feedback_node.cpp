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
//      accuracy" - see ticket), stamps the swept disc of radius tool_width/2 along the path into a
//      sparse set of covered world-cells. Instead of republishing a full grid, the node streams
//      DELTAS (only the cells newly covered since the last tick) on coverage_feedback/coverage_delta,
//      so transport cost is O(robot movement), never O(lawn). A snapshot service hands a new client
//      the full covered set on demand. See docs/map-layers-architecture.md.
//
//   B. Gap detect + refill (on demand, via the GetFillPaths service). Rasterizes the area polygon
//      minus its obstacles, erodes it by erosion_margin (so the outline/keepout rim is excluded),
//      subtracts the covered cells (read out of the sparse set into a local window), declutters the
//      difference (morphological opening + connected components with min width/area thresholds), and
//      for each surviving gap builds a polygon and requests a linear fill from slic3r along the gap's
//      major axis (PCA). The aggregated fill paths are returned so MowingBehavior can run them before
//      docking.

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/Status.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <slic3r_coverage_planner/PlanPath.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
#include <xbot_msgs/AbsolutePose.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <vector>

#include "coverage_feedback/CoverageStatus.h"
#include "coverage_feedback/GetFillPaths.h"

namespace {

using json = nlohmann::ordered_json;

// Marker value used while rasterizing the gap-detect window (CV_8UC1).
constexpr unsigned char CELL_COVERED = 255;

// Sparse-tile wire contract (must match the app exactly, see docs/map-layers-architecture.md):
//   cell world-index  cwx = floor(x/res), cwy = floor(y/res)
//   tile (128 cells)  tx = floordiv(cwx,128), ty = floordiv(cwy,128)
//   local index       idx = (cwy - ty*128)*128 + (cwx - tx*128)   in [0, 128*128)
constexpr int TILE = 128;

// Pack a world-cell (cwx, cwy) into a single 64-bit key: high 32 bits = cwx, low 32 bits = cwy.
// The two ranges are disjoint, so the map is a bijection and the inverse is exact.
inline int64_t cellKey(int cwx, int cwy) {
  return (static_cast<int64_t>(cwx) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(cwy));
}
inline void cellUnpack(int64_t key, int& cwx, int& cwy) {
  cwx = static_cast<int32_t>(key >> 32);
  cwy = static_cast<int32_t>(static_cast<uint32_t>(key & 0xFFFFFFFFu));
}
// Floored integer division (cwx may be negative); b is the positive tile size.
inline int floorDiv(int a, int b) {
  int q = a / b, r = a % b;
  if (r != 0 && (r < 0) != (b < 0)) --q;
  return q;
}

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
    private_nh.param("publish_delta", publish_delta_, true);

    plan_client_ = nh.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");

    pose_sub_ = nh.subscribe(pose_topic_, 50, &CoverageFeedback::onPose, this);
    status_sub_ = nh.subscribe(status_topic_, 10, &CoverageFeedback::onStatus, this);
    // Reset the coverage grid when a new mowing job starts so stale coverage from a previous job
    // (e.g. yesterday's mow, with the node still running) is not mistaken for freshly cut ground.
    state_sub_ = nh.subscribe(state_topic_, 10, &CoverageFeedback::onState, this);
    // Latest slic3r mow plan (latched), kept so pass 0's planned_path.json mirrors what was actually mowed.
    planned_path_sub_ = nh.subscribe("mower_logic/planned_path", 1, &CoverageFeedback::onPlannedPath, this);
    fill_service_ = private_nh.advertiseService("get_fill_paths", &CoverageFeedback::onGetFillPaths, this);

    // Live coverage as deltas (newly-covered cells only) on a 1 Hz timer, plus a snapshot service that
    // hands a new client the full covered set. The timer also drives throttled disk persistence.
    delta_pub_ = private_nh.advertise<std_msgs::String>("coverage_delta", 4, false);
    snapshot_srv_ = private_nh.advertiseService("GetCoverageSnapshot", &CoverageFeedback::onGetSnapshot, this);
    delta_timer_ = nh.createTimer(ros::Duration(1.0), &CoverageFeedback::publishDelta, this);

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
    if (left_autonomous && !covered_.empty() && !last_job_id_.empty()) {
      ROS_INFO_STREAM("coverage_feedback: job '" << last_job_id_ << "' ended - persisting final coverage (pass "
                                                 << pass_index_ << ").");
      persistPass({}, {}, finalStatus());
    }
    // A new, non-empty job id means we switched jobs. Because this node has respawn=true, a crash or
    // charge-reboot mid-mow restarts it with empty in-RAM state; on the next state message it sees the
    // current (unchanged) job id as "new" and would wipe coverage that was already cut. To stay correct
    // we persist the covered set to disk per job and, on a job-id change, prefer resuming from disk over
    // reset. Resuming the same job after a dock keeps the same id, so nothing changes there either.
    if (!msg->job_id.empty() && msg->job_id != last_job_id_) {
      last_job_id_ = msg->job_id;
      // The per-pass track buffer and the cached pass-0 plan belong to the previous job: drop them so
      // the new job records its own. The new job's plan re-publishes when slic3r planning runs.
      track_.clear();
      planned_path_plan_ = json::object();
      have_planned_path_ = false;
      resetCoverage();
      // Tell the app to drop the previous job's overlay immediately; a resumed job then re-fetches its
      // covered set via the snapshot service, so the cleared overlay is repopulated right away.
      publishReset();
      if (loadLatest(msg->job_id)) {
        ROS_INFO_STREAM("coverage_feedback: resuming job '" << msg->job_id << "' from disk (pass " << pass_index_
                                                            << ", " << covered_.size() << " covered cells).");
      } else {
        ROS_INFO_STREAM("coverage_feedback: new job '" << msg->job_id << "' - starting fresh coverage.");
      }
      return;
    }
    last_job_id_ = msg->job_id;
  }

  // Drop all live coverage state (new job / explicit reset). Persistence on disk is untouched.
  void resetCoverage() {
    covered_.clear();
    delta_keys_.clear();
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

  // Stamp the swept swath ending at (x, y): the capsule from the previous sample to here with radius
  // tool_width/2. We step along the segment at ~res spacing and rasterize the disc at each step, so a
  // sparse pose stream still paints a continuous strip. Every cell newly added to covered_ is also
  // pushed to delta_keys_ so the next 1 Hz tick streams it.
  void stamp(double x, double y) {
    const double radius = tool_width_ / 2.0;
    double sx = x, sy = y;  // segment start (defaults to a point stamp at the current pose)
    if (have_prev_ && std::hypot(x - prev_x_, y - prev_y_) <= max_connect_gap_) {
      sx = prev_x_;
      sy = prev_y_;
    }
    const double seg_len = std::hypot(x - sx, y - sy);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / res_)) + 1);
    for (int s = 0; s < steps; ++s) {
      const double t = steps > 1 ? static_cast<double>(s) / (steps - 1) : 0.0;
      stampDisc(sx + (x - sx) * t, sy + (y - sy) * t, radius);
    }
    prev_x_ = x;
    prev_y_ = y;
    have_prev_ = true;
  }

  // Rasterize the disc of the given radius centred at world (cx, cy) into covered_, recording newly
  // covered cells in delta_keys_. A cell is covered when its centre lies within radius of (cx, cy);
  // the centre cell is always included so a sub-cell tool radius still leaves a mark.
  void stampDisc(double cx, double cy, double radius) {
    const int ccx = static_cast<int>(std::floor(cx / res_));
    const int ccy = static_cast<int>(std::floor(cy / res_));
    const int rcells = std::max(0, static_cast<int>(std::ceil(radius / res_)));
    const double r2 = radius * radius;
    for (int dy = -rcells; dy <= rcells; ++dy) {
      for (int dx = -rcells; dx <= rcells; ++dx) {
        const int cwx = ccx + dx;
        const int cwy = ccy + dy;
        const double wx = (cwx + 0.5) * res_;  // cell centre in world coords
        const double wy = (cwy + 0.5) * res_;
        if ((dx != 0 || dy != 0) && (wx - cx) * (wx - cx) + (wy - cy) * (wy - cy) > r2) continue;
        const int64_t key = cellKey(cwx, cwy);
        if (covered_.insert(key).second) delta_keys_.push_back(key);
      }
    }
  }

  // Group a list of world-cell keys by tile into the wire-format "tiles" array:
  //   [ {"tx":T,"ty":T,"cells":[idx,...]}, ... ]   idx = (cwy-ty*128)*128 + (cwx-tx*128)
  template <typename Range>
  json tilesJson(const Range& keys) const {
    std::map<std::pair<int, int>, std::vector<int>> by_tile;
    for (const int64_t key : keys) {
      int cwx, cwy;
      cellUnpack(key, cwx, cwy);
      const int tx = floorDiv(cwx, TILE);
      const int ty = floorDiv(cwy, TILE);
      const int lx = cwx - tx * TILE;
      const int ly = cwy - ty * TILE;
      by_tile[{tx, ty}].push_back(ly * TILE + lx);
    }
    json tiles = json::array();
    for (auto& [txy, cells] : by_tile) {
      tiles.push_back({{"tx", txy.first}, {"ty", txy.second}, {"cells", cells}});
    }
    return tiles;
  }

  // Full state of covered_ as one JSON string (same shape as a delta, but every covered cell and no
  // "reset" field). Used by the snapshot service and the per-job/per-pass persistence. Hold mutex_.
  json snapshotJson() const {
    json j;
    j["job_id"] = last_job_id_;
    j["res"] = res_;
    j["tile"] = TILE;
    j["tiles"] = tilesJson(covered_);
    return j;
  }

  // Wrap a JSON document in a std_msgs/String for publishing on coverage_delta.
  static std_msgs::String toStringMsg(const json& j) {
    std_msgs::String msg;
    msg.data = j.dump();
    return msg;
  }

  // Publish a one-shot reset delta so the app drops the previous job's overlay. Hold mutex_.
  void publishReset() {
    json j;
    j["job_id"] = last_job_id_;
    j["reset"] = true;
    j["tiles"] = json::array();
    delta_pub_.publish(toStringMsg(j));
  }

  // 1 Hz: stream the cells covered since the last tick (idle-gated: nothing new -> nothing published),
  // then periodically refresh the on-disk snapshot so a respawn resumes with near-current coverage.
  void publishDelta(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!delta_keys_.empty()) {
      if (publish_delta_) {
        json j;
        j["job_id"] = last_job_id_;
        j["res"] = res_;
        j["tile"] = TILE;
        j["reset"] = false;
        j["tiles"] = tilesJson(delta_keys_);
        delta_pub_.publish(toStringMsg(j));
      }
      delta_keys_.clear();
    }

    // Throttle disk saves to avoid churn; only meaningful once we actually have coverage.
    constexpr int kSaveEverySeconds = 10;
    if (!covered_.empty() && ++save_tick_ >= kSaveEverySeconds) {
      save_tick_ = 0;
      saveLatest();
    }
  }

  // Snapshot service (std_srvs/Trigger): the full covered set as JSON in the response `message`. Using
  // Trigger keeps this dependency-light - no custom .srv, and xbot_monitoring only needs std_srvs.
  bool onGetSnapshot(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    res.success = true;
    res.message = snapshotJson().dump();
    return true;
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

  // Repopulate covered_ from a snapshot JSON document ({tiles:[{tx,ty,cells:[idx,...]}]}). Returns the
  // number of cells inserted. Tolerant of missing/garbled fields (best-effort resume).
  size_t loadSnapshot(const json& snap) {
    size_t added = 0;
    if (!snap.is_object() || !snap.contains("tiles") || !snap["tiles"].is_array()) return 0;
    for (const auto& tile : snap["tiles"]) {
      if (!tile.is_object() || !tile.contains("tx") || !tile.contains("ty") || !tile.contains("cells")) continue;
      const int tx = tile["tx"].get<int>();
      const int ty = tile["ty"].get<int>();
      for (const auto& cell : tile["cells"]) {
        const int idx = cell.get<int>();
        if (idx < 0 || idx >= TILE * TILE) continue;
        const int lx = idx % TILE;
        const int ly = idx / TILE;
        if (covered_.insert(cellKey(tx * TILE + lx, ty * TILE + ly)).second) ++added;
      }
    }
    return added;
  }

  // Atomically (write-to-temp + rename) refresh <job>/latest.json.gz from covered_. The snapshot JSON
  // string is stored inside an OpenCV FileStorage (already a workspace dep) so it is gzipped for free.
  // Must hold mutex_.
  void saveLatest() {
    if (covered_.empty() || last_job_id_.empty()) return;
    const std::string dir = jobDir(last_job_id_);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot create job dir '" << dir << "': " << ec.message());
      return;
    }
    const std::string final_path = dir + "/latest.json.gz";
    const std::string tmp_path = dir + "/latest.tmp.json.gz";  // keep .json.gz so OpenCV gzips it
    try {
      cv::FileStorage fs(tmp_path, cv::FileStorage::WRITE);
      if (!fs.isOpened()) {
        ROS_WARN_STREAM("coverage_feedback: cannot open '" << tmp_path << "' for writing.");
        return;
      }
      fs << "res" << res_;
      fs << "pass_index" << pass_index_;
      fs << "snapshot" << snapshotJson().dump();
      fs.release();
    } catch (const cv::Exception& e) {
      ROS_WARN_STREAM("coverage_feedback: failed to write latest snapshot: " << e.what());
      return;
    }
    std::filesystem::rename(tmp_path, final_path, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot finalize '" << final_path << "': " << ec.message());
    }
  }

  // Restore covered_ + pass counter from <job>/latest.json.gz. Returns false (leaving state untouched)
  // if there is no usable saved data. Must hold mutex_.
  bool loadLatest(const std::string& job_id) {
    const std::string path = jobDir(job_id) + "/latest.json.gz";
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) return false;
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      if (!fs.isOpened()) return false;
      std::string snap_str;
      fs["snapshot"] >> snap_str;
      double rr = res_;
      int pass = 0;
      fs["res"] >> rr;
      fs["pass_index"] >> pass;
      fs.release();
      if (snap_str.empty()) {
        ROS_WARN_STREAM("coverage_feedback: '" << path << "' has no usable snapshot - starting fresh.");
        return false;
      }
      const json snap = json::parse(snap_str, nullptr, /*allow_exceptions=*/false);
      if (rr > 0.0) res_ = rr;  // keep coverage self-consistent with the resolution it was built at
      pass_index_ = pass;
      const size_t n = loadSnapshot(snap);
      have_prev_ = false;
      return n > 0;
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

  // Per-pass coverage snapshot for the history view: the same sparse-tile shape the snapshot service
  // and live deltas use ({job_id, res, tile, tiles:[{tx,ty,cells:[idx,...]}]}). Must hold mutex_.
  json coverageJson() const {
    return snapshotJson();
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

  // Lightweight CoverageStatus for an end-of-job snapshot. The full gap analysis (and the area mask it
  // needs) only exists inside get_fill_paths; here we just report the exact covered area from the
  // sparse set. Target/percent need the area polygon, which we don't have at job end, so they stay 0.
  // Hold mutex_.
  coverage_feedback::CoverageStatus finalStatus() const {
    coverage_feedback::CoverageStatus s;
    s.covered_area = static_cast<double>(covered_.size()) * (res_ * res_);
    s.target_area = 0.0;
    s.uncovered_area = 0.0;
    s.coverage_percent = 0.0;
    s.gap_count = 0;
    return s;
  }

  // Write an immutable snapshot of the just-completed pass into <job>/pass<N>/ (coverage + meta + JSON),
  // then advance the pass counter and refresh latest.json.gz so a resume continues at the next pass.
  // Must hold mutex_.
  void persistPass(const std::vector<GapViz>& gaps, const std::vector<slic3r_coverage_planner::Path>& paths,
                   const coverage_feedback::CoverageStatus& status) {
    if (covered_.empty() || last_job_id_.empty()) return;
    const std::string dir = jobDir(last_job_id_) + "/pass" + std::to_string(pass_index_);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      ROS_WARN_STREAM("coverage_feedback: cannot create pass dir '" << dir << "': " << ec.message());
    } else {
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

    if (covered_.empty()) {
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

    // --- window aligned to the global world-cell grid (cwx = floor(x/res)) so the covered set maps in
    //     by integer offset; the window's bottom-left cell is (cwx0, cwy0) ---
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

    const int cwx0 = static_cast<int>(std::floor(min_x / res_));
    const int cwy0 = static_cast<int>(std::floor(min_y / res_));
    const int cwx1 = static_cast<int>(std::floor(max_x / res_));
    const int cwy1 = static_cast<int>(std::floor(max_y / res_));
    const int cols = cwx1 - cwx0 + 1;
    const int rows = cwy1 - cwy0 + 1;
    if (cols <= 0 || rows <= 0 || static_cast<long>(cols) * rows > 50'000'000L) {
      ROS_WARN_STREAM("coverage_feedback: unreasonable gap window " << cols << "x" << rows << ", skipping refill.");
      clearGaps(hdr);
      emitStatus(false);
      return true;
    }

    // Window column/row -> world-cell index: c = floor(x/res) - cwx0. Cell (c,r) covers world cell
    // (cwx0+c, cwy0+r); its centre in world coords is ((cwx0+c)+0.5)*res, ((cwy0+r)+0.5)*res.
    auto worldToWin = [&](double x, double y) {
      return cv::Point(static_cast<int>(std::floor(x / res_)) - cwx0,
                       static_cast<int>(std::floor(y / res_)) - cwy0);
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

    // --- covered window: stamp the cells of covered_ that fall inside the window (O(covered_)) ---
    cv::Mat covered = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (const int64_t key : covered_) {
      int cwx, cwy;
      cellUnpack(key, cwx, cwy);
      const int c = cwx - cwx0;
      const int r = cwy - cwy0;
      if (c >= 0 && c < cols && r >= 0 && r < rows) covered.at<unsigned char>(r, c) = CELL_COVERED;
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
        wp.x = static_cast<float>((cwx0 + cp.x + 0.5) * res_);
        wp.y = static_cast<float>((cwy0 + cp.y + 0.5) * res_);
        wp.z = 0.0f;
        poly.points.push_back(wp);
      }

      GapViz gv;
      gv.polygon = poly;
      gv.angle = angle;
      gv.area_m2 = stats.at<int>(label, cv::CC_STAT_AREA) * (res_ * res_);
      gv.cx = (cwx0 + centroids.at<double>(label, 0) + 0.5) * res_;
      gv.cy = (cwy0 + centroids.at<double>(label, 1) + 0.5) * res_;
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
  ros::ServiceServer snapshot_srv_;
  ros::ServiceClient plan_client_;
  ros::Publisher delta_pub_;
  ros::Publisher status_pub_;
  ros::Publisher gaps_pub_;
  ros::Publisher fill_paths_pub_;
  ros::Timer delta_timer_;

  // --- params ---
  double res_ = 0.05;
  double tool_width_ = 0.14;
  double min_gap_width_ = 0.14;
  double min_gap_area_ = 0.04;
  double residual_stop_area_ = 0.05;
  double erosion_margin_ = 0.07;
  double max_connect_gap_ = 0.5;
  bool publish_delta_ = true;
  std::string map_frame_ = "map";
  std::string pose_topic_;
  std::string status_topic_;
  std::string state_topic_;

  // --- coverage state: sparse set of covered world-cells (cwx = floor(x/res), cwy = floor(y/res)),
  //     keyed by cellKey(cwx, cwy). delta_keys_ collects cells newly covered since the last 1 Hz tick. ---
  std::mutex mutex_;
  std::unordered_set<int64_t> covered_;
  std::vector<int64_t> delta_keys_;
  double kMaxJump_ = 2.0;  // metres; reject EKF/GPS teleports

  // --- disk persistence ---
  int pass_index_ = 0;      // per-job pass counter (pass<N> snapshot dir); restored on resume
  int save_tick_ = 0;       // delta-timer ticks since the last periodic saveLatest()
  uint8_t prev_state_ = 0;  // last HighLevelStatus.state (NULL); detects AUTONOMOUS->IDLE = job end

  bool mowing_active_ = false;
  bool have_prev_ = false;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  std::string last_job_id_;

  // --- per-pass replay capture (written into pass<N>/ alongside the coverage snapshot) ---
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
