#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

using json = nlohmann::ordered_json;

/*
 * Per-job planned-path history, mirroring PositionHistory's persistence model for the actual track.
 *
 * mower_logic publishes the slic3r plan once per mowing area, as {job_id, area_id, paths:[...]}.
 * This class accumulates all areas of a job into one whole-job plan (so the live overlay and the
 * saved history both show the COMPLETE planned path, exactly like the actual driven track), keyed
 * by job_id. A job_id change rotates to a new file.
 *
 * Persistence (one JSON file per job, in planned_path/):
 *   Filename: <unix_epoch_seconds>_<job_id>.json
 *   Content : {"job_id":"...","areas":[{"area_id":"...","paths":[{"is_outline":bool,"points":[[x,y]]}]}]}
 * The live MQTT layer and the history RPC expose the FLATTENED form {"job_id","paths":[...]}.
 */
class PlannedPathHistory {
 public:
  void init() {
    base_dir_ = "planned_path";
    std::filesystem::create_directories(base_dir_);
    dir_index_ = scanDir();
    // Load the latest job into memory so the live overlay can be re-published after a restart mid-job.
    if (!dir_index_.empty() && loadFile(dir_index_.front().path, current_job_id_, areas_)) {
      file_path_ = dir_index_.front().path.string();
    } else {
      current_job_id_.clear();
      areas_.clear();
    }
  }

  // Feed one area's published plan. Accumulates into the current job; rotates the file on job_id change.
  void addPlan(const json& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!msg.is_object() || !msg.contains("job_id") || !msg["job_id"].is_string()) return;
    const std::string job_id = msg["job_id"].get<std::string>();
    if (job_id.empty()) return;
    const std::string area_id = msg.value("area_id", std::string{});
    json paths = msg.contains("paths") ? msg["paths"] : json::array();

    if (job_id != current_job_id_) {
      // New job: rotate to a fresh accumulator + file.
      current_job_id_ = job_id;
      areas_.clear();
      const int64_t epoch = epochNow();
      file_path_ = base_dir_ + "/" + std::to_string(epoch) + "_" + job_id + ".json";
      dir_index_.insert(dir_index_.begin(), {epoch, job_id, file_path_});
    }
    // Replace this area's paths if it has been published before (e.g. a resume re-slice), else append.
    // An empty area_id can't be matched (every empty id would alias to one slot and collapse a
    // multi-area job to its last area), so always append those as distinct areas.
    auto it = area_id.empty()
                  ? areas_.end()
                  : std::find_if(areas_.begin(), areas_.end(), [&](const AreaPlan& a) { return a.area_id == area_id; });
    if (it != areas_.end()) {
      it->paths = std::move(paths);
    } else {
      areas_.push_back({area_id, std::move(paths)});
    }
    save();
  }

  bool hasCurrent() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return !current_job_id_.empty() && !areas_.empty();
  }

  // Whole-job flattened plan of the current job, for the live overlay: {"job_id","paths":[...]}.
  json getCurrent() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return flatten(current_job_id_, areas_);
  }

  // Returns a job's whole-job plan (current job from memory, past jobs from disk).
  // Returns {"error": "not found"} if no such job exists.
  json getHistory(const std::string& job_id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!current_job_id_.empty() && job_id == current_job_id_) return flatten(current_job_id_, areas_);
    for (const auto& e : dir_index_) {
      if (e.job_id != job_id) continue;
      std::string jid;
      std::vector<AreaPlan> areas;
      if (loadFile(e.path, jid, areas)) return flatten(jid, areas);
      break;
    }
    return {{"error", "not found"}};
  }

  // No job_id: the current job, else the most-recent saved job, else an empty plan.
  json getHistory() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!current_job_id_.empty()) return flatten(current_job_id_, areas_);
    if (!dir_index_.empty()) {
      std::string jid;
      std::vector<AreaPlan> areas;
      if (loadFile(dir_index_.front().path, jid, areas)) return flatten(jid, areas);
    }
    return {{"job_id", ""}, {"paths", json::array()}};
  }

  // Array of {job_id, timestamp} sorted newest-first.
  json listHistories() const {
    std::lock_guard<std::mutex> lk(mutex_);
    json arr = json::array();
    for (const auto& e : dir_index_) arr.push_back({{"job_id", e.job_id}, {"timestamp", e.epoch}});
    return arr;
  }

  // Deletes the file for job_id (or ALL files when job_id is empty). Never deletes the active job.
  json deleteHistory(const std::optional<std::string>& job_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    int deleted = 0;
    for (auto it = dir_index_.begin(); it != dir_index_.end();) {
      if (job_id.has_value() && it->job_id != *job_id) {
        ++it;
        continue;
      }
      if (it->job_id == current_job_id_) {
        ++it;
        continue;
      }
      std::error_code ec;
      if (std::filesystem::remove(it->path, ec)) {
        ++deleted;
        it = dir_index_.erase(it);
      } else {
        ++it;
      }
    }
    return {{"deleted", deleted}};
  }

 private:
  struct AreaPlan {
    std::string area_id;
    json paths;
  };
  struct FileEntry {
    int64_t epoch;
    std::string job_id;
    std::filesystem::path path;
  };

  static int64_t epochNow() {
    return static_cast<int64_t>(ros::Time::now().toSec());
  }

  // Collapse the per-area accumulator into the flat {"job_id","paths":[...]} form the app consumes.
  static json flatten(const std::string& job_id, const std::vector<AreaPlan>& areas) {
    json out;
    out["job_id"] = job_id;
    json all = json::array();
    for (const auto& a : areas) {
      if (!a.paths.is_array()) continue;
      for (const auto& p : a.paths) all.push_back(p);
    }
    out["paths"] = all;
    return out;
  }

  // Atomically (write-to-temp + rename) persist the current job's accumulated areas.
  void save() const {
    if (current_job_id_.empty() || file_path_.empty()) return;
    json doc;
    doc["job_id"] = current_job_id_;
    json arr = json::array();
    for (const auto& a : areas_) arr.push_back({{"area_id", a.area_id}, {"paths", a.paths}});
    doc["areas"] = arr;

    const std::string tmp = file_path_ + ".tmp";
    {
      std::ofstream os(tmp, std::ios::trunc);
      if (!os) {
        ROS_WARN_STREAM("PlannedPathHistory: cannot open '" << tmp << "' for writing");
        return;
      }
      os << doc.dump();
    }
    std::error_code ec;
    std::filesystem::rename(tmp, file_path_, ec);
    if (ec) ROS_WARN_STREAM("PlannedPathHistory: cannot finalize '" << file_path_ << "': " << ec.message());
  }

  static bool loadFile(const std::filesystem::path& path, std::string& job_id, std::vector<AreaPlan>& areas) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    try {
      json doc;
      f >> doc;
      job_id = doc.value("job_id", std::string{});
      areas.clear();
      if (doc.contains("areas") && doc["areas"].is_array()) {
        for (const auto& a : doc["areas"]) {
          areas.push_back({a.value("area_id", std::string{}), a.contains("paths") ? a["paths"] : json::array()});
        }
      }
      return true;
    } catch (const json::exception&) {
      return false;
    }
  }

  // Parse filenames of the form "<epoch>_<job_id>.json", newest-first.
  std::vector<FileEntry> scanDir() const {
    std::vector<FileEntry> entries;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(base_dir_, ec)) {
      if (!entry.is_regular_file()) continue;
      const std::string name = entry.path().filename().string();
      if (name.size() < 6 || name.substr(name.size() - 5) != ".json") continue;
      const std::string stem = name.substr(0, name.size() - 5);
      const auto sep = stem.find('_');
      if (sep == std::string::npos || sep == 0) continue;
      try {
        const int64_t epoch = std::stoll(stem.substr(0, sep));
        const std::string job_id = stem.substr(sep + 1);
        if (!job_id.empty()) entries.push_back({epoch, job_id, entry.path()});
      } catch (...) {
        continue;
      }
    }
    std::sort(entries.begin(), entries.end(), [](const FileEntry& a, const FileEntry& b) { return a.epoch > b.epoch; });
    return entries;
  }

  std::string base_dir_;
  std::string file_path_;
  std::vector<FileEntry> dir_index_;
  std::string current_job_id_;
  std::vector<AreaPlan> areas_;
  mutable std::mutex mutex_;
};
