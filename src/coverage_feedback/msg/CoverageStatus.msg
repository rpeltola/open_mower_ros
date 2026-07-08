# Telemetry summary emitted by coverage_feedback every time the gap check runs for an area.
# Published latched on coverage_feedback/status so a `rosbag record -a` captures the full
# decision trail: how much of the area was actually cut, how much was missed, and whether a
# refill was requested.
std_msgs/Header header
string area_id
string area_name
int32 gap_count            # gap clusters kept (>= min_gap_area), i.e. detected misses
float64 target_area        # eroded area-minus-obstacles target (m^2)
float64 covered_area       # of the target, how much the blade actually cut (m^2)
float64 coverage_percent   # 100 * covered_area / target_area
float64 uncovered_area     # total area of the kept gap clusters (m^2)
bool refill_requested      # true if fill paths were returned for re-mowing
