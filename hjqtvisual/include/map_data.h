// Copyright(c) 2021-2026 Hongjing
// All rights reserved.
//
// Author: Cheng Wang
// Update: 2021-4-26
#ifndef NOP_MAP_DATA_H_
#define NOP_MAP_DATA_H_

#include <cstdint>
#include "common_type.h"

namespace autodrive {

namespace nop {

/// @brief Max speed limit num
static constexpr std::uint8_t kMaxSpeedLimitNum{5U};
/// @brief Max center line point num
static constexpr std::uint8_t kMaxCenterLinePointNum{250U};
/// @brief Max lane width num
static constexpr std::uint8_t kMaxLaneWidthNum{5U};
/// @brief Max lane num
static constexpr std::uint8_t kMaxLaneNum{4U};
/// @brief Max lane marker change num
static constexpr std::uint8_t kMaxLaneMarkerChangeNum{5U};
/// @brief Max attribute num
static constexpr std::uint8_t kMaxAttributeNum{10U};

// HdMap status
enum class HdMapStatus : std::uint8_t {
  UNKNOWN = 0,          // Known
  VALID = 1,            // Valid
  INVALID = 2,          // Invalid
  ERROR = 3             // Error
};

// Navigation status
enum class NavigationStatus : std::uint8_t {
  NONE = 0,             // None
  MATCH = 1,            // Match
  NOT_MATCH = 2,        // Not match
  INITIAL = 3           // Initial
};

// Lane change type
enum class LaneChangeType : std::uint8_t {
  LANE_FOLLOW = 0,              // Lane follow
  LEFT_LANE_CHANGE = 1,         // Left lane change
  RIGHT_LANE_CHANGE = 2,        // Right lane change
  LANE_FOLLOW_CROSS_LEFT = 3,   // Lane follow cross left
  LANE_FOLLOW_CROSS_RIGHT = 4   // Lane follow cross right
};

// Lane change reason
enum class LaneChangeReason : std::uint8_t {
  NO_REASON = 0,                // No reason
  LANE_DISAPPEAR = 1,           // Lane disappear
  JCT_ENTRY_AHEAD = 2,          // JCT entry ahead
  ENTER_JCT = 3,                // Enter JCT
  JCT_EXIT_AHEAD = 4,           // JCT exit ahead
  EXIT_JCT = 5,                 // Exit JCT
  ROAD_TYPE_Y = 6,              // Road type Y
  IC_ENTRY_AHEAD = 10,          // IC entry ahead
  ENTER_IC = 11,                // Enter IC
  IC_EXIT_AHEAD = 12,           // IC exit ahead
  EXIT_IC = 13,                 // Exit IC
  OTHERS = 14                   // Others
};

// Lane index
enum class LaneIndex : std::uint8_t {
  LEFT = 0,            // Left
  RIGHT = 1,           // Right
  NEXT_LEFT = 2,       // Next left
  NEXT_RIGHT = 3       // Next right
};


// Key point type
enum class KeyPointType : std::uint8_t {
  UNKNOWN = 0,                          // Unknown
  SPLIT_UNKNOWN = 1,                    // Split unknown
  Y_SPLIT_TO_LEFT = 2,                  // Y split to left
  Y_SPLIT_TO_RIGHT = 3,                 // Y split to right
  SPLIT_CONTINUATION_ON_LEFT = 4,       // Split continuation on left
  SPLIT_CONTINUATION_ON_RIGHT = 5,      // Split continuation on right
  SPLIT_NON_CONTINUATION_ON_LEFT = 6,   // Split non continuation on left
  SPLIT_NON_CONTINUATION_ON_RIGHT = 7,  // Split non continuation on right
  MERGE_UNKNOWN = 8,                    // Merge unknown
  MERGE_FROM_LEFT = 9,                  // Merge from left
  MERGE_FROM_RIGHT = 10,                // Merge from right
  MERGE_CONTINUATION_ON_LEFT = 11,      // Merge continuation on left
  MERGE_CONTINUATION_ON_RIGHT = 12,     // Merge continuation on right
  Y_MERGE_FROM_LEFT = 13,               // Y merge from left
  Y_MERGE_FROM_RIGHT = 14,              // Y merge from right
  ENTRANCE_UNKNOWN = 15,                // Entrance unknown
  ENTRANCE_CONTINUATION_ON_RIGHT = 16,  // Entrance continuation on right
  ENTRANCE_MERGE_FROM_LEFT = 17,        // Entrance merge from left
  ENTRANCE_MERGE_FROM_RIGHT = 18,       // Entrance merge from right
  ENTRANCE_CONTINUATION_ON_LEFT = 19,   // Entrance continuation on left
  ENTRANCE_NON_MERGE_TO_RIGHT = 20,     // Entrance non merge to right
  ENTRANCE_NON_MERGE_ON_RIGHT = 21,     // Entrance non merge on right
  ENTRANCE_NON_MERGE_ON_LEFT = 22,      // Entrance non merge on left
  ENTRANCE_NON_MERGE_TO_LEFT = 23,      // Entrance non merge to left
  EXIT_UNKNOWN = 24,                    // Exit unknown
  EXIT_TO_RIGHT = 25,                   // Exit to right
  EXIT_TO_LEFT = 26,                    // Exit to left
  EXIT_CONTINUATION_ON_LEFT = 27,       // Exit continuation on left
  EXIT_CONTINUATION_ON_RIGHT = 28       // Exit continuation on right
};

// Lane marker change type
enum class LaneMarkerChangeType : std::uint8_t {
  UNKNOWN = 0,                          // Unknown
  NONE = 1,                             // None
  SOLID_LINE = 2,                       // Solid line
  DASHED_LINE = 3,                      // Dashed line
  DOUBLE_SOLID_LINE = 4,                // Double solid line
  DOUBLE_DASHED_LINE = 5,               // Double dashed line
  LEFT_SOLID_RIGHT_DASHED = 6,          // Left solid right dashed
  RIGHT_SOLID_LEFT_DASHED = 7,          // Right solid left dashed
  DASHED_BLOCKS = 8,                    // Dashed blocks
  SHADED_AREA = 9,                      // Shaded area
  PHYSICAL_DIVIDER = 10                 // Physical divider
};

// Inhibit status
struct alignas(2) InhibitStatus {
  /// @brief Inhibit flag
  bool inhibit_flag;
  /// @brief Inhibit reason
  std::uint8_t inhibit_reason;
};

// Map header
struct alignas(8) MapHeader {
  /// @brief Time stamp
  std::uint32_t timestamp_high32;
  std::uint32_t timestamp_low32;
  std::uint32_t udp_index;
  std::uint32_t reserved;
  /// @brief HD map status
  HdMapStatus hd_map_status;
  /// @brief Whether is in defined geofence
  bool is_in_defined_geofence;
  /// @brief Whether is in defined ODD (Operational Designed Domain)
  bool is_in_defined_odd;
  /// @brief Whether need take over
  bool need_take_over;
  /// @brief Left inhibit status for auto lane change
  InhibitStatus left_inhibit_status;
  /// @brief Right inhibit status for auto lane change
  InhibitStatus right_inhibit_status;
  /// @brief Distance to the switch point (cm)
  std::uint16_t key_point_offset;
  /// @brief Navigation speed limit count
  std::uint8_t num_speed_limits;
  /// @brief Navigation lane change type
  LaneChangeType navi_lane_change_type;
  /// @brief Navigation lane change reason
  LaneChangeReason navi_lane_change_reason;
  bool navi_lane_change_cross_lane;
  NavigationStatus navigation_state;
  std::uint8_t padding;
};

// Data pair
struct alignas(8) DataPair {
  /// @brief Value
  float value;
  /// @brief Offset (cm)
  std::uint32_t offset;
};

// Host info
struct alignas(8) HostInfo {
  /// @brief Position (UTM)
  Vector3f position;
  /// @brief Orientation (pitch, yaw, roll)
  Vector3f orientation;
  ///GPS Position
  float latitude;
  float longitude;
  std::uint32_t start_lat;
  std::uint32_t start_lon;
  /// @brief Lane id
  std::uint8_t lane_id;
  /// @brief Lane count with the same direction
  std::uint8_t num_lanes;
  std::uint8_t lane_type;
  std::uint8_t padding;
  /// @brief Offset (cm)
  std::uint32_t offset;
};

// Center line point
struct alignas(4) CenterLinePoint {
  /// @brief Position (relative to start position)
  Vector2f position;
  /// @brief Curvature
  float curvature;
  /// @brief Heading (rad)
  float heading;
  /// @brief Offset (cm)
  std::uint32_t offset;
};

// Key point
struct alignas(8) KeyPoint {
  /// @brief Position
  Vector2f position;
  /// @brief Type
  KeyPointType type;
  std::uint8_t padding[3];
  /// @brief Offset (cm)
  std::uint32_t offset;
};

// Lane marker change
struct alignas(4) LaneMarkerChange {
  /// @brief Type
  LaneMarkerChangeType type;
  std::uint8_t padding[3];
  /// @brief Offset (cm)
  std::uint32_t offset;
};

// Attribute
struct alignas(4) Attribute {
  /// @brief Slope
  float slope;
  /// @brief Banking
  float banking;
  /// @brief Offset (cm)
  std::uint32_t offset;
};

// Lane info
struct alignas(8) LaneInfo {
  /// @brief Center line point count
  std::uint8_t num_center_line_points;
  /// @brief Lane width count
  std::uint8_t num_lane_widths;
  std::uint8_t padding[6];
  /// @brief Center line points
  CenterLinePoint center_line_points[kMaxCenterLinePointNum];
  /// @brief Lane widths
  DataPair lane_widths[kMaxLaneWidthNum];
  /// @brief Key point
  KeyPoint keypoint;
};

// Host lane info
struct alignas(8) HostLaneInfo {
  /// @brief Lane info
  LaneInfo lane_info;
  KeyPoint mergePoint;
  /// @brief Speed limit count
  std::uint8_t num_speed_limits;
  /// @brief Left lane marker change count
  std::uint8_t num_left_lane_marker_changes;
  /// @brief Right lane marker change count
  std::uint8_t num_right_lane_marker_changes;
  /// @brief Attribute count
  std::uint8_t num_attributes;
  std::uint8_t padding[4];
  /// @brief Speed limits
  DataPair speed_limits[kMaxSpeedLimitNum];
  /// @brief Left lane marker changes
  LaneMarkerChange left_lane_marker_changes[kMaxLaneMarkerChangeNum];
  /// @brief Right lane marker changes
  LaneMarkerChange right_lane_marker_changes[kMaxLaneMarkerChangeNum];
  /// @brief Attributes
  Attribute attrites[kMaxAttributeNum];
};

// Sub host lane info
struct alignas(8) SubHostLaneInfo {
  /// @brief Direction
  std::uint8_t direction;
  std::uint8_t padding[7];
  /// @brief Lane info
  LaneInfo lane_info;
};

// Map data
struct alignas(8) MapData {
  /// @brief Map header
  MapHeader map_header;
  /// @brief Host info
  HostInfo host_info;
  /// @brief Navigation speed limits
  DataPair speed_limits[kMaxSpeedLimitNum];
  /// @brief Host lane info
  HostLaneInfo host_lane;
  /// @brief Sub host lane info
  SubHostLaneInfo sub_host_lane;
  /// @brief Lanes (left, right, next left, next right)
  LaneInfo lanes[kMaxLaneNum];
};

}

}

#endif // NOP_MAP_DATA_H_
