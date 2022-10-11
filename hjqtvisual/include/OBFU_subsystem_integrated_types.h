/**
 * \addtogroup  integrated 
 * \file        OBFU_subsystem_integrated_types.h 
 * \brief       Object Fusion subsystem integrated header file, to include types needed
 * \update      2020/09/11
 **/
#include "common_type.h"

#ifndef OBFU_SUBSYSTEM_INTEGRATED_TYPES_H_
#define OBFU_SUBSYSTEM_INTEGRATED_TYPES_H_


// #include "OBFU_CommonMatrix.h"
// #include "Converters_types.h"
// #include "../../shared_library_code/Std_Types.h"



/***********************************************************/
/// @brief Max bounding box point num
#define kBoundingBoxPointNum_Bus 4U
/// @brief Max slope point num
#define kMaxSlopePointNum_Bus 200U
/// @brief Crosswalk vertex num
#define kCrosswalkVertexNum_Bus 4U
/// @brief Max lane marker num
#define kMaxLaneMarkerNum_Bus 8U
/// @brief Max lane marker num from HRZJ2
#define kMaxLaneMarkerNumFromJ2 4U
/// @brief Max traffic sign num from HRZJ2
#define kMaxTrafficSignNumFromJ2 1U
/// @brief Max trans-point num
#define kMaxTransPointNum_Bus 4U
/// @brief Max traffic sign num
#define kMaxTrafficSignNum_Bus 16U
/// @brief Max lane num
#define kMaxLaneNum_Bus 8U
/// @brief Max cross walk num
#define kMaxCrossWalkNum_Bus 4U
/// @brief Max stop line num
#define kMaxStopLineNum_Bus 4U
/// @brief Max speed limit point num
#define kMaxSpeedLimitPointNum_Bus 20U
/// @brief Max curvature point num
#define kMaxCurvaturePointNum_Bus 20U
/// @brief Max roll angle point num
#define kMaxRollAnglePointNum_Bus 6U
/// @brief Max pitch angle point num
#define kMaxPitchAnglePointNum_Bus 6U

#define CfOBFU_MAX_OBSTACLE_NUM 32U
#define CfOBFU_MAX_SIDE_OBSTACLE_NUM 32U

// #ifndef DEFINED_TYPEDEF_FOR_Vector2f_
// #define DEFINED_TYPEDEF_FOR_Vector2f_

// // typedef struct
// // {
// //   /* position_covariance of x */
// //   real32_T x;

// //   /* position covariance of y */
// //   real32_T y;
// // } Vector2f;
// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Vector3f_
// #define DEFINED_TYPEDEF_FOR_Vector3f_

// // typedef struct
// // {
// //   real32_T x;
// //   real32_T y;
// //   real32_T z;
// // } Vector3f;
// // #endif

#ifndef DEFINED_TYPEDEF_FOR_OBJ_Cipv_
#define DEFINED_TYPEDEF_FOR_OBJ_Cipv_
// CIPV
typedef struct
{
  /// @brief CIPV front time to collision (ms)
  uint32 front_ttc;
  /// @brief CIPV front left time to collision (ms)
  uint32 front_left_ttc;
  /// @brief CIPV front right time to collision (ms)
  uint32 front_right_ttc;
  uint8 padding[4];
} OBJ_Cipv;
#endif

#ifndef DEFINED_TYPEDEF_FOR_OBJ_Obstacle_
#define DEFINED_TYPEDEF_FOR_OBJ_Obstacle_
// Obstacle
typedef struct
{
  /// @brief Whether is valid
  boolean is_valid;
  /// @brief Whether type is changed
  boolean is_type_changed;
  /// @brief Whether velocity is valid
  boolean is_velocity_valid;
  /// @brief Whether acceleration is valid
  boolean is_acceleration_valid;
  /// @brief Id [1, 255]
  uint16 id;
  /// @brief Tracking count
  uint16 tracking_count;
  /// @brief Tracking Id from Vision
  uint32 vision_id;
  /// @brief Obstacle type
  // INVALID = 0,          // Invalid
  // VEHICLE = 1,          // Vehicle
  // PEDESTRIAN = 2,       // Pedestrian
  // RIDER = 3,            // Rider
  // TRAFFIC_CONE = 4,     // Traffic cone
  // ANIMAL = 5,           // Animal
  // ROAD_DEBRIS = 6,      // Road debris
  // GENERAL_OBJECT = 7,   // Fence
  // WHEEL = 8             // Wheel
  uint8 type;
  /// @brief Vehicle subtype
  // UNKNOWN = 0,          // Unknown
  // BUS = 1,              // Bus
  // SMALL_MEDIUM_CAR = 2, // Small medium car
  // TRUCK = 3,            // Truck
  // SPECIAL_VEHICLE = 4,  // Special vehicle
  // TINY_CAR = 5,         // Tiny car
  // VAN = 6               // Van
  uint8 vehicle_subtype;
  /// @brief Rider subtype
  // UNKNOWN = 0,          // Unknown
  // CYCLIST = 1,          // Cyclist
  // MOTORCYCLIST = 2,     // Motorcyclist
  // TRICYCLIST = 3        // Tricyclist
  uint8 rider_subtype;
  /// @brief Pedestrian subtype
  // UNKNOWN = 0,          // Unknown
  // ADULT = 1,            // Adult
  // CHILD = 2             // Child
  uint8 pedestrian_subtype;
  /// @brief Pedestrian orientation
  // UNKNOWN = 0,          // Unknown
  // BACK = 1,             // Back
  // FRONT = 2,            // Front
  // LEFT = 3,             // Left
  // LEFT_FRONT = 4,       // Left front
  // LEFT_BACK = 5,        // Left back
  // RIGHT = 6,            // Right
  // RIGHT_FRONT = 7,      // Right front
  // RIGHT_BACK = 8        // Right back
  uint8 pedestrian_orientation;
  /// @brief Pedestrian pose
  // UNKNOWN = 0,          // Unknown
  // BENDING = 1,          // Bending
  // RIDING = 2,           // Riding
  // LYING = 3,            // Lying
  // STANDING = 4,         // Standing
  // SITTING = 5           // Sitting
  uint8 pedestrian_pose;
  /// @brief Motion status
  // UNKNOWN = 0,              // Unknown
  // SYNTROPY_MOVING = 1,      // Syntropy moving
  // ONCOMING_MOVING = 2,      // Stationary
  // STATIONARY = 3,           // Stationary
  // SYNTROPY_STOPPED = 4,     // Syntropy stopped
  // ONCOMING_STOPPED = 5,     // Syntropy stopped
  // FRONT_CROSSING_LEFT = 6,  // Front crossing left
  // FRONT_CROSSING_RIGHT = 7, // Front crossing right
  // RIGHT_CROSSING_LEFT = 8,  // Right crossing left
  // LEFT_CROSSING_RIGHT = 9   // Left crossing right
  uint8 motion_status;
  /// @brief Position zone
  // UNKNOWN = 0,          // Unknown
  // FRONT_LEFT = 1,       // Front left
  // FRONT = 2,            // Front
  // FRONT_RIGHT = 3,      // Front right
  // LEFT = 4,             // Left
  // RIGHT = 5,            // Right
  // REAR_LEFT = 6,        // Rear left
  // REAR = 7,             // Rear
  // REAR_RIGHT = 8        // Rear right
  uint8 position_zone;
  /// @brief CIPV type
  // INVALID = 0,          // Invalid
  // CIPV_F = 1,           // CIPV front
  // CIPV_FF = 2,          // CIPV front front
  // CIPV_FR = 3,          // CIPV front right
  // CIPV_FFR = 4,         // CIPV front front right
  // CIPV_FL = 5,          // CIPV front left
  // CIPV_FFL = 6,         // CIPV front front left
  // CIPV_RRear = 8,       // CIPV rear rear
  // CIPV_R = 9,           // CIPV rear
  // CIPV_RR = 10,         // CIPV rear right
  // CIPV_RL = 11,         // CIPV rear left
  // CIPV_RRearRI = 12,    // CIPV rear rear right
  // CIPV_RRearLF = 13,    // CIPV rear rear left
  // CIPV_RI = 14,         // CIPV right
  // CIPV_LF = 15          // CIPV left
  uint8 cipv_type;
  /// @brief Cut type
  // UNKNOWN = 0,          // Unknown
  // NO_CUT = 1,           // No cut
  // CUT_IN = 2,           // Cut in
  // CUT_OUT = 3           // Cut out
  uint8 cut_type;
  /// @brief Turn indicator
  // UNKNOWN = 0,          // Unknown
  // NONE = 1,             // None
  // LEFT = 2,             // Left turn
  // RIGHT = 3,            // Right turn
  // EMERGENCY = 4         // Both lights
  uint8 turn_indicator;
  /// @brief Brake indicator
  // UNKNOWN = 0,          // Unknown
  // OFF = 1,              // Off
  // ON = 2                // On
  uint8 brake_indicator;
  /// @brief Lane assignment
  // UNKNOWN = 0,          // Unknown
  // LEFT_LEFT = 1,        // Left left
  // LEFT = 2,             // Left
  // HOST = 3,             // Host
  // RIGHT = 4,            // Right
  // RIGHT_RIGHT = 5       // Right right
  uint8 lane_assignment;
  /// @brief Tracking status
  // INVALID = 0,          // Invalid
  // CREATE = 1,           // New
  // UPDATE_NEW = 2,       // First update
  // UPDATE = 3,           // Second update
  // UPDATE_STABLE = 4,    // More updates
  // PREDICT_NEW = 5,      // First predict
  // PREDICT = 6           // More predicts
  uint8 tracking_status;
  /// @brief Confidence
  uint8 confidence;
  /// @brief Feature target flag (flag & FeatureTargetFlag > 0)
  // INVALID = 0,          // Invalid
  // ACC = 1,              // ACC target
  // AEB = 2               // AEB target
  uint8 feature_target_flag;
  /// @brief Source flag (flag & SensorType > 0)
  // Not_Fused=0,
  // Fused_Camera=1,
  // Fused_Radar=2,
  // Fused_CameraAndRadar=3,
  // Fused_Lidar=4,
  // Fused_CameraAndLidar=5,
  // Fused_RadarAndLidar=6,
  // Fused_CmaeraAndRadarAndLidar=7
  uint8 source_flag;
  uint8 sensor_sources[3];
  uint8 nearest_side;
  uint8 traffic_scenario;
  uint8 cmbb_primary_confidence;
  uint8 cmbb_secondary_confidence;
  uint8 fcw_confidence;
  uint8 padding_1[3];
  float curvature;
  /// @brief Nearest point to ego car in relative coordinate
  Vector2f nearest_point_rel;
  /// @brief Nearest point standard deviation
  Vector2f nearest_point_rel_sd;
  /// @brief Heading in relative coordinate  (-pi, pi]
  float heading_rel;
  /// @brief Heading standard deviation
  float heading_rel_sd;
  /// @brief Absolute velocity in host vehicle coordinate (m/s)
  Vector2f velocity_abs;
  /// @brief Absolute speed in host vehicle coordinate (!Scalar!) (m/s)
  float speed_abs_scalar;
  /// @brief Velocity standard deviation
  Vector2f velocity_sd;
  /// @brief Absolute acceleration in host vehicle coordinate (m/s^2)
  Vector2f acceleration_abs;
  /// @brief Absolute acceleration in host vehicle coordinate (!Scalar!) (m/s)
  float acceleration_abs_scalar;
  /// @brief Acceleration standard deviation
  Vector2f acceleration_sd;
  /// @brief Length (m)
  float length;
  /// @brief Width (m)
  float width;
  /// @brief Height (m)
  float height;
  /// @brief Bounding box points
  Vector2f bounding_box_points[kBoundingBoxPointNum_Bus];
} OBJ_Obstacle;
#endif

#ifndef DEFINED_TYPEDEF_FOR_OBJ_DataBus_
#define DEFINED_TYPEDEF_FOR_OBJ_DataBus_
// Fused object communication data
typedef struct
{
  /// @brief Time stamp (us)
  sint32 timestamp_high;
  uint32 timestamp_low;
  /// @brief Latency for the nearest sensor object (ms)
  uint32 latency;
  uint8 padding_1[4];
  /// @brief CIPV
  OBJ_Cipv cipv;
  /// @brief Obstacles
  OBJ_Obstacle obstacles[CfOBFU_MAX_OBSTACLE_NUM];
  /// @brief Obstacle num
  uint8 num_obstacles;
  uint8 padding_2[7];
} OBJ_DataBus;
#endif

#ifndef DEFINED_TYPEDEF_FOR_TsSIDEOBJ_HEADER_
#define DEFINED_TYPEDEF_FOR_TsSIDEOBJ_HEADER_
typedef struct
{
    uint32_T timestamp;
    uint8_T front_side_obstacle_num;
    uint8_T rear_side_obstacle_num;
} TsSIDEOBJ_HEADER;
#endif // !DEFINED_TYPEDEF_FOR_TsSIDEOBJ_HEADER_

#ifndef DEFINED_TYPEDEF_FOR_TsSIDEOBJ_
#define DEFINED_TYPEDEF_FOR_TsSIDEOBJ_
typedef struct
{
    uint8_T obs_id;
    uint8_T obs_update_flag;
    Vector2f obs_position;
    Vector2f obs_velocity_rel;
    Vector2f obs_acceleration_rel;
    Vector2f obs_position_sd;
    real32_T obs_velocity_sd_x;
    real32_T obs_heading_angle;
    uint8_T obs_confidence;
    uint8_T obs_type;
    uint8_T obs_motion_pattern;
    uint8_T obs_cipv_type;
    real32_T obs_width;
    real32_T obs_length;
} TsSIDEOBJ;
#endif // !DEFINED_TYPEDEF_FOR_TsSIDEOBJ_

#ifndef DEFINED_TYPEDEF_FOR_TsSIDEOBJ_DATABUS_
#define DEFINED_TYPEDEF_FOR_TsSIDEOBJ_DATABUS_
typedef struct
{
    TsSIDEOBJ_HEADER header;
    TsSIDEOBJ front_obstacle[CfOBFU_MAX_SIDE_OBSTACLE_NUM];
    TsSIDEOBJ rear_obstacle[CfOBFU_MAX_SIDE_OBSTACLE_NUM];
} TsSIDEOBJ_DATABUS;
#endif // !DEFINED_TYPEDEF_FOR_TsSIDEOBJ_DATABUS_



#ifndef DEFINED_TYPEDEF_FOR_MAP_Routing_
#define DEFINED_TYPEDEF_FOR_MAP_Routing_
// Routing
typedef struct
{
  /// @brief Action
  // UNKNOWN = 0,                // Unknown
  // TURN_LEFT = 1,              // Turn left
  // TURN_RIGHT = 2,             // Turn right
  // LEFT_FRONT = 3,             // Left front
  // RIGHT_FRONT = 4,            // Right front
  // LEFT_TURN_AROUND = 5,       // Left turn around
  // STRAIGHT = 6,               // Straight
  // LEFT_BACK = 7,              // Left back
  // RIGHT_BACK = 8,             // Right back
  // ARRIVED_WAYPOINT= 9,        // Arrived waypoint
  // ENTER_ROUNDABOUT = 10,      // Enter roundabout
  // EXIT_ROUNDABOUT = 11,       // Exit roundabout
  // ARRIVED_SERVICE_AREA = 12,  // Arrived serv
  // ARRIVED_TOLLGATE = 13,      // Arrived tollgate
  // ARRIVED_DESTINATION = 14,   // Arrived destination
  // ARRIVED_TUNNEL = 15,        // Arrived tunnel
  // SPECIAL_CONTINUE = 16,      // Special continue
  // U_TURN_RIGHT = 17           // U turn right
  uint8 action;
  /// @brief Target lane type
  // UNKNOWN = 0,              // Unknown
  // NORMAL = 1,               // Normal
  // ENTRY = 2,                // Entry (on ramp)
  // EXIT = 3,                 // Exit (off ramp
  // AUXILIARY = 4,            // Auxiliary
  // EMERGENCY = 5,            // Emergency
  // RESTRICTED_FORBIDDEN = 6, // Restricted forbidden
  // RESTRICTED_USABLE = 7,    // Restricted usable
  // HOV = 8,                  // Hov
  // EXPRESS = 9,              // Express
  // REVERSIBLE = 10,          // Reversible
  // SLOW = 11,                // Slow
  // DRIVABLE_SHOULDER = 12,   // Drivable shoulder
  // TURN_OR_SUICIDE = 13,     // Turn or suicide
  // CONNECT_RAMP = 14         // Connect ramp
  uint8 target_lane_type;
  /// @brief Whether next target is junction
  boolean is_next_target_junction;
  /// @brief Target speed limit max
  uint8 target_speed_limit;
  /// @brief Current speed limit max
  uint8 current_speed_limit;
  /// @brief Target lane id
  sint8 target_lane_id;
  uint8 padding_1[2];
  /// @brief Time to destination (s)
  uint32 time_to_destination;
  /// @brief Distance to next target
  float distance_to_next_target;
  /// @brief Distance to destination
  float distance_to_destination;
  uint8 padding_2[4];
} MAP_Routing;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_SlopePoints_
#define DEFINED_TYPEDEF_FOR_MAP_SlopePoints_
// Slope points
typedef struct 
{
  /// @brief Whether is valid
  boolean is_valid;
  uint8  padding_1[3];
  /// @brief Slope points (rad, [-PI, PI]) (10m per point, 2000m front)
  float slope_points[kMaxSlopePointNum_Bus];
  /// @brief Slope point num
  uint8 num_slope_points;
  uint8 padding_2[3];
}MAP_SlopePoints;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_LaneMarker_
#define DEFINED_TYPEDEF_FOR_MAP_LaneMarker_
// Lane marker
typedef struct
{
  /// @brief Whether is valid
  boolean is_valid;
  /// @brief Fusion status
  // NONE = 0,               // None
  // VISION_ONLY = 1,        // Vision only
  // HDMAP_ONLY = 2,         // HD map only
  // VISION_HDMAP = 3,       // Vision and HD map
  // VISION_HDMAP_OBJECT = 4 // Vision, HD map and object
  uint8 fusion_status;
  /// @brief HighwayLaneMarkerId for highway
  uint8 id;
  uint8 padding_1;
  /// @brief Tracking count
  uint16 tracking_count;
  /// @brief Lane marker color
  // UNKNOWN = 0,          // Unknown
  // WHITE = 1,            // White
  // YELLOW = 2,           // Yellow
  // BLUE = 3,             // Blue
  // GREEN = 4,            // Green
  // RED = 5               // Red
  uint8 color;
  /// @brief Lane marker type
  // UNKNOWN = 0,                  // Unknown
  // SOLID = 1,                    // Solid
  // ROAD_EDGE = 2,                // Road edge
  // DASHED = 3,                   // Dashed
  // LEFT_DASHED_RIGHT_SOLID = 4,  // Left dashed right solid
  // LEFT_SOLID_RIGHT_DASHED = 5,  // Left solid right dashed
  // DOUBLE_DASHED = 6,            // Double dashed
  // DOUBLE_SOLID = 7,             // Double solid
  // BARRIER = 8                   // Barrier
  uint8 type;
  /// @brief Quality
  uint8 quality;
   /// @brief Lane measuring status
  // UNKNOWN = 0,      // Unknown
  // NEW = 1,          // New
  // MEASURED = 2,     // Measured
  // PREDICTED = 3,    // Predicted
  // NOT_VALID = 4     // Not valid
  uint8 measuring_status;
  uint8 padding_2[2];
  /// @brief Lane marker width (m)
  float width;
  /// @brief c0: position
  float c0_position;
  /// @brief c1: heading
  float c1_heading;
  /// @brief c2: curvature
  float c2_curvature;
  /// @brief c3: curvature_derivative
  float c3_curvature_derivative;
  /// @brief Start point x in relative coordinate
  float start_rel_x;
  /// @brief End point x in relative coordinate
  float end_rel_x;
  /// @brief Lateral distance to host (m)
  float distance_to_host;
  uint8 padding_3[4];
} MAP_LaneMarker;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_RearLaneMarker_
#define DEFINED_TYPEDEF_FOR_MAP_RearLaneMarker_
// Lane marker
typedef struct
{
  /// @brief Whether is valid
  boolean is_valid;
  /// @brief Fusion status
  // NONE = 0,               // None
  // VISION_ONLY = 1,        // Vision only
  // HDMAP_ONLY = 2,         // HD map only
  // VISION_HDMAP = 3,       // Vision and HD map
  // VISION_HDMAP_OBJECT = 4 // Vision, HD map and object
  uint8 fusion_status;
  /// @brief HighwayLaneMarkerId for highway
  uint8 id;
  uint8 padding_1;
  /// @brief Tracking count
  uint16 tracking_count;
  /// @brief Lane marker color
  // UNKNOWN = 0,          // Unknown
  // WHITE = 1,            // White
  // YELLOW = 2,           // Yellow
  // BLUE = 3,             // Blue
  // GREEN = 4,            // Green
  // RED = 5               // Red
  uint8 color;
  /// @brief Lane marker type
  // UNKNOWN = 0,                  // Unknown
  // SOLID = 1,                    // Solid
  // ROAD_EDGE = 2,                // Road edge
  // DASHED = 3,                   // Dashed
  // LEFT_DASHED_RIGHT_SOLID = 4,  // Left dashed right solid
  // LEFT_SOLID_RIGHT_DASHED = 5,  // Left solid right dashed
  // DOUBLE_DASHED = 6,            // Double dashed
  // DOUBLE_SOLID = 7,             // Double solid
  // BARRIER = 8                   // Barrier
  uint8 type;
  /// @brief Quality
  uint8 quality;
   /// @brief Lane measuring status
  // UNKNOWN = 0,      // Unknown
  // NEW = 1,          // New
  // MEASURED = 2,     // Measured
  // PREDICTED = 3,    // Predicted
  // NOT_VALID = 4     // Not valid
  uint8 measuring_status;
  uint8 padding_2[2];
  /// @brief Lane marker width (m)
  float width;
  /// @brief c0: position
  float c0_position;
  /// @brief c1: heading
  float c1_heading;
  /// @brief c2: curvature
  float c2_curvature;
  /// @brief c3: curvature_derivative
  float c3_curvature_derivative;
  /// @brief Start point x in relative coordinate
  float start_rel_x;
  /// @brief End point x in relative coordinate
  float end_rel_x;
  /// @brief Lateral distance to host (m)
  float distance_to_host;
  uint8 padding_3[4];
} MAP_RearLaneMarker;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_TransPoint_
#define DEFINED_TYPEDEF_FOR_MAP_TransPoint_
// Trans-point
typedef struct
{
  /// @brief Whether is valid
  boolean is_valid;
  /// @brief Type
  // UNKNOWN = 0,          // Unknown
  // MERGE = 1,            // Merge
  // SPLIT = 2             // Split
  uint8 type;
  /// @brief Merge subtype
  // UNKNOWN = 0,                    // Unknown
  // MERGE_TO_LEFT = 1,              // Merge to left
  // MERGE_TO_RIGHT = 2,             // Merge to right
  // MERGE_FROM_LEFT = 3,            // Merge from left
  // MERGE_FROM_RIGHT = 4,           // Merge from right
  // JOIN_FROM_LEFT = 5,             // Join from left
  // JOIN_FROM_RIGHT = 6,            // Join from right
  // MERGE_FROM_LEFT_TO_CENTER = 7,  // Merge from left to center
  // MERGE_FTOM_RIGHT_TO_CENTER = 8  // Merge from right to center
  uint8 merge_subtype;
  /// @brief Split subtype
  // UNKNOWN = 0,                    // Unknown
  // DISJOINT_FROM_LEFT = 1,         // Disjoint from left
  // DISJOINT_FROM_AHEAD = 2,        // Disjoint from ahead
  // DISJOINT_FROM_RIGHT = 3,        // Disjoint from right
  // OPEN_TO_LEFT = 4,               // Open to left
  // OPEN_TO_RIGHT = 5,              // Open to right
  // OPEN_SHARPLY_TO_LEFT = 6,       // Open sharply to left
  // OPEN_SHARPLY_TO_RIGHT = 7,      // Open sharply to right
  // SPLIT_FROM_LEFT = 8,            // Split from left
  // SPLIT_FROM_AHEAD = 9,           // Split from ahead
  // SPLIT_FROM_RIGHT = 10,          // Split from right
  // EXPAND_TO_LEFT = 11,            // Expand to left
  // EXPAND_TO_RIGHT = 12            // Expand to right
  uint8 split_subtype;
  /// @brief Position to host
  // UNKNOWN = 0,                    // Unknown
  // HOST_LEFT_LANEMARK = 1,         // Host left lane marker
  // HOST_RIGHT_LANEMARK = 2,        // Host right lane marker
  // LEFT_LEFT_LANEMARK = 3,         // Left lane left lane marker
  // LEXT_RIGHT_LANEMARK = 4,        // Left lane right lane marker
  // RIGHT_LEFT_LANEMARK = 5,        // Right lane left lane marker
  // RIGHT_RIGHT_LANEMARK = 6        // Right lane right lane marker
  uint8 position_to_host;
  uint8 padding[3];
  /// @brief Position in relative coordinate
  Vector2f position_rel;
} MAP_TransPoint;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_TrafficSign_
#define DEFINED_TYPEDEF_FOR_MAP_TrafficSign_
// Traffic sign
typedef struct
{
  /// @brief Fusion status
  // NONE = 0,               // None
  // VISION_ONLY = 1,        // Vision only
  // HDMAP_ONLY = 2,         // HD map only
  // VISION_HDMAP = 3,       // Vision and HD map
  // VISION_HDMAP_OBJECT = 4 // Vision, HD map and object
  uint8 fusion_status;
  /// @brief Sign type
  // UNKNOWN = 0,            // Unknown
  // SPEED_LIMIT = 1,        // Speed limit
  // STOP_SIGN = 2,          // Stop sign
  // CANCEL_SPEED_LIMIT = 3, // Cancel speed limit
  // NO_ENTRY = 4,           // No entry
  // OTHERS = 5              // Other types
  uint8 type;
  /// @brief Value for speed limit etc.
  sint16 value;
  /// @brief Position in relative coordinate
  Vector2f position_rel;
  uint8 padding[4];
} MAP_TrafficSign;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_TrafficLight_
#define DEFINED_TYPEDEF_FOR_MAP_TrafficLight_
// Traffic light
typedef struct
{
  /// @brief Whether is valid
  boolean is_valid;
  /// @brief State
  // UNKNOWN = 0,          // Unknown
  // RED = 1,              // Red
  // YELLOW = 2,           // Yellow
  // GREEN = 3,            // Green
  // BLACK = 4,            // Black
  // RED_YELLOW = 5,       // Red yellow
  // RED_FLASH = 6,        // Red flash
  // YELLOW_FLASH = 7      // Yellow flash
  uint8 state;
  /// @brief Count down time (sec) (-1 for invalid)
  sint8 count_down_time;
  uint8 padding_1;
  /// @brief Position in relative coordinate
  Vector2f position_rel;
  uint8 padding_2[4];
} MAP_TrafficLight;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_LaneSpeedLimitPoint_
#define DEFINED_TYPEDEF_FOR_MAP_LaneSpeedLimitPoint_
// Speed limit point
typedef struct {
  /// @brief S (m)
  uint32 s;
  /// @brief Speed limit max (km/h)
  uint32 speed_limit_max;
}MAP_LaneSpeedLimitPoint;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_Lane_
#define DEFINED_TYPEDEF_FOR_MAP_Lane_
// Lane
typedef struct
{
  /// @brief Whether is valid
  boolean is_valid;
  /// @brief Fusion status
  // NONE = 0,               // None
  // VISION_ONLY = 1,        // Vision only
  // HDMAP_ONLY = 2,         // HD map only
  // VISION_HDMAP = 3,       // Vision and HD map
  // VISION_HDMAP_OBJECT = 4 // Vision, HD map and object
  uint8 fusion_status;
  /// @brief Id (0: ego, 1: left 1, 2: right 1, ...)
  uint8 id;
  uint8 padding_1;
  /// @brief Tracking count
  uint16 tracking_count;
  /// @brief Type
  // UNKNOWN = 0,              // Unknown
  // NORMAL = 1,               // Normal
  // ENTRY = 2,                // Entry (on ramp)
  // EXIT = 3,                 // Exit (off ramp
  // AUXILIARY = 4,            // Auxiliary
  // EMERGENCY = 5,            // Emergency
  // RESTRICTED_FORBIDDEN = 6, // Restricted forbidden
  // RESTRICTED_USABLE = 7,    // Restricted usable
  // HOV = 8,                  // Hov
  // EXPRESS = 9,              // Express
  // REVERSIBLE = 10,          // Reversible
  // SLOW = 11,                // Slow
  // DRIVABLE_SHOULDER = 12,   // Drivable shoulder
  // TURN_OR_SUICIDE = 13,     // Turn or suicide
  // CONNECT_RAMP = 14         // Connect ramp
  uint8 type;
  /// @brief Turn type
  // UNKNOWN = 0,                  // Unknown
  // ONLY_FORWARD = 1,             // Only forward
  // ONLY_LEFT_TURN = 2,           // Only left turn
  // ONLY_RIGHT_TURN = 3,          // Only right turn
  // FORWARD_LEFT_TURN = 4,        // Forward and left turn
  // FORWARD_RIGHT_TURN = 5,       // Forward and right turn
  // LEFT_RIGHT_TURN = 6,          // Left and right turn
  // U_TURN = 7,                   // U turn
  // NO_U_TURN = 8,                // No U turn
  // NO_LEFT_TURN = 9,             // No left turn
  // NO_RIGHT_TURN = 10,           // No right turn
  // NO_LEFT_RIGHT_TURN = 11,      // No left and right turn
  // FORWARD_LEFT_RIGHT_TURN = 12, // Forward or left or right turn
  // LEFT_U_TURN = 13,             // Left or U turn
  // FORWARD_U_TURN = 14           // Forward or U turn
  uint8 turn_type;
  /// @brief Direction
  // UNKNOWN = 0,            // Unknown
  // BOTH = 1,               // Both direction
  // ALONG = 2,              // Same direction
  // AGAINST = 3             // Opposite direction
  uint8 direction;
  /// @brief Road type
  // UNKNOWN = 0,          // Unknown
  // HIGHWAY = 1,          // Highway
  // URBAN = 2,            // Urban
  // RURAL = 3             // Rural
  uint8 road_type;
  /// @brief Left lane marker index (-1 for none-exist)
  sint8 left_lane_marker_index;
  /// @brief��Right lane marker index (-1 for none-exist)
  sint8 right_lane_marker_index;
  /// @brief Speed limit max (km/h)
  uint8 speed_limit_max;
  /// @brief Speed limit min (km/h)
  uint8 speed_limit_min;
  uint8 padding_2[2];
  /// @brief Width (m)
  float width;
  /// @brief Length (m)
  float length;
  /// @brief Traffic light
  MAP_TrafficLight traffic_light;
  /// @brief Speed limit points, 2000m front
  MAP_LaneSpeedLimitPoint speed_limit_points[kMaxSpeedLimitPointNum_Bus];
  uint8 num_speed_limit_points;
  uint8 padding_3[3];
  /// @brief Curvature points (1/m) (100m per point, 2000m front)
  float curvature_points[kMaxCurvaturePointNum_Bus];
  /// @brief Curvature point num
  uint8 num_curvature_points;
  uint8 padding_4[3];
  /// @brief Roll angle points (rad, [-PI, PI]) (10m per point, 50m front)
  float roll_angle_points[kMaxRollAnglePointNum_Bus];
  /// @brief Roll angle point num
  uint8 num_roll_angle_points;
  uint8 padding_5[3];
  /// @brief Pitch angle points (rad, [-PI, PI]) (10m per point, 50m front)
  float pitch_angle_points[kMaxPitchAnglePointNum_Bus];
  /// @brief Pitch angle point num
  uint8 num_pitch_angle_points;
  uint8 padding_6[3];
} MAP_Lane;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_CrossWalk_
#define DEFINED_TYPEDEF_FOR_MAP_CrossWalk_
// Cross walk
typedef struct
{
  /// @brief Fusion status
  // NONE = 0,               // None
  // VISION_ONLY = 1,        // Vision only
  // HDMAP_ONLY = 2,         // HD map only
  // VISION_HDMAP = 3,       // Vision and HD map
  // VISION_HDMAP_OBJECT = 4 // Vision, HD map and object
  uint8 fusion_status;
  /// @brief Lane index
  uint8 lane_index;
  /// @brief Quality
  uint8 quality;
  uint8 padding_1;
  /// @brief Vertices in relative coordinate
  Vector2f vertices_rel[kCrosswalkVertexNum_Bus];
  uint8 padding_2[4];
} MAP_CrossWalk;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_StopLine_
#define DEFINED_TYPEDEF_FOR_MAP_StopLine_
// Stop line
typedef struct
{
  /// @brief Fusion status
  // NONE = 0,               // None
  // VISION_ONLY = 1,        // Vision only
  // HDMAP_ONLY = 2,         // HD map only
  // VISION_HDMAP = 3,       // Vision and HD map
  // VISION_HDMAP_OBJECT = 4 // Vision, HD map and object
  uint8 fusion_status;
  /// @brief Lane index
  uint8 lane_index;
  /// @brief Quality
  uint8 quality;
  uint8 padding_1;
  /// @brief Start point in relative coordinate
  Vector2f start_rel;
  /// @brief End point in relative coordinate
  Vector2f end_rel;
  uint8 padding_2[4];
} MAP_StopLine;
#endif

#ifndef DEFINED_TYPEDEF_FOR_MAP_DataBus_
#define DEFINED_TYPEDEF_FOR_MAP_DataBus_
// Fused local map communication data
typedef struct
{
  /// @brief Time stamp (us)
  sint32 timestamp_high;
  uint32 timestamp_low;
  /// @brief Routing
  MAP_Routing routing;
  /// @brief Lane markers
  MAP_LaneMarker lane_markers[kMaxLaneMarkerNum_Bus];
  /// @brief Lane marker num
  uint8 num_lane_markers;
  uint8 padding_1[7];
  /// @brief Trans-points
  MAP_TransPoint trans_points[kMaxTransPointNum_Bus];
  /// @brief Trans-point num
  uint8 num_trans_points;
  uint8 padding_2[7];
  /// @brief Traffic signs
  MAP_TrafficSign traffic_signs[kMaxTrafficSignNum_Bus];
  /// @brief Traffic sign num
  uint8 num_traffic_signs;
  uint8 padding_3[7];
  /// @brief Lanes
  MAP_Lane lanes[kMaxLaneNum_Bus];
  /// @brief Lane num
  uint8 num_lanes;
  uint8 padding_4[7];
  /// @brief Cross walks
  MAP_CrossWalk cross_walks[kMaxCrossWalkNum_Bus];
  /// @brief Cross walk num
  uint8 num_cross_walks;
  uint8 padding_5[7];
  /// @brief Stop lines
  MAP_StopLine stop_lines[kMaxStopLineNum_Bus];
  /// @brief Stop line num
  uint8 num_stop_lines;
  uint8 padding_6[7];
  /// @brief Slope points
  MAP_SlopePoints slope_points;
  /// @brief Rear lane markers
  MAP_RearLaneMarker rear_lane_markers[kMaxLaneMarkerNum_Bus];
  /// @brief Lane marker num
  uint8 num_rear_lane_markers;
  uint8 padding_7[7];
} MAP_DataBus;
#endif

#endif /* OBFU_SUBSYSTEM_INTEGRATED_TYPES_H_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
