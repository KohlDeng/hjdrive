/*
 * File: VDYN_subsystem_integrated_types.h
 *
 * Code generated for Simulink model 'VDYN_subsystem_integrated'.
 *
 * Model version                  : 1.2902
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Tue Nov 23 15:28:15 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Infineon->TriCore
 * Code generation objectives:
 *    1. MISRA C:2012 guidelines
 *    2. Safety precaution
 *    3. Execution efficiency
 *    4. RAM efficiency
 *    5. ROM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_VDYN_subsystem_integrated_types_h_
#define RTW_HEADER_VDYN_subsystem_integrated_types_h_
#include "common_type.h"

/* Includes for objects with custom storage classes. */
// #include "PPnc_config.h"

/*
 * Check that imported macros with storage class "ImportedDefine" are defined
 */
// #ifndef PassengerVehicle_VDYN
// #error The variable for the parameter "PassengerVehicle_VDYN" is not defined
// #endif

// #ifndef TRUCK_VDYN
// #error The variable for the parameter "TRUCK_VDYN" is not defined
// #endif

// #ifndef VDYN_TYPE
// #error The variable for the parameter "VDYN_TYPE" is not defined
// #endif

// /* Model Code Variants */
// #ifndef VDYN_PassengerCar_Model
// #define VDYN_PassengerCar_Model        (VDYN_TYPE == PassengerVehicle_VDYN)
// #endif

// #ifndef VDYN_Truck_Model
// #define VDYN_Truck_Model               (VDYN_TYPE == TRUCK_VDYN)
// #endif

// /* Exactly one variant for '<S3>/VDYN_EGMODataCor' should be active */
// #if ((VDYN_PassengerCar_Model) ? 1 : 0) + ((VDYN_Truck_Model) ? 1 : 0) != 1
// #error Exactly one variant for '<S3>/VDYN_EGMODataCor' should be active
// #endif

// /* Exactly one variant for '<S3>/VDYN_MGE' should be active */
// #if ((VDYN_PassengerCar_Model) ? 1 : 0) + ((VDYN_Truck_Model) ? 1 : 0) != 1
// #error Exactly one variant for '<S3>/VDYN_MGE' should be active
// #endif

// /* Exactly one variant for '<S3>/VDYN_VRE_LSE' should be active */
// #if ((VDYN_PassengerCar_Model) ? 1 : 0) + ((VDYN_Truck_Model) ? 1 : 0) != 1
// #error Exactly one variant for '<S3>/VDYN_VRE_LSE' should be active
// #endif

// /* Exactly one variant for '<S21>/VDYN_VRE_LSE' should be active */
// #if ((VDYN_PassengerCar_Model) ? 1 : 0) + ((VDYN_Truck_Model) ? 1 : 0) != 1
// #error Exactly one variant for '<S21>/VDYN_VRE_LSE' should be active
// #endif


#ifndef DEFINED_TYPEDEF_FOR_VDYN_ModelTransForTruck_DataBus_
#define DEFINED_TYPEDEF_FOR_VDYN_ModelTransForTruck_DataBus_

typedef struct
{
  /* Mass parameters for 3-DOF models to 2-DOF models. */
  real32_T VDYN_SelfAdaptMass;

  /* Lf parameters for 3-DOF models to 2-DOF models */
  real32_T VDYN_SelfAdaptLf;

  /* Lr parameters for 3-DOF models to 2-DOF models */
  real32_T VDYN_SelfAdaptLr;

  /* Iz parameters for 3-DOF models to 2-DOF models */
  real32_T VDYN_SelfAdaptRotationalInertia;

  /* Cf parameters for 3-DOF models to 2-DOF models */
  real32_T VDYN_SelfAdaptFrontTireStiffness;

  /* Cr parameters for 3-DOF models to 2-DOF models */
  real32_T VDYN_SelfAdaptRearTireStiffness;

  /* Model parameters valid signal. */
  boolean_T ModelTransForTruckValid;
}
VDYN_ModelTransForTruck_DataBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VDYN_TransformDataBus_
#define DEFINED_TYPEDEF_FOR_VDYN_TransformDataBus_

typedef struct
{
  /* The longitudinal position change of the vehicle's previous loop in the current coordinate system */
  real32_T dx;

  /* The lateral position change of the vehicle's previous loop in the current coordinate system */
  real32_T dy;

  /* The driving heading angle change of the vehicle's previous loop in the current coordinate system */
  real32_T dheading;

  /* Sine of change in heading angle. */
  real32_T sin_dheading;

  /* Cosine of change in heading angle. */
  real32_T cos_dheading;

  /* Valid signal of history position transform data. */
  boolean_T TransformDataValid;
}
VDYN_TransformDataBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VDYN_TransformDataBus_50ms_
#define DEFINED_TYPEDEF_FOR_VDYN_TransformDataBus_50ms_

typedef struct
{
  /* The longitudinal position change of the vehicle's previous 5 loops in the current coordinate system */
  real32_T dx;

  /* The lateral position change of the vehicle's previous 5 loops in the current coordinate system */
  real32_T dy;

  /* The driving heading angle change of the vehicle's previous 5 loops in the current coordinate system */
  real32_T dheading;

  /* Sine of change in heading angle. */
  real32_T sin_dheading;

  /* Cosine of change in heading angle. */
  real32_T cos_dheading;

  /* Valid signal of 50ms history position transform data. */
  boolean_T TransformData50msValid;
}
VDYN_TransformDataBus_50ms;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VDYN_DataBus_
#define DEFINED_TYPEDEF_FOR_VDYN_DataBus_

typedef struct
{
  uint32_T VDYN_timestamp;

  /* 0 = Ukwn: Unknown
     1 = StandStillVal1: Standing still information with at least QM intetrity
     2 = StandStillVal2: Standing still information with at least ASIL B intetrity
     3 = StandStillVal3: Standing still information with at least ASIL C intetrity
     4 = RollgFwdVal1: Rolling forward information with at least ASIL B integrity
     5 = RollgFwdVal2: Rolling forward information with at least ASIL C integrity
     6 = RollgBackwVal1: Rolling backward information with at least ASIL B integrity
     7 = RollgBackwVal2: Rolling backward information with at least ASIL C integrity */
  uint8_T VDYN_VehicleMotionState;

  /* Ego vehicle driving curvature. */
  real32_T VDYN_EgoCurvature;

  /* Valid signal of curvature. */
  boolean_T VDYN_EgoCurvatureValid;

  /* Ego vehicle driving curvature rate. */
  real32_T VDYN_EgoCurvatureRate;

  /* Valid signal of curvature rate. */
  boolean_T VDYN_EgoCurvatureRateValid;

  /* Ego vehicle slip angle. */
  real32_T VDYN_EgoSlipAngle;

  /* Valid signal of slip angle. */
  boolean_T VDYN_EgoSlipAngleValid;

  /* When the speed signal of CAN is available, VDYN forwards the speed signal.
     When CAN speed is unavailable, the speed is estimated by wheel speed and acceleration sensors. */
  real32_T VDYN_EgoLinearVelocity;

  /* Valid signal of V. */
  boolean_T VDYN_EgoLinearVelocityValid;

  /* 0 = DevOfDataUndefd: The accuracy of the data is undefinable
     1 = DataTmpUndefdAndEvlnInProgs: The data is temp. undefined, evaluation in progress
     2 = DevOfDataNotWithinRngAllwd: The accuracy of the data is not within specification
     3 = DataCalcdWithDevDefd: The data is calculated with the specified accuracy */
  uint8_T VDYN_EgoLinearVelocity_Confidence;

  /* Vehicle_Longitudinal_Velocity */
  real32_T VDYN_LongitudinalVelocity;

  /* Valid signal of Vx. */
  boolean_T VDYN_LongitudinalVelocityValid;

  /* Vehicle lateral velocity */
  real32_T VDYN_LateralVelocity;

  /* Valid signal of Vy. */
  boolean_T VDYN_LateralVelocityValid;

  /* Longitudinal acceleration of the vehicle */
  real32_T VDYN_LongitudinalAcceleration;

  /* Longitudinal acceleration calculated from speed. */
  real32_T VDYN_LongitudinalAcceleration_FrmSpd;

  /* Valid signal of Ax. */
  boolean_T VDYN_LongitudinalAccelerationValid;

  /* 0 = DevOfDataUndefd: The accuracy of the data is undefinable
     1 = DataTmpUndefdAndEvlnInProgs: The data is temp. undefined, evaluation in progress
     2 = DevOfDataNotWithinRngAllwd: The accuracy of the data is not within specification
     3 = DataCalcdWithDevDefd: The data is calculated with the specified accuracy */
  uint8_T VDYN_LongitudinalAcceleration_Confidence;

  /* Lateral acceleration of the vehicle */
  real32_T VDYN_LateralAcceleration;

  /* Valid signal of Ay */
  boolean_T VDYN_LateralAccelerationValid;

  /* 0 = DevOfDataUndefd: The accuracy of the data is undefinable
     1 = DataTmpUndefdAndEvlnInProgs: The data is temp. undefined, evaluation in progress
     2 = DevOfDataNotWithinRngAllwd: The accuracy of the data is not within specification
     3 = DataCalcdWithDevDefd: The data is calculated with the specified accuracy */
  uint8_T VDYN_LateralAcceleration_Confidence;

  /* Semi-Trailer Truck Articulated Angle. */
  real32_T VDYN_Ang_TruckArtAng;

  /* Valid signal of articulated angle. */
  boolean_T VDYN_TruckArtAngValid;

  /* Semi-Trailer Truck Articulated Angle Rate. */
  real32_T VDYN_v_TruckArtAngRate;

  /* Valid signal of articulated angle rate. */
  boolean_T VDYN_TruckArtAngRateValid;

  /* Semi-Trailer Truck Articulated Angle Rate Acc. */
  real32_T VDYN_a_TruckArtAngAcc;

  /* Valid signal of articulated angle acceleration. */
  boolean_T VDYN_TruckArtAngAccValid;

  /* Ego vehicle yaw rate estimated by VDYN subsystem. */
  real32_T VDYN_EgoYawRate;

  /* Valid signal of yaw rate. */
  boolean_T VDYN_EgoYawRateValid;

  /* 0 = DevOfDataUndefd: The accuracy of the data is undefinable
     1 = DataTmpUndefdAndEvlnInProgs: The data is temp. undefined, evaluation in progress
     2 = DevOfDataNotWithinRngAllwd: The accuracy of the data is not within specification
     3 = DataCalcdWithDevDefd: The data is calculated with the specified accuracy */
  uint8_T VDYN_EgoYawRate_Confidence;

  /* Ego vehicle max acceleration limitation based on vehicle dynamics. */
  real32_T AccelerationLongMaxLimit;

  /* Valid signal of max Ax. */
  boolean_T AccelerationLongMaxLimitValid;

  /* Ego vehicle min acceleration (deceleration)  limitation based on vehicle dynamics. */
  real32_T AccelerationLongMinLimit;

  /* Valid signal of min Ax. */
  boolean_T AccelerationLongMinLimitValid;

  /* Ego vehicle max lateral acceleration (deceleration)  limitation based on vehicle dynamics. */
  real32_T AccelerationLatMaxLimit;

  /* Valid signal of max Ay. */
  boolean_T AccelerationLatMaxLimitValid;

  /* Ego vehicle max longitudinal velocity limitation calculated by vehicle dynamics. */
  real32_T VelocityLongMaxLimit;

  /* Valid signal of max Vx. */
  boolean_T VelocityLongMaxLimitValid;

  /* Ego vehicle max yaw rate limitation based on vehicle dynamics. */
  real32_T YawRateMaxLimit;

  /* Valid signal of max yaw rate. */
  boolean_T YawRateMaxLimitValid;
  VDYN_ModelTransForTruck_DataBus ModelTransForTruck;
  VDYN_TransformDataBus TransformData;
  VDYN_TransformDataBus_50ms TransformData_50ms;

  /* The yaw rate sensor zero position value. Add COMM yaw rate value to this value to get the correction value. */
  real32_T VDYN_YawRateOffset;
  boolean_T VDYN_YawRateOffsetValid;

  /* Total vehicle mass estimated by VDYN. */
  real32_T VDYN_EstVehicleTotalMass;
  boolean_T VDYN_EstVehicleTotalMassValid;

  /* Truck load ratio. */
  real32_T VDYN_TruckLoadRatio;
  boolean_T VDYN_TruckLoadRatioValid;

  /* Determine whether the truck has a semitrailer at this time. */
  boolean_T VDYN_EstHasSemiTrailer;

  /* Judge whether the trailer signal is reliable. */
  boolean_T VDYN_EstHasSemiTrailerValid;

  /* UTM slope angle. */
  real32_T VDYN_UTMSlopeAngle;
  boolean_T VDYN_UTMSlopeAngleValid;

  /* UTM Roll Angle. */
  real32_T VDYN_UTMRollAngle;
  boolean_T VDYN_UTMRollAngleValid;

  /* Roll angle in road coordinate system. */
  real32_T VDYN_RCSRollAngle;
  boolean_T VDYN_RCSRollAngleValid;

  /* Estimated road lateral slope angle. */
  real32_T VDYN_EstRoadLateralSlopeAngle;
  boolean_T VDYN_EstRoadLateralSlopeAngleValid;
}
VDYN_DataBus;

#endif

/* Custom Type definition for MATLAB Function: '<S268>/VDYN_H_MatrixWOT' */
#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif                                 /*struct_tag_skA4KFEZ4HPkJJBOYCrevdH*/

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH_VDYN__T
#define typedef_skA4KFEZ4HPkJJBOYCrevdH_VDYN__T

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH_VDYN__T;

#endif                                 /*typedef_skA4KFEZ4HPkJJBOYCrevdH_VDYN__T*/

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 /*struct_tag_sJCxfmxS8gBOONUZjbjUd9E*/

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_VDYN__T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_VDYN__T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E_VDYN__T;

#endif                                 /*typedef_sJCxfmxS8gBOONUZjbjUd9E_VDYN__T*/
#endif                                 /* RTW_HEADER_VDYN_subsystem_integrated_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
