#pragma once

#include "rtwtypes.h"

// struct Vector2f
// {
//     float x;
//     float y;
// };

// struct Vector3f
// {
//     float x;
//     float y;
//     float z;
// };

typedef struct
{
  /* position_covariance of x */
  real32_T x;

  /* position covariance of y */
  real32_T y;
} Vector2f;

typedef struct
{
  /* position_covariance of x */
  real32_T x;

  /* position covariance of y */
  real32_T y;

/* position covariance of y */
  real32_T z;
} Vector3f;