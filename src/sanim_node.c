/* Copyright (C) CNRS 2016
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>. */

#include "sanim_node_c.h"
#include "sanim_device_c.h"
#include "sanim.h"

#include <rsys/mem_allocator.h>
#include <rsys/ref_count.h>
#include <rsys/double33.h>
#include <rsys/double22.h>

#include <math.h>

/*******************************************************************************
* Helper functions
******************************************************************************/
static int
is_ancestor
  (const struct sanim_node* node, const struct sanim_node* possible_ancestor)
{
  ASSERT(node && node->data && possible_ancestor);
  while (node) {
    if (node == possible_ancestor) return 1;
    node = node->data->father;
  }
  return 0;
}

static int
is_after_pivot(const struct sanim_node* node) {
  ASSERT(node);
  while (node) {
    if (node->data->pivot_data) return 1;
    node = node->data->father;
  }
  return 0;
}

static void
d34_muld34(double dst[12], const double a[12], const double b[12]) {
  ASSERT(dst && a && b);
  double tmp[3];
  d3_add(dst + 9, d33_muld3(tmp, a, b + 9), a + 9);
  d33_muld33(dst, a, b);
}

static void
d34_set_identity(double dst[12]) {
  ASSERT(dst);
  d33_set_identity(dst);
  d3_splat(dst + 9, 0);
}

static INLINE double*
d22_rotation(double dst[4], const double angle/* in radian */)
{
  const double c = (double) cos((double) angle);
  const double s = (double) sin((double) angle);
  ASSERT(dst);
  dst[0] = c; dst[1] = s;
  dst[2] = -s; dst[3] = c;
  return dst;
}

static double*
get_Xpivot_transform
  (const double angle,
   const double spacing,
   double transform[12])
{
  ASSERT(transform);
  d33_rotation_pitch(transform, angle);
  d3(transform + 9, 0, spacing, 0);
  return transform;
}

static double*
compose_Xpivot_transform
  (const double angle,
   const double spacing,
   double accum[12])
{
  double pivot[12];
  ASSERT(accum);
  get_Xpivot_transform(angle, spacing, pivot);
  d34_muld34(accum, pivot, accum);
  return accum;
}

static double*
get_Zpivot_transform(const double angle, double transform[12]) {
  ASSERT(transform);
  d33_rotation_roll(transform, angle);
  d3_splat(transform + 9, 0);
  return transform;
}

static double*
compose_Zpivot_transform(const double angle, double accum[12]) {
  double pivot[12];
  ASSERT(accum);
  get_Zpivot_transform(angle, pivot);
  d34_muld34(accum, pivot, accum);
  return accum;
}

static double*
get_ZXpivot_transform
  (const double angleZ,
   const double angleX,
   const double spacing,
   double transform[12])
{
  ASSERT(transform);
  get_Xpivot_transform(angleX, spacing, transform);
  compose_Zpivot_transform(angleZ, transform);
  return transform;
}

static double*
compose_ZXpivot_transform
  (const double angleZ,
   const double angleX,
   const double spacing,
   double accum[12])
{
  ASSERT(accum);
  compose_Xpivot_transform(angleX, spacing, accum);
  compose_Zpivot_transform(angleZ, accum);
  return accum;
}

static double*
compose_pivot_transform(const struct pivot_data* pivot, double accum[12]) {
  ASSERT(pivot && accum);
  switch (pivot->pivot.type) {
  case PIVOT_SINGLE_AXIS: {
    ASSERT(pivot->angleZ == 0);
    compose_Xpivot_transform(pivot->angleX, 0, accum);
    break;
  }
  case PIVOT_TWO_AXIS: {
    compose_ZXpivot_transform(
        pivot->angleZ, pivot->angleX, pivot->pivot.data.pivot2.spacing, accum);
    break;
  }
  default: FATAL("Unreachable code.\n"); break;
  }
  return accum;
}

static double*
get_pivot_transform(const struct pivot_data* pivot, double transform[12]) {
  ASSERT(pivot && transform);
  switch (pivot->pivot.type) {
  case PIVOT_SINGLE_AXIS: {
    ASSERT(pivot->angleZ == 0);
    get_Xpivot_transform(pivot->angleX, 0, transform);
    break;
  }
  case PIVOT_TWO_AXIS: {
    get_ZXpivot_transform(
      pivot->angleZ, pivot->angleX, pivot->pivot.data.pivot2.spacing, transform);
    break;
  }
  default: FATAL("Unreachable code.\n"); break;
  }
  return transform;
}

static double*
node_get_own_transform
  (struct sanim_node* node,
   const int include_pivot,
   double transform[12])
{
  ASSERT(node && node->data && transform);
  if (include_pivot && node->data->pivot_data) {
    double local[12];
    get_pivot_transform(node->data->pivot_data, transform);
    d33_rotation(local, SPLIT3(node->data->rotations));
    d3_set(local + 9, node->data->translation);
    d34_muld34(transform, local, transform);
  }
  else {
    d33_rotation(transform, SPLIT3(node->data->rotations));
    d3_set(transform + 9, node->data->translation);
  }
  return transform;
}

static double*
compose_node_transform(struct sanim_node* node, double accum[12]) {
  double local[12];
  ASSERT(node && node->data && accum);
  if (node->data->pivot_data) {
    compose_pivot_transform(node->data->pivot_data, accum);
  }
  d33_rotation(local, SPLIT3(node->data->rotations));
  d3_set(local + 9, node->data->translation);
  d34_muld34(accum, local, accum);
  return accum;
}

static void
node_get_transform
  (struct sanim_node* node,
   const int include_own_pivot,
   double transform[12])
{
  struct sanim_node* father;
  ASSERT(node && node->data && transform);
  father = node->data->father;
  node_get_own_transform(node, include_own_pivot, transform);
  while (father) {
    compose_node_transform(father, transform);
    father = father->data->father;
  }
}

static void
compute_single_axis_angle
  (const double ref_2D[2],
   const double rotated_2D[2],
   double* angle )
{
  double x, y;
  ASSERT(ref_2D && rotated_2D && angle);
  ASSERT(d2_is_normalized(rotated_2D));
  ASSERT(d2_is_normalized(ref_2D));
  /* in the YZ plane */
  y = d2_cross(ref_2D, rotated_2D);
  x = d2_dot(ref_2D, rotated_2D);
  *angle = atan2(y, x);
}

static res_T
pivot_solve_single_axis_sun
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3];
  double local_in_2D[2], rotated_n_2D[2];
  const double* ref_normal_2D;
  struct pivot_data* pivot_data;
  ASSERT(node && node->data && in_dir);
  pivot_data = node->data->pivot_data;
  ASSERT(pivot_data);
  ASSERT(pivot_data->pivot.type == PIVOT_SINGLE_AXIS);
  ASSERT(pivot_data->tracking.policy == TRACKING_SUN);
  ASSERT(d3_is_normalized(in_dir));

  ref_normal_2D = pivot_data->pivot.data.pivot1.ref_normal + 1;
  ASSERT(d2_is_normalized(ref_normal_2D));
  
  /* get in_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);

  /* solve in the YZ plane */
  if (d2_normalize(local_in_2D, local_in + 1) < 0.25) {
    /* not really in the YZ-plane */
    return RES_BAD_ARG;
  }

  /* rotated_n = -local_in */
  d2_muld(rotated_n_2D, local_in_2D, -1);

  compute_single_axis_angle(ref_normal_2D, rotated_n_2D, &pivot_data->angleX);
  return RES_OK;
}

FINLINE res_T
pivot_solve_single_axis_line
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[9];
  double local_in[3], local_target[3];
  double rotated_n_2D[2], local_out_2D[2], local_in_2D[2], ref_point_2D[2];
  const double* ref_normal_2D;
  double* const local_target_2D = local_target + 1;
  struct pivot_data* pivot_data;
  double angle, previous_angle, delta;
  double sign_dA, prev_sign_dA;
  double kA;
  double d1, d2;
  int cpt = 0;
  ASSERT(node && node->data && in_dir);
  pivot_data = node->data->pivot_data;
  ASSERT(pivot_data);
  ASSERT(pivot_data->pivot.type == PIVOT_SINGLE_AXIS);
  ASSERT(pivot_data->tracking.policy == TRACKING_POINT);
  ASSERT(d3_is_normalized(in_dir));

  ref_normal_2D = pivot_data->pivot.data.pivot1.ref_normal + 1;
  ASSERT(pivot_data->pivot.data.pivot1.ref_normal[0] == 0); /* solve in YZ plane */
  ASSERT(d2_is_normalized(ref_normal_2D));
  d2_set(ref_point_2D, pivot_data->pivot.data.pivot1.ref_point + 1);

  /* get in_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);
  /* solve in the YZ plane */
  if (d2_normalize(local_in_2D, local_in + 1) < 0.25) {
    /* not really in the YZ-plane */
    return RES_BAD_ARG;
  }

  /* get target point in local space */
  if (pivot_data->tracking.data.point.target_is_local) {
    d3_set(local_target, pivot_data->tracking.data.point.target);
  }
  else {
    d3_sub(local_target, pivot_data->tracking.data.point.target, mat + 9);
    d33_muld3(local_target, inv, local_target);
  }

  /* check if in, target_point and ref_point are compatible */
  d1 = d2_dot(local_target_2D, local_target_2D); /* in the YZ plane */
  d2 = d2_dot(ref_point_2D, ref_point_2D);
  if (d1 <= d2) {
    /* target in the pivot */
    return RES_BAD_ARG;
  }

  angle = 0;
  prev_sign_dA = 0;
  kA = 0.9;
  do {
    double pivot[4];
    /* compute 2D normal after rotation */
    d2_sub(local_out_2D, local_target_2D, ref_point_2D);
    if (d2_normalize(local_out_2D, local_out_2D) < 0.25) {
      /* not really in the YZ-plane */
      return RES_BAD_ARG;
    }

    /* rotated_n = bisectrix of local_in and out_dir */
    d2_sub(rotated_n_2D, local_out_2D, local_in_2D);
    if (d2_normalize(rotated_n_2D, rotated_n_2D) < 1e-4) {
      /* tangent rays */
      return RES_BAD_ARG;
    }

    previous_angle = angle;
    compute_single_axis_angle(ref_normal_2D, rotated_n_2D, &angle);
    if (fabs(previous_angle - angle) > PI) {
      previous_angle = (angle > 0) ? 2 * PI : -2 * PI;
    }
    
    delta = previous_angle - angle;
    if (fabs(delta) < 1e-7 || ++cpt > 10)
      break;

    if (d2) {
      /* only if ref_point is not the rotation point
      * the heuristic is to amortize algorithm's oscillations */
      sign_dA = sign(previous_angle - angle);
      if (prev_sign_dA != sign_dA)
        kA *= 0.9;
      else
        kA *= 1 / 0.9;
      angle = previous_angle + kA * (angle - previous_angle);
      prev_sign_dA = sign_dA;
    }

    /* update ref_point */
    d22_rotation(pivot, angle);
    d22_muld2(ref_point_2D, pivot, pivot_data->pivot.data.pivot1.ref_point + 1);
    /* no d3_add(ref_point, ref_point, pivot + 9) as pivot has no offset to add */
  } while (1);
  
  pivot_data->angleX = angle;
  return RES_OK;
}

FINLINE res_T
pivot_solve_single_axis_dir
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3], local_out[3];
  double local_in_2D[2], rotated_n_2D[2], local_out_2D[2];
  const double* ref_normal_2D;
  struct pivot_data* pivot_data;
  ASSERT(node && node->data && in_dir);
  pivot_data = node->data->pivot_data;
  ASSERT(pivot_data);
  ASSERT(pivot_data->pivot.type == PIVOT_SINGLE_AXIS);
  ASSERT(pivot_data->tracking.policy == TRACKING_OUT_DIR);
  ASSERT(d3_is_normalized(in_dir));
  ASSERT(d3_is_normalized(pivot_data->tracking.data.out_dir.u));

  ref_normal_2D = pivot_data->pivot.data.pivot1.ref_normal + 1;
  ASSERT(pivot_data->pivot.data.pivot1.ref_normal[0] == 0); /* solve in YZ plane */
  ASSERT(d2_is_normalized(ref_normal_2D));
  
  /* get in_dir and out_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);
  d33_muld3(local_out, inv, pivot_data->tracking.data.out_dir.u);

  /* solve in the YZ plane */
  if (d2_normalize(local_in_2D, local_in + 1) < 0.25) {
    /* not really in the YZ-plane */
    return RES_BAD_ARG;
  }
  if (d2_normalize(local_out_2D, local_out + 1) < 0.25) {
    /* not really in the YZ-plane */
    return RES_BAD_ARG;
  }

  /* rotated_n = bisectrix of local_in and out_dir */
  d2_sub(rotated_n_2D, local_out_2D, local_in_2D);
  if (d2_normalize(rotated_n_2D, rotated_n_2D) < 1e-4) {
    /* tangent rays */
    return RES_BAD_ARG;
  }

  compute_single_axis_angle(ref_normal_2D, rotated_n_2D, &pivot_data->angleX);
  return RES_OK;
}

FINLINE res_T
pivot_solve_single_axis
  (struct sanim_node* node,
   const double in_dir[3])
{
  res_T res = RES_OK;
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_SINGLE_AXIS);

  switch (node->data->pivot_data->tracking.policy) {
  case TRACKING_SUN:
    res = pivot_solve_single_axis_sun(node, in_dir);
    break;
  case TRACKING_POINT:
    /* track the X line that includes ref_point */
    res = pivot_solve_single_axis_line(node, in_dir);
    break;
  case TRACKING_OUT_DIR:
    res = pivot_solve_single_axis_dir(node, in_dir);
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
  ASSERT(node->data->pivot_data->angleZ == 0);
  return res;
}

static void
compute_two_axis_angles
  (const double rotated_n[3],
   double* angleX,
   double* angleZ)
{
  /* ref normal is <0,1,0> */
  ASSERT(rotated_n && angleX && angleZ);
  ASSERT(d3_is_normalized(rotated_n));
  if (fabs(rotated_n[2]) >= 1) {
    *angleX = 0.5 * sign(rotated_n[2]) * PI;
    *angleZ = 0;
    return;
  }
  *angleX = asin(rotated_n[2]);
  *angleZ = atan2(-rotated_n[0], rotated_n[1]);
}

FINLINE res_T
pivot_solve_two_axis_sun
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3], rotated_n[3];
  struct pivot_data* pivot_data;
  ASSERT(node && node->data && in_dir);
  pivot_data = node->data->pivot_data;
  ASSERT(pivot_data);
  ASSERT(pivot_data->pivot.type == PIVOT_TWO_AXIS);
  ASSERT(pivot_data->tracking.policy == TRACKING_SUN);
  ASSERT(d3_is_normalized(in_dir));

  /* get in_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);
  ASSERT(d3_is_normalized(local_in));

  /* rotated_n = -local_in */
  d3_muld(rotated_n, local_in, -1);

  compute_two_axis_angles(rotated_n, &pivot_data->angleX, &pivot_data->angleZ);
  return RES_OK;
}

FINLINE res_T
pivot_solve_two_axis_point
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[9];
  double local_in[3], rotated_n[3], local_out[3], local_target[3], ref_point[3];
  double angleX, angleZ, previous_angleX, previous_angleZ, delta;
  double sign_dX, sign_dZ, prev_sign_dX, prev_sign_dZ;
  double kX, kZ;
  double d1, d2;
  struct pivot_data* pivot_data;
  int cpt = 0;
  ASSERT(node && node->data && in_dir);
  pivot_data = node->data->pivot_data;
  ASSERT(pivot_data);
  ASSERT(pivot_data->pivot.type == PIVOT_TWO_AXIS);
  ASSERT(pivot_data->tracking.policy == TRACKING_POINT);
  ASSERT(d3_is_normalized(in_dir));

  d3_set(ref_point, pivot_data->pivot.data.pivot2.ref_point);
  ref_point[1] += pivot_data->pivot.data.pivot2.spacing;

  /* get in_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);
  ASSERT(d3_is_normalized(local_in));

  /* get target point in local space */
  if (pivot_data->tracking.data.point.target_is_local) {
    d3_set(local_target, pivot_data->tracking.data.point.target);
  }
  else {
    d3_sub(local_target, pivot_data->tracking.data.point.target, mat + 9);
    d33_muld3(local_target, inv, local_target);
  }

  /* check if in, target_point and ref_point are compatible */
  d1 = d3_dot(local_target, local_target);
  d2 = d3_dot(ref_point, ref_point);
  if (d1 <= d2) {
    /* target in the pivot */
    return RES_BAD_ARG;
  }

  angleX = angleZ = 0;
  prev_sign_dX = prev_sign_dZ = 0;
  kX = kZ = 0.9;
  do {
    double pivot[12];
    /* compute rotated_n */
    d3_sub(local_out, local_target, ref_point);
    d3_normalize(local_out, local_out);

    /* rotated_n = bisectrix of local_in and out_dir */
    d3_sub(rotated_n, local_out, local_in);
    if (d3_normalize(rotated_n, rotated_n) < 1e-4) {
      /* tangent rays */
      return RES_BAD_ARG;
    }

    previous_angleX = angleX;
    previous_angleZ = angleZ;
    compute_two_axis_angles(rotated_n, &angleX, &angleZ);
    if (fabs(previous_angleX - angleX) > PI) {
      previous_angleX = (angleX > 0) ? 2 * PI : -2 * PI;
    }
    if (fabs(previous_angleZ - angleZ) > PI) {
      previous_angleZ += (angleZ > 0) ? 2 * PI : -2 * PI;
    }
    delta = MMAX(fabs(previous_angleX - angleX), fabs(previous_angleZ - angleZ));
    if (delta < 1e-7 || ++cpt > 10)
      break;

    if (d2) {
      /* only if ref_point is not the rotation point
       * the heuristic is to amortize algorithm's oscillations */
      sign_dX = sign(previous_angleX - angleX);
      sign_dZ = sign(previous_angleZ - angleZ);
      if (prev_sign_dX != sign_dX)
        kX *= 0.9;
      else
        kX *= 1 / 0.9;
      angleX = previous_angleX + kX * (angleX - previous_angleX);
      if (prev_sign_dZ != sign_dZ)
        kZ *= 0.9;
      else
        kZ *= 1 / 0.9;
      angleZ = previous_angleZ + kZ * (angleZ - previous_angleZ);
      prev_sign_dX = sign_dX;
      prev_sign_dZ = sign_dZ;
    }

    get_ZXpivot_transform(
      angleZ, angleX, pivot_data->pivot.data.pivot2.spacing, pivot);
    /* update ref_point */
    d33_muld3(ref_point, pivot, pivot_data->pivot.data.pivot2.ref_point);
    d3_add(ref_point, ref_point, pivot + 9);
  } while (1);

  pivot_data->angleX = angleX;
  pivot_data->angleZ = angleZ;
  return RES_OK;
}

FINLINE res_T
pivot_solve_two_axis_dir
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3], rotated_n[3], local_out[3];
  struct pivot_data* pivot_data;
  ASSERT(node && node->data && in_dir);
  pivot_data = node->data->pivot_data;
  ASSERT(pivot_data);
  ASSERT(pivot_data->pivot.type == PIVOT_TWO_AXIS);
  ASSERT(pivot_data->tracking.policy == TRACKING_OUT_DIR);
  ASSERT(d3_is_normalized(in_dir));
  ASSERT(d3_is_normalized(pivot_data->tracking.data.out_dir.u));

  /* get in_dir and out_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);
  d33_muld3(local_out, inv, pivot_data->tracking.data.out_dir.u);
  
  /* rotated_n = bisectrix of local_in and out_dir */
  d3_sub(rotated_n, local_out, local_in);
  if (d3_normalize(rotated_n, rotated_n) < 1e-4) {
    /* tangent rays */
    return RES_BAD_ARG;
  }

  compute_two_axis_angles(rotated_n, &pivot_data->angleX, &pivot_data->angleZ);
  return RES_OK;
}

FINLINE res_T
pivot_solve_two_axis
  (struct sanim_node* node,
   const double in_dir[3])
{
  res_T res = RES_OK;
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_TWO_AXIS);

  switch (node->data->pivot_data->tracking.policy) {
  case TRACKING_SUN:
    res = pivot_solve_two_axis_sun(node, in_dir);
    break;
  case TRACKING_POINT:
    res = pivot_solve_two_axis_point(node, in_dir);
    break;
  case TRACKING_OUT_DIR:
    res = pivot_solve_two_axis_dir(node, in_dir);
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
  return res;
}

FINLINE res_T
copy_and_normalise_pivot_data
  (struct pivot_data* dest,
   const struct sanim_pivot* pivot,
   const struct sanim_tracking* tracking)
{
  dest->pivot.type = pivot->type;
  switch (pivot->type) {
  case PIVOT_SINGLE_AXIS:
    if (!d3_normalize(dest->pivot.data.pivot1.ref_normal, pivot->data.pivot1.ref_normal))
      return RES_BAD_ARG;
    if (dest->pivot.data.pivot1.ref_normal[0])
      /* ref_normal not in the YZ plane */
      return RES_BAD_ARG;
    d3_set(dest->pivot.data.pivot1.ref_point, pivot->data.pivot1.ref_point);
    break;
  case PIVOT_TWO_AXIS:
    if (pivot->data.pivot2.spacing < 0)
      return RES_BAD_ARG;
    d3_set(dest->pivot.data.pivot2.ref_point, pivot->data.pivot2.ref_point);
    dest->pivot.data.pivot2.spacing = pivot->data.pivot2.spacing;
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
  dest->tracking.policy = tracking->policy;
  switch (tracking->policy) {
  case TRACKING_SUN:
    /* nothing to be copied */
    break;
  case TRACKING_POINT:
    d3_set(dest->tracking.data.point.target, tracking->data.point.target);
    dest->tracking.data.point.target_is_local = tracking->data.point.target_is_local;
    break;
  case TRACKING_OUT_DIR:
    if (!d3_normalize(dest->tracking.data.out_dir.u, tracking->data.out_dir.u))
      return RES_BAD_ARG;
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
  return RES_OK;
}

/*******************************************************************************
* Exported ssol_spectrum functions
******************************************************************************/
res_T
sanim_node_add_child
  (struct sanim_node* father,
   struct sanim_node* child)
{
  res_T res = RES_OK;

  if (!father || !child) return RES_BAD_ARG;
  if (child->data->father) return RES_BAD_ARG;
  if (is_ancestor(father, child)) return RES_BAD_ARG;
  if (child->data->pivot_data && is_after_pivot(father)) return RES_BAD_ARG;

  child->data->father = father;
  res = darray_children_push_back(&father->data->children, &child);
  if (res != RES_OK) {
    goto error;
  }

exit:
  return res;
error:
  if (child->data) {
    child->data = NULL;
  }
  goto exit;
}

res_T
sanim_node_initialize
  (struct mem_allocator* allocator,
   struct sanim_node* node)
{
  struct mem_allocator* alloc;
  res_T res = RES_OK;

  if (!node) return RES_BAD_ARG;
  alloc = allocator ? allocator : &mem_default_allocator;

  node->data = MEM_CALLOC(alloc, 1, sizeof(struct node_data));
  if (!node->data) {
    res = RES_MEM_ERR;
    goto error;
  }

  darray_children_init(alloc, &node->data->children);
  node->data->allocator = alloc;

exit:
  return res;
error:
  if (node->data) {
    darray_children_release(&node->data->children);
    node->data = NULL;
  }
  goto exit;
}

res_T
sanim_node_initialize_pivot
  (struct mem_allocator* allocator,
   const struct sanim_pivot* pivot,
   const struct sanim_tracking* tracking,
   struct sanim_node* node)
{
  struct mem_allocator* alloc;
  res_T res = RES_OK;

  if (!node || !pivot || !tracking) return RES_BAD_ARG;
  res = sanim_node_initialize(allocator, node);
  if (res != RES_OK) goto error;

  alloc = allocator ? allocator : &mem_default_allocator;
  node->data->pivot_data = MEM_CALLOC(alloc, 1, sizeof(struct pivot_data));
  if (!node->data->pivot_data) {
    res = RES_MEM_ERR;
    goto error;
  }
  
  res = copy_and_normalise_pivot_data(node->data->pivot_data, pivot, tracking);
  if (res != RES_OK) goto error;

exit:
  return res;
error:
  sanim_node_release(node);
  goto exit;
}

res_T
sanim_node_solve_pivot
  (struct sanim_node* node,
   const double in_dir[3])
{
  double dir[3];
  if (!node || !in_dir) return RES_BAD_ARG;
  if (!node->data->pivot_data) return RES_BAD_ARG;
  if (!d3_normalize(dir, in_dir)) return RES_BAD_ARG;

  switch (node->data->pivot_data->pivot.type) {
  case PIVOT_SINGLE_AXIS:
    return pivot_solve_single_axis(node, dir);
    break;
  case PIVOT_TWO_AXIS:
    return pivot_solve_two_axis(node, dir);
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
}

res_T
sanim_node_release
  (struct sanim_node* node)
{
  if (!node) return RES_BAD_ARG;
  if (node->data) {
    darray_children_release(&node->data->children);
    if (node->data->pivot_data) {
      MEM_RM(node->data->allocator, node->data->pivot_data);
    }
    MEM_RM(node->data->allocator, node->data);
  }
  return RES_OK;
}

res_T
sanim_node_set_translation
  (struct sanim_node* node,
   const double translation[3])
{
  if (!node || !node->data || !translation) return RES_BAD_ARG;
  d3_set(node->data->translation, translation);
  return RES_OK;
}

res_T
sanim_node_set_rotations
  (struct sanim_node* node,
   const double rotations[3])
{
  if (!node || !node->data || !rotations) return RES_BAD_ARG;
  d3_set(node->data->rotations, rotations);
  return RES_OK;
}

res_T
sanim_node_get_transform(struct sanim_node* node, double transform[12])
{
  if (!node || !node->data || !transform)
    return RES_BAD_ARG;
  node_get_transform(node, 1, transform);
  return RES_OK;
}