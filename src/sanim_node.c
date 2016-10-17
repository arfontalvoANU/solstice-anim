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

static double*
set_node_transform(struct sanim_node* node, const char include_pivot, double transform[12]) {
  double tmp[12];
  ASSERT(node && node->data && transform);
  if (include_pivot && node->data->pivot_data) {
    d33_rotation(transform, SPLIT3(node->data->pivot_data->pivot_angles));
    d3_splat(transform + 9, 0);
    d33_rotation(tmp, SPLIT3(node->data->rotations));
    d3_set(tmp + 9, node->data->translation);
    d34_muld34(transform, tmp, transform);
  }
  else {
    d33_rotation(transform, SPLIT3(node->data->rotations));
    d3_set(transform + 9, node->data->translation);
  }
  return transform;
}

static double*
compose_node_transform(struct sanim_node* node, double transform[12]) {
  double tmp[12];
  ASSERT(node && node->data && transform);
  if (node->data->pivot_data) {
    d33_rotation(tmp, SPLIT3(node->data->pivot_data->pivot_angles));
    d3_splat(tmp + 9, 0);
    d34_muld34(transform, tmp, transform);
  }
  d33_rotation(tmp, SPLIT3(node->data->rotations));
  d3_set(tmp + 9, node->data->translation);
  d34_muld34(transform, tmp, transform);
  return transform;
}

static void
node_get_transform(struct sanim_node* node, const char include_own_pivot, double transform[12])
{
  struct sanim_node* ptr;
  ASSERT(node && node->data && transform);
  set_node_transform(node, include_own_pivot, transform);
  ptr = node->data->father;
  while (ptr) {
    compose_node_transform(ptr, transform);
    ptr = ptr->data->father;
  }
}

static res_T
compute_single_axis_angle
(const double ref_normal[3],
  const double rotated_n[3],
  double* angle )
{
  ASSERT(ref_normal && rotated_n && angle);
  ASSERT(d3_is_normalized(rotated_n));
  ASSERT(d3_is_normalized(ref_normal));
  ASSERT(ref_normal[0] == 0); /* ref_normal in the YZ plane */
  if (fabs(rotated_n[0]) > 0.9) {
    /* cannot be obtained by X-rotate */
    return RES_BAD_ARG;
  }

  *angle =
    atan2(-(ref_normal[2] * rotated_n[1] - ref_normal[1] * rotated_n[2]),
      ref_normal[1] * rotated_n[1] + ref_normal[2] * rotated_n[2]);

  return RES_OK;
}

static res_T
pivot_solve_single_axis_sun
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3], rotated_n[3];
  const double* ref_normal;
  double* angle;
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_SINGLE_AXIS);
  ASSERT(node->data->pivot_data->tracking.policy == TRACKING_SUN);
  ASSERT(d3_is_normalized(in_dir));

  ref_normal = node->data->pivot_data->pivot.data.pivot1.ref_normal;
  ASSERT(d3_is_normalized(ref_normal));
  angle = &node->data->pivot_data->pivot_angles[0]; /* X-rotate */
  
  /* get in_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);

  /* rotated_n = -local_in */
  d3_muld(rotated_n, local_in, -1);

  return compute_single_axis_angle(ref_normal, rotated_n, angle);
}

FINLINE res_T
pivot_solve_single_axis_point
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3], rotated_n[3], local_target[3];
  const double* ref_normal;
  double* angle;
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_SINGLE_AXIS);
  ASSERT(node->data->pivot_data->tracking.policy == TRACKING_POINT);
  ASSERT(d3_is_normalized(in_dir));

  ref_normal = node->data->pivot_data->pivot.data.pivot1.ref_normal;
  ASSERT(d3_is_normalized(ref_normal));
  angle = &node->data->pivot_data->pivot_angles[0]; /* X-rotate */

  /* get in_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);

  /* get target point in local space */
  if (node->data->pivot_data->tracking.data.point.target_is_local) {
    d3_set(local_target, node->data->pivot_data->tracking.data.point.target);
  }
  else {
    d33_muld3(local_target, inv, node->data->pivot_data->tracking.data.point.target);
    d3_add(local_target, local_target, inv + 9);
  }

  /* compute rotated_n */


  return compute_single_axis_angle(ref_normal, rotated_n, angle);
}

FINLINE res_T
pivot_solve_single_axis_dir
  (struct sanim_node* node,
   const double in_dir[3])
{
  double mat[12], inv[12];
  double local_in[3], rotated_n[3], local_out[3];
  const double* ref_normal;
  double* angle;
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_SINGLE_AXIS);
  ASSERT(node->data->pivot_data->tracking.policy == TRACKING_OUT_DIR);
  ASSERT(d3_is_normalized(in_dir));
  ASSERT(d3_is_normalized(node->data->pivot_data->tracking.data.out_dir.u));

  ref_normal = node->data->pivot_data->pivot.data.pivot1.ref_normal;
  ASSERT(d3_is_normalized(ref_normal));
  angle = &node->data->pivot_data->pivot_angles[0]; /* X-rotate */

  /* get in_dir and out_dir in local space */
  node_get_transform(node, 0, mat);
  d33_transpose(inv, mat); /* no scale factors: inverse is transpose */
  d33_muld3(local_in, inv, in_dir);
  d33_muld3(local_out, inv, node->data->pivot_data->tracking.data.out_dir.u);

  /* compute rotated_n = bisectrix of local_in and out_dir */
  d3_sub(rotated_n, local_out, local_in);
  if (0 == d3_normalize(rotated_n, rotated_n)) {
    /* cannot be obtained by X-rotate */
    return RES_BAD_ARG;
  }

  return compute_single_axis_angle(ref_normal, rotated_n, angle);
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
    res = pivot_solve_single_axis_point(node, in_dir);
    break;
  case TRACKING_OUT_DIR:
    res = pivot_solve_single_axis_dir(node, in_dir);
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
  ASSERT(node->data->pivot_data->pivot_angles[1] == 0
    && node->data->pivot_data->pivot_angles[2] == 0);
  return res;
}

FINLINE res_T
pivot_solve_two_axis_sun
  (struct sanim_node* node,
   const double in_dir[3])
{
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_TWO_AXIS);
  ASSERT(node->data->pivot_data->tracking.policy == TRACKING_SUN);

  return RES_OK;
}

FINLINE res_T
pivot_solve_two_axis_point
  (struct sanim_node* node,
   const double in_dir[3])
{
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_TWO_AXIS);
  ASSERT(node->data->pivot_data->tracking.policy == TRACKING_POINT);

  return RES_OK;
}

FINLINE res_T
pivot_solve_two_axis_dir
  (struct sanim_node* node,
   const double in_dir[3])
{
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_TWO_AXIS);
  ASSERT(node->data->pivot_data->tracking.policy == TRACKING_OUT_DIR);

  return RES_OK;
}

FINLINE res_T
pivot_solve_two_axis
  (struct sanim_node* node,
   const double in_dir[3])
{
  ASSERT(node && in_dir);
  ASSERT(node->data->pivot_data);
  ASSERT(node->data->pivot_data->pivot.type == PIVOT_TWO_AXIS);

  switch (node->data->pivot_data->tracking.policy) {
  case TRACKING_SUN:
    return pivot_solve_two_axis_sun(node, in_dir);
  case TRACKING_POINT:
    return pivot_solve_two_axis_point(node, in_dir);
  case TRACKING_OUT_DIR:
    return pivot_solve_two_axis_dir(node, in_dir);
  default: FATAL("Unreachable code.\n"); break;
  }
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
    d3_set(dest->pivot.data.pivot2.ref_point, pivot->data.pivot2.ref_point);
    d3_set(dest->pivot.data.pivot2.spacing, pivot->data.pivot2.spacing);
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
  (struct sanim_node* node,
   struct sanim_node* child)
{
  res_T res = RES_OK;

  if (!node || !child) return RES_BAD_ARG;
  if (child->data->father) return RES_BAD_ARG;
  if (is_ancestor(node, child)) return RES_BAD_ARG;
  if (child->data->pivot_data && is_after_pivot(node)) return RES_BAD_ARG;

  child->data->father = node;
  res = darray_children_push_back(&node->data->children, &child);
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
    pivot_solve_two_axis(node, dir);
    break;
  default: FATAL("Unreachable code.\n"); break;
  }
  return RES_OK;
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