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

#include "sanim.h"
#include "test_sanim_utils.h"

#include <rsys/double33.h>

int
main(int argc, char** argv)
{
  struct mem_allocator allocator;
  struct my_type t1, t2, t3;
  struct sanim_pivot pivot;
  struct sanim_tracking tracking;
  double transform[12];
  double transl[3], rot[3], in_dir[3], n[3], tmp[3];
  const double y_ref_normal[3] = { 0, 1, 0 };
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);

  CHECK(my_type_init_pivot(&allocator, NULL, &tracking, &t1), RES_BAD_ARG);
  CHECK(my_type_init_pivot(&allocator, &pivot, NULL, &t1), RES_BAD_ARG);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, NULL), RES_BAD_ARG);

  /*
   * 1 axis pivots
   * (in/out dirs can be off the YZ plane)
   */

  /* 1 axis tracking sun */

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);

  /* ref_normal not in the YZ plane */
  d3(pivot.data.pivot1.ref_normal, 1, 0, 1);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t1), RES_BAD_ARG);
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t1), RES_OK);
  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_init_pivot(NULL, &pivot, &tracking, &t1), RES_OK);
  CHECK(my_type_release(&t1), RES_OK);

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  d3_splat(transl, +1);
  d3(rot, 0, 0, PI/2);
  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 0, 0.99, -0.1);
  /* rotation axis is Y after positioning: cannot accomodate in_dir */
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_BAD_ARG);

  d3(in_dir, 1, 0.2, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, pivot.data.pivot1.ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -sqrt(2), 3, 2), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  /* 1 axis tracking with a fixed output dir */

  tracking.policy = TRACKING_OUT_DIR;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(tracking.data.out_dir.u, 0, 1, 0);

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);
  
  d3(in_dir, 0, -1, -0.1);
  /* rotation axis is Y after positioning: cannot accomodate <in_dir,out_dir> */
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_BAD_ARG);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_OUT_DIR;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(tracking.data.out_dir.u, 1, 0, 1);

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, -0.3, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, pivot.data.pivot1.ref_normal);
  CHECK(d3_eq_eps(pivot.data.pivot1.ref_normal, n, 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(n, -1, 3, 3), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  /* 1 axis tracking a target point */

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 0);
  d3(tracking.data.point.target, 0, 0, 30);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, 0.1, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, pivot.data.pivot1.ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 10 * sqrt(2));
  d3(tracking.data.point.target, 0, 10, 30);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, 0.1, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, pivot.data.pivot1.ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  /* same 1 axis tracking with a non-local target point */

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 10 * sqrt(2));
  d3(tracking.data.point.target, -10, 2, 32);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, -0.5, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, pivot.data.pivot1.ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 5, 5);
  d3(tracking.data.point.target, -12, 2, -10);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, pivot.data.pivot1.ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -1, 0, 0), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -1, 3, 1), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  /* 
   * 2 axis pivots
   * (using only one axis at a time)
   */

  /* 2 axis tracking sun */

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 0, 0, 1);

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  d3_splat(transl, +1);
  d3(rot, 0, 0, PI / 2);
  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -1, 3, 2 + sqrt(2)), 1e-10), 1);

  d3(in_dir, 0, 1, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, 0, -1, 0), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -1, 0, 3), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);
  
  /* 2 axis tracking sun */

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  d3_splat(transl, +1);
  d3(rot, 0, 0, PI / 2);
  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 1, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -1, 3, 2 + sqrt(2)), 1e-10), 1);

  d3(in_dir, 0, 1, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, 0, -1, 0), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -1, 0, 3), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  /* 2 axis tracking with a fixed output dir */

  tracking.policy = TRACKING_OUT_DIR;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(tracking.data.out_dir.u, 0, 1, 0);
  
  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);
  
  d3(in_dir, 0, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, 0, sqrt(2) / 2, sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, 1, 3, 2 + sqrt(2)), 1e-10), 1);

  d3(in_dir, -1, 0, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, sqrt(2) / 2, sqrt(2) / 2, 0), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, 3 * sqrt(2) / 2, 2 + sqrt(2) / 2, 3), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  /* 2 axis tracking a target point */

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 0;
  d3(pivot.data.pivot2.ref_point, 0, 0, 0);
  d3(tracking.data.point.target, 30, 0, 0);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, -1, 0, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, sqrt(2) / 2, sqrt(2) / 2, 0), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, sqrt(2), 2, 3), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = sqrt(2);
  d3(pivot.data.pivot2.ref_point, 0, 10 * sqrt(2), 0);
  d3(tracking.data.point.target, 30, -11, 0);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, -1, 0, 0);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, sqrt(2) / 2, sqrt(2) / 2, 0), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, 1 + sqrt(2), 3, 3), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 0, 10 * sqrt(2), 0);
  d3(tracking.data.point.target, 0, 30, 10);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, -sqrt(2) / 2, 0, +sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, -1, 3, 2 + sqrt(2)), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 0, 10 * sqrt(2), 0);
  d3(tracking.data.point.target, 0, 30, 12);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, 0, sqrt(2) / 2, sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, 1, 3, 2 + sqrt(2)), 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 10, 8 * sqrt(2), 5 * sqrt(2));
  d3(tracking.data.point.target, 10, 12, 15);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);
  CHECK(my_type_set_translation(&t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  CHECK(sanim_node_solve_pivot(&t2.node, in_dir), RES_OK);
  CHECK(my_type_get_transform(&t3, transform), RES_OK);
  d33_muld3(n, transform, y_ref_normal);
  CHECK(d3_eq_eps(n, d3(tmp, 0, sqrt(2) / 2, sqrt(2) / 2), 1e-10), 1);
  CHECK(d3_eq_eps(transform + 9, d3(tmp, 1, 3, 2 + sqrt(2)), 1e-10), 1);

  /* release memory */
  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);
  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
