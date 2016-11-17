
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

#include <rsys/hash_table.h>
struct transform { double transform[12]; };
#define HTABLE_NAME transforms
#define HTABLE_KEY const struct sanim_node *
#define HTABLE_DATA struct transform
#include <rsys/hash_table.h>

static res_T
store_transform(const struct sanim_node* n, const double transform[12], void* data) {
  struct htable_transforms* transforms = data;
  struct transform tr;
  int i;
  for (i = 0; i < 12; i++) tr.transform[i] = transform[i];
  return htable_transforms_set(transforms, &n, &tr);
}

static void
check_visit
  (struct mem_allocator allocator,
   double in_dir[3],
   struct my_type *root,
   struct my_type *pivot,
   struct my_type *leaf)
{
  struct htable_transforms transforms;
  struct transform *tr;
  const struct sanim_node *key;
  double transform[12];

  htable_transforms_init(&allocator, &transforms);
  CHECK(sanim_node_visit_tree(
    &root->node, in_dir, &transforms, store_transform), RES_OK);
  CHECK(htable_transforms_size_get(&transforms), 3);
  key = &leaf->node;
  tr = htable_transforms_find(&transforms, &key);
  NCHECK(tr, NULL);
  CHECK(my_type_solve_pivot(pivot, in_dir), RES_OK);
  CHECK(my_type_get_transform(leaf, transform), RES_OK);
  key = &leaf->node;
  tr = htable_transforms_find(&transforms, &key);
  NCHECK(tr, NULL);
  d34_eq_eps(tr->transform, transform, 1e-10);
  CHECK(my_type_get_transform(pivot, transform), RES_OK);
  key = &pivot->node;
  tr = htable_transforms_find(&transforms, &key);
  NCHECK(tr, NULL);
  d34_eq_eps(tr->transform, transform, 1e-10);
  CHECK(my_type_get_transform(root, transform), RES_OK);
  key = &root->node;
  tr = htable_transforms_find(&transforms, &key);
  NCHECK(tr, NULL);
  d34_eq_eps(tr->transform, transform, 1e-10);
  htable_transforms_release(&transforms);
}

int
main(int argc, char** argv)
{
  struct mem_allocator allocator;
  struct my_type *t1, *t2, *t3, *target;
  struct sanim_pivot pivot = SANIM_PIVOT_NULL;
  struct sanim_tracking tracking = SANIM_TRACKING_NULL;
  double transl[3], rot[3], in_dir[3], tmp[3];
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);

  /*
  * 1 axis pivots
  * (in/out dirs can be off the YZ plane)
  */

  /* 1 axis tracking sun */

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);

  /* ref_normal not in the YZ plane */
  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  d3_splat(transl, +1);
  d3(rot, 0, 0, PI / 2);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);
  
  d3(in_dir, 1, 0.2, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  /* 1 axis tracking with a fixed output dir */
  
  tracking.policy = TRACKING_OUT_DIR;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(tracking.data.out_dir.u, 1, 0, 1);

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, -0.3, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  /* 1 axis tracking a target point */

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 0);
  d3(tracking.data.point.target, 0, 0, 30);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, 0.1, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 10 * sqrt(2));
  d3(tracking.data.point.target, 0, 10, 30);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, 0.1, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  /* same 1 axis tracking with a non-local target point */

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 10 * sqrt(2));
  d3(tracking.data.point.target, -10, 2, 32);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, -0.5, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 5, 5);
  d3(tracking.data.point.target, -12, 2, -10);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);
  
  /* 1 axis tracking a node target */

  CHECK(my_type_create(&allocator, &target), RES_OK);
  d3(tmp, 0, 0, 10 * sqrt(2));
  CHECK(my_type_set_translation(target, tmp), RES_OK);

  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);
  d3(pivot.data.pivot1.ref_point, 0, 0, 0);
  CHECK(my_type_track_me(target, &tracking), RES_OK);

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, 0.1, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);
  CHECK(my_type_ref_put(target), RES_OK);

  /*
  * 2 axis pivots
  * (using only one axis at a time)
  */

  /* 2 axis tracking sun */

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 0, 0, 1);

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  d3_splat(transl, +1);
  d3(rot, 0, 0, PI / 2);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  d3(in_dir, 0, 1, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  /* 2 axis tracking sun */

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  d3_splat(transl, +1);
  d3(rot, 0, 0, PI / 2);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 1, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  d3(in_dir, 0, 1, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  /* 2 axis tracking with a fixed output dir */

  tracking.policy = TRACKING_OUT_DIR;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(tracking.data.out_dir.u, 0, 1, 0);

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  d3(in_dir, -1, 0, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  /* 2 axis tracking a target point */

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 0;
  d3(pivot.data.pivot2.ref_point, 0, 0, 0);
  d3(tracking.data.point.target, 30, 0, 0);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, -1, 0, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = sqrt(2);
  d3(pivot.data.pivot2.ref_point, 0, 10 * sqrt(2), 0);
  d3(tracking.data.point.target, 30, -11, 0);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, -1, 0, 0);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 0, 10 * sqrt(2), 0);
  d3(tracking.data.point.target, 0, 30, 10);
  tracking.data.point.target_is_local = 1;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 0, 10 * sqrt(2), 0);
  d3(tracking.data.point.target, 0, 30, 12);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);

  tracking.policy = TRACKING_POINT;
  pivot.type = PIVOT_TWO_AXIS;
  pivot.data.pivot2.spacing = 1;
  d3(pivot.data.pivot2.ref_point, 10, 8 * sqrt(2), 5 * sqrt(2));
  d3(tracking.data.point.target, 10, 12, 15);
  tracking.data.point.target_is_local = 0;

  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_translation(t3, transl), RES_OK);

  d3(in_dir, 0, 0, -1);
  check_visit(allocator, in_dir, t1, t2, t3);

  /* release memory */
  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);
  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
