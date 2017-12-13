/* Copyright (C) CNRS 2016-2017
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
  struct my_type *t1, *t2, *ptr;
  struct sanim_pivot pivot = SANIM_PIVOT_NULL;
  struct sanim_tracking tracking = SANIM_TRACKING_NULL;
  struct sanim_node s1 = SANIM_NODE_NULL, s2 = SANIM_NODE_NULL;
  size_t count;
  int p;
  double transform1[12], transform2[12];
  double transl[3], transl_[3], rot[3], rot_[3];
  (void) argc, (void) argv;

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);

  CHK(my_type_create(NULL, &t1) == RES_OK);
  CHK(my_type_ref_put(NULL) == RES_BAD_ARG);
  CHK(my_type_ref_get(NULL) == RES_BAD_ARG);
  CHK(my_type_ref_get(t1) == RES_OK);
  CHK(my_type_ref_put(t1) == RES_OK);
  CHK(my_type_ref_put(t1) == RES_OK);
  CHK(my_type_create(&allocator, &t1) == RES_OK);
  CHK(my_type_create(&allocator, NULL) == RES_BAD_ARG);
  CHK(my_type_create(&allocator, &t2) == RES_OK);
  CHK(my_type_ref_put(t1) == RES_OK);

  CHK(my_type_pivot_create(&allocator, NULL, &tracking, &t1) == RES_BAD_ARG);
  CHK(my_type_pivot_create(&allocator, &pivot, NULL, &t1) == RES_BAD_ARG);
  CHK(my_type_pivot_create(&allocator, &pivot, &tracking, NULL) == RES_BAD_ARG);
  CHK(my_type_pivot_create(&allocator, &pivot, &tracking, &t1) == RES_OK);

  CHK(my_type_is_pivot(NULL, &p) == RES_BAD_ARG);
  CHK(my_type_is_pivot(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_is_pivot(t1, &p) == RES_OK);
  CHK(p == 1);
  CHK(my_type_ref_put(t1) == RES_OK);

  CHK(my_type_create(&allocator, &t1) == RES_OK);

  CHK(my_type_is_pivot(t1, &p) == RES_OK);
  CHK(p == 0);

  CHK(my_type_add_child(NULL, t1) == RES_BAD_ARG);
  CHK(my_type_add_child(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_add_child(t1, t1) == RES_BAD_ARG);
  CHK(my_type_add_child(t1, t2) == RES_OK);
  CHK(my_type_add_child(t1, t2) == RES_BAD_ARG);
  CHK(my_type_add_child(t2, t1) == RES_BAD_ARG);

  CHK(my_type_copy_create(NULL, &ptr) == RES_BAD_ARG);
  CHK(my_type_copy_create(t1, NULL) == RES_BAD_ARG);

  CHK(my_type_get_father(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_get_father(NULL, &ptr) == RES_BAD_ARG);
  CHK(my_type_get_children_count(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_get_children_count(NULL, &count) == RES_BAD_ARG);
  CHK(my_type_get_children_count(t1, &count) == RES_OK);
  CHK(count == 1);
  CHK(my_type_get_child(NULL, 0, &ptr) == RES_BAD_ARG);
  CHK(my_type_get_child(t1, 10, &ptr) == RES_BAD_ARG);
  CHK(my_type_get_child(t1, 0, NULL) == RES_BAD_ARG);
  CHK(my_type_get_child(t1, 0, &ptr) == RES_OK);
  CHK(ptr == t2);
  CHK(my_type_get_father(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_get_father(NULL, &ptr) == RES_BAD_ARG);
  CHK(my_type_get_father(t1, &ptr) == RES_OK);
  CHK(ptr == NULL);
  CHK(my_type_get_father(t2, &ptr) == RES_OK);
  CHK(ptr == t1);

  CHK(my_type_set_translation(NULL, transl) == RES_BAD_ARG);
  CHK(my_type_set_translation(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_set_translation(t1, transl) == RES_OK);

  CHK(my_type_get_translation(NULL, transl_) == RES_BAD_ARG);
  CHK(my_type_get_translation(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_get_translation(t1, transl_) == RES_OK);
  CHK(d3_eq(transl, transl_) == 1);

  CHK(my_type_set_rotations(NULL, rot) == RES_BAD_ARG);
  CHK(my_type_set_rotations(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_set_rotations(t1, rot) == RES_OK);

  CHK(my_type_get_rotations(NULL, rot_) == RES_BAD_ARG);
  CHK(my_type_get_rotations(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_get_rotations(t1, rot_) == RES_OK);
  CHK(d3_eq(rot, rot_) == 1);

  CHK(my_type_get_transform(NULL, transform1) == RES_BAD_ARG);
  CHK(my_type_get_transform(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_get_transform(t1, transform1) == RES_OK);

  CHK(my_type_copy_create(NULL, &ptr) == RES_BAD_ARG);
  CHK(my_type_copy_create(t1, NULL) == RES_BAD_ARG);
  /*t1->my_data = 1;*/
  CHK(my_type_copy_create(t1, &ptr) == RES_OK);
  CHK(t1->my_data == ptr->my_data);
  CHK(my_type_get_transform(ptr, transform2) == RES_OK);
  CHK(d34_eq_eps(transform1, transform2, 0) == 1);
  CHK(my_type_ref_put(ptr) == RES_OK);

  CHK(my_type_recursive_copy(&allocator, NULL, &ptr) == RES_BAD_ARG);
  CHK(my_type_recursive_copy(&allocator, t1, NULL) == RES_BAD_ARG);
  CHK(my_type_recursive_copy(&allocator, t1, &t1) == RES_BAD_ARG);
  CHK(my_type_recursive_copy(&allocator, t1, &ptr) == RES_OK);

  tracking.policy = TRACKING_POINT;
  tracking.data.node_target.tracked_node = NULL;
  CHK(my_type_track_me(NULL, &tracking) == RES_BAD_ARG);
  CHK(my_type_track_me(t1, NULL) == RES_BAD_ARG);
  CHK(my_type_track_me(t1, &tracking) == RES_OK);
  CHK(tracking.policy == TRACKING_NODE_TARGET);
  CHK(&t1->node == tracking.data.node_target.tracked_node);

  /* Some sanim API tests that cannot be carried out through my_type testing.
   * Test use of non-initialized sanim nodes. */
  CHK(sanim_node_is_initialized(NULL, &p) == RES_BAD_ARG);
  CHK(sanim_node_is_initialized(&s1, NULL) == RES_BAD_ARG);
  CHK(sanim_node_is_initialized(&s1, &p) == RES_OK);
  CHK(p == 0);
  CHK(sanim_node_release(&s1) == RES_OK); /* safe release of uninitialized nodes */
  CHK(sanim_node_initialize(&allocator, NULL) == RES_BAD_ARG);
  CHK(sanim_node_initialize(&allocator, &s1) == RES_OK);
  CHK(sanim_node_is_initialized(&s1, &p) == RES_OK);
  CHK(p == 1);
  CHK(sanim_node_release(&s1) == RES_OK);
  CHK(sanim_node_is_initialized(&s2, &p) == RES_OK);
  CHK(p == 0);
  CHK(sanim_node_initialize_pivot(&allocator, NULL, &tracking, &s2) == RES_BAD_ARG);
  CHK(sanim_node_initialize_pivot(&allocator, &pivot, NULL, &s2) == RES_BAD_ARG);
  CHK(sanim_node_initialize_pivot(&allocator, &pivot, &tracking, NULL) == RES_BAD_ARG);
  CHK(sanim_node_initialize_pivot(&allocator, &pivot, &tracking, &s2) == RES_OK);
  CHK(sanim_node_is_initialized(&s2, &p) == RES_OK);
  CHK(p == 1);
  CHK(sanim_node_release(NULL) == RES_BAD_ARG);
  CHK(sanim_node_release(&s2) == RES_OK);
  CHK(sanim_node_release(&s2) == RES_OK); /* safe mumtiple release */

  /* release memory */
  CHK(my_type_ref_put(t1) == RES_OK);
  CHK(my_type_ref_put(t2) == RES_OK);
  CHK(my_type_ref_put(ptr) == RES_OK);
  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHK(mem_allocated_size() == 0);
  return 0;
}
