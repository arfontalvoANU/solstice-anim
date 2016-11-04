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
  struct my_type *t1, *t2, *t3, *t, *ptr;
  double transl[3], rot[3];
  double transform[12], transform_[12];
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);
  
  /* test a typical use in my_type */
  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_create(&allocator, &t2), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);

  d3_splat(transl, +1);
  CHECK(my_type_set_translation(t1, transl), RES_OK);

  d3_splat(transl, -1);
  CHECK(my_type_set_translation(t2, transl), RES_OK);

  CHECK(my_type_get_transform(t2, transform), RES_OK);
  CHECK(d33_is_identity(transform), 1);
  CHECK(d3_is_zero(transform + 9), 1);

  d3(rot, PI, 0, 0);
  CHECK(my_type_set_rotations(NULL, rot), RES_BAD_ARG);
  CHECK(my_type_set_rotations(t1, NULL), RES_BAD_ARG);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  CHECK(my_type_set_rotations(t2, rot), RES_OK);
  d3(transl, 0, +1, 0);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_translation(t2, transl), RES_OK);

  CHECK(my_type_get_transform(t2, transform), RES_OK);
  CHECK(d33_is_identity_eps(transform, 1e-7), 1);
  CHECK(d3_is_zero_eps(transform + 9, 1e-7), 1);

  /* check translation & 1 rotation in a single node VS in 2 chained nodes */
  d3(rot, 0.17, 0, 0);
  d3(transl, 0.3, 2, -1);
  CHECK(my_type_create(&allocator, &t), RES_OK);
  CHECK(my_type_set_translation(t, transl), RES_OK);
  CHECK(my_type_set_rotations(t, rot), RES_OK);
  CHECK(my_type_get_transform(t, transform), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);
  d3(rot, 0, 0, 0);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  d3(transl, 0, 0, 0);
  d3(rot, 0.17, 0, 0);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_rotations(t2, rot), RES_OK);
  CHECK(my_type_get_transform(t2, transform_), RES_OK);
  CHECK(d33_eq_eps(transform, transform_, 1e-7), 1);
  CHECK(d3_eq_eps(transform + 9, transform_ + 9, 1e-7), 1);

  /* check translation & 2 rotations in a single node VS in 2 chained nodes */
  d3(rot, 0.17, -0.52, 0);
  d3(transl, 0.3, 2, -1);
  CHECK(my_type_set_translation(t, transl), RES_OK);
  CHECK(my_type_set_rotations(t, rot), RES_OK);
  CHECK(my_type_get_transform(t, transform), RES_OK);
  d3(rot, 0, 0, 0);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  d3(transl, 0, 0, 0);
  d3(rot, 0.17, 0, 0);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_rotations(t2, rot), RES_OK);
  d3(rot, 0, -0.52, 0);
  CHECK(my_type_set_translation(t3, transl), RES_OK);
  CHECK(my_type_set_rotations(t3, rot), RES_OK);
  CHECK(my_type_get_transform(t3, transform_), RES_OK);
  CHECK(d33_eq_eps(transform, transform_, 1e-7), 1);
  CHECK(d3_eq_eps(transform + 9, transform_ + 9, 1e-7), 1);

  /* check 1 node with 3 rotations VS 3 chained nodes with 1 rotation each */
  d3(rot, 0.17, -0.52, 0.31);
  d3(transl, 0.3, 2, -1);
  CHECK(my_type_set_translation(t, transl), RES_OK);
  CHECK(my_type_set_rotations(t, rot), RES_OK);
  CHECK(my_type_get_transform(t, transform), RES_OK);
  d3(rot, 0.17, 0, 0);
  CHECK(my_type_set_translation(t1, transl), RES_OK);
  CHECK(my_type_set_rotations(t1, rot), RES_OK);
  d3(transl, 0, 0, 0);
  d3(rot, 0, -0.52, 0);
  CHECK(my_type_set_translation(t2, transl), RES_OK);
  CHECK(my_type_set_rotations(t2, rot), RES_OK);
  d3(rot, 0, 0, 0.31);
  CHECK(my_type_set_translation(t3, transl), RES_OK);
  CHECK(my_type_set_rotations(t3, rot), RES_OK);
  CHECK(my_type_get_transform(t3, transform_), RES_OK);
  CHECK(d33_eq_eps(transform, transform_, 1e-7), 1);
  CHECK(d3_eq_eps(transform + 9, transform_ + 9, 1e-7), 1);

  /* check 3 chained nodes with 1 rotation each VS a copy */
  CHECK(my_type_ref_put(t), RES_OK);
  CHECK(my_type_recursive_copy(&allocator, t1, &t), RES_OK);
  CHECK(my_type_get_transform(t3, transform), RES_OK);
  CHECK(my_type_get_child(t, 0, &ptr), RES_OK);
  CHECK(my_type_get_child(ptr, 0, &ptr), RES_OK);
  CHECK(my_type_get_transform(ptr, transform_), RES_OK);
  CHECK(d33_eq_eps(transform, transform_, 1e-7), 1);
  CHECK(d3_eq_eps(transform + 9, transform_ + 9, 1e-7), 1);

  /* release memory */
  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);
  CHECK(my_type_ref_put(t), RES_OK);

  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
