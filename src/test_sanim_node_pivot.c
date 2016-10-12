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
  double transl[3], rot[3], transform[12];
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);
  
  /* test a typical use in my_type */
  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3_splat(pivot.data.pivot1.ref_point, 0);
  d3(pivot.data.pivot1.ref_normal, 0, 1, 0);
  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init_pivot(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_init(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t2, &t3), RES_OK);

  d3_splat(transl, +1);
  CHECK(my_type_set_translation(&t1, transl), RES_OK);

  d3_splat(transl, -1);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);

  CHECK(my_type_get_world_transform(&t2, transform), RES_OK);
  CHECK(d33_is_identity(transform), 1);
  CHECK(d3_is_zero(transform + 9), 1);

  d3(rot, PI, 0, 0);
  CHECK(my_type_set_rotations(&t1, rot), RES_OK);
  CHECK(my_type_set_rotations(&t2, rot), RES_OK);
  d3(transl, 0, +1, 0);
  CHECK(my_type_set_translation(&t1, transl), RES_OK);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);

  CHECK(my_type_get_world_transform(&t2, transform), RES_OK);
  CHECK(d33_is_identity_eps(transform, 1e-10), 1);
  CHECK(d3_is_zero_eps(transform + 9, 1e-10), 1);

  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_release(&t2), RES_OK);
  CHECK(my_type_release(&t3), RES_OK);

  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
