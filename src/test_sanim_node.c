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

struct my_type {
  struct sanim_node node;
  double my_data;
  /* may be some ref count mechanism */
};

static res_T
my_type_init(struct mem_allocator *allocator, struct my_type* t) {
  if (!t) return RES_BAD_ARG;
  /* init my stuff */
  t->my_data = 0;
  /* init node stuff */
  return sanim_node_initialize(allocator, &t->node);
}

static res_T
my_type_release(struct my_type* t) {
  if (!t) return RES_BAD_ARG;
  /* release my stuff */
  /* release node stuff */
  return sanim_node_release(&t->node);
}

static res_T
my_type_add_child(struct my_type* t, struct my_type* child) {
  if (!t || !child) return RES_BAD_ARG;
  /* release my stuff */
  /* release node stuff */
  return sanim_node_add_child(&t->node, &child->node);
}

static res_T
my_type_set_translation(struct my_type* t, const double translation[3]) {
  if (!t || !translation) return RES_BAD_ARG;
  return sanim_node_set_translation(&t->node, translation);
}

static res_T
my_type_set_rotations(struct my_type* t, const double rotations[3]) {
  if (!t || !rotations) return RES_BAD_ARG;
  return sanim_node_set_rotations(&t->node, rotations);
}

static res_T
my_type_get_world_transform(struct my_type* t, double transform[12]) {
  if (!t || !transform) return RES_BAD_ARG;
  return sanim_node_get_world_transform(&t->node, transform);
}

static char
d3_is_zero_eps(const double v[3], const double eps) {
  int x;
  ASSERT(eps >= 0);
  FOR_EACH(x, 0, 3) {
    if (fabs(v[x]) > eps) return 0;
  }
  return 1;
}

static char
d3_is_zero(const double v[3]) {
  int x;
  FOR_EACH(x, 0, 3) {
    if (v[x]) return 0;
  }
  return 1;
}

static char
d33_is_identity_eps(const double v[9], const double eps) {
  int i = 0, x, y;
  ASSERT(eps >= 0);
  FOR_EACH(x, 0, 3) {
    FOR_EACH(y, 0, 3) {
      if (fabs(v[i] - (x == y ? 1 : 0)) > eps) return 0;
      ++i;
    }
  }
  return 1;
}

int
main(int argc, char** argv)
{
  struct mem_allocator allocator;
  struct my_type t1, t2;
  double transl[3], rot[3], transform[12];
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);
  
  /* test a typical use in my_type */
  CHECK(my_type_init(NULL, &t1), RES_OK);
  CHECK(my_type_release(NULL), RES_BAD_ARG);
  CHECK(my_type_release(&t1), RES_OK);
  CHECK(my_type_init(&allocator, &t1), RES_OK);
  CHECK(my_type_init(&allocator, NULL), RES_BAD_ARG);
  CHECK(my_type_init(&allocator, &t2), RES_OK);

  CHECK(my_type_add_child(NULL, &t1), RES_BAD_ARG);
  CHECK(my_type_add_child(&t1, NULL), RES_BAD_ARG);
  CHECK(my_type_add_child(&t1, &t1), RES_BAD_ARG);
  CHECK(my_type_add_child(&t1, &t2), RES_OK);
  CHECK(my_type_add_child(&t1, &t2), RES_BAD_ARG);
  CHECK(my_type_add_child(&t2, &t1), RES_BAD_ARG);

  d3_splat(transl, +1);
  CHECK(my_type_set_translation(NULL, transl), RES_BAD_ARG);
  CHECK(my_type_set_translation(&t1, NULL), RES_BAD_ARG);
  CHECK(my_type_set_translation(&t1, transl), RES_OK);

  d3_splat(transl, -1);
  CHECK(my_type_set_translation(&t2, transl), RES_OK);

  CHECK(my_type_get_world_transform(NULL, transform), RES_BAD_ARG);
  CHECK(my_type_get_world_transform(&t2, NULL), RES_BAD_ARG);
  CHECK(my_type_get_world_transform(&t2, transform), RES_OK);
  CHECK(d33_is_identity(transform), 1);
  CHECK(d3_is_zero(transform + 9), 1);

  d3(rot, PI, 0, 0);
  CHECK(my_type_set_rotations(NULL, rot), RES_BAD_ARG);
  CHECK(my_type_set_rotations(&t1, NULL), RES_BAD_ARG);
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

  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
