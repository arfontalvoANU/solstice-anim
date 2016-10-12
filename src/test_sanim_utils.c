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

#include "test_sanim_utils.h"

#include <rsys/mem_allocator.h>
#include <rsys/logger.h>

#include <stdio.h>

res_T
my_type_init(struct mem_allocator *allocator, struct my_type* t) {
  if (!t) return RES_BAD_ARG;
  /* init my stuff */
  t->my_data = 0;
  /* init node stuff */
  return sanim_node_initialize(allocator, &t->node);
}

res_T
my_type_init_pivot
  (struct mem_allocator *allocator, 
   const struct sanim_pivot* pivot,
   const struct sanim_tracking* tracking,
   struct my_type* t)
{
  if (!t) return RES_BAD_ARG;
  /* init my stuff */
  t->my_data = 0;
  /* init node stuff */
  return sanim_node_initialize_pivot(allocator, pivot, tracking, &t->node);
}

res_T
my_type_release(struct my_type* t) {
  if (!t) return RES_BAD_ARG;
  /* release my stuff */
  /* release node stuff */
  return sanim_node_release(&t->node);
}

res_T
my_type_add_child(struct my_type* t, struct my_type* child) {
  if (!t || !child) return RES_BAD_ARG;
  /* release my stuff */
  /* release node stuff */
  return sanim_node_add_child(&t->node, &child->node);
}

res_T
my_type_set_translation(struct my_type* t, const double translation[3]) {
  if (!t || !translation) return RES_BAD_ARG;
  return sanim_node_set_translation(&t->node, translation);
}

res_T
my_type_set_rotations(struct my_type* t, const double rotations[3]) {
  if (!t || !rotations) return RES_BAD_ARG;
  return sanim_node_set_rotations(&t->node, rotations);
}

res_T
my_type_get_world_transform(struct my_type* t, double transform[12]) {
  if (!t || !transform) return RES_BAD_ARG;
  return sanim_node_get_world_transform(&t->node, transform);
}

char
d3_is_zero_eps(const double v[3], const double eps) {
  int x;
  ASSERT(eps >= 0);
  FOR_EACH(x, 0, 3) {
    if (fabs(v[x]) > eps) return 0;
  }
  return 1;
}

char
d3_is_zero(const double v[3]) {
  int x;
  FOR_EACH(x, 0, 3) {
    if (v[x]) return 0;
  }
  return 1;
}

char
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

void
log_stream(const char* msg, void* ctx) {
  ASSERT(msg);
  (void) msg, (void) ctx;
  printf("%s\n", msg);
}

void
check_memory_allocator(struct mem_allocator* allocator) {
  if (MEM_ALLOCATED_SIZE(allocator)) {
    char dump[512];
    MEM_DUMP(allocator, dump, sizeof(dump) / sizeof(char));
    fprintf(stderr, "%s\n", dump);
    FATAL("Memory leaks\n");
  }
}

