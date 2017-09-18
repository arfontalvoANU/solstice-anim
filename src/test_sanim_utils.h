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

#ifndef TEST_SANIM_UTILS_H
#define TEST_SANIM_UTILS_H

#include "sanim.h"

#include <rsys/rsys.h>
#include <rsys/ref_count.h>

struct mem_allocator;

/*******************************************************************************
 * Define a custom type of node based on sanim_node
 ******************************************************************************/
struct my_type {
  struct sanim_node node;
  double my_data;
  struct mem_allocator *allocator;
  ref_T ref;
};

res_T
my_type_create(struct mem_allocator *allocator, struct my_type** t);

res_T
my_type_pivot_create
  (struct mem_allocator *allocator,
   const struct sanim_pivot* pivot,
   const struct sanim_tracking* tracking,
   struct my_type** t);

/* create a new node and copy content of src into it 
 * non-recursive copy (father/child links are not set)
 * copy my_data, rotations, translation and possible pivot information */
res_T
my_type_copy_create
  (const struct my_type* src,
   struct my_type** dst);

res_T
my_type_ref_get(struct my_type* t);

res_T
my_type_ref_put(struct my_type* t);

res_T
my_type_add_child(struct my_type* t, struct my_type* child);

res_T
my_type_set_translation(struct my_type* t, const double translation[3]);

res_T
my_type_get_translation(const struct my_type* t, double translation[3]);

res_T
my_type_set_rotations(struct my_type* t, const double rotations[3]);

res_T
my_type_get_rotations(const struct my_type* t, double rotations[3]);

res_T
my_type_get_transform(struct my_type* t, double transform[12]);

res_T
my_type_solve_pivot(struct my_type* t, const double in_dir[3]);

res_T
my_type_is_pivot(struct my_type* t, int* pivot);

res_T
my_type_track_me(const struct my_type* t, struct sanim_tracking* tracking);

res_T
my_type_get_father
  (const struct my_type* t,
   struct my_type** father);

res_T
my_type_get_children_count(const struct my_type* t, size_t* count);

res_T
my_type_get_child
  (const struct my_type* t,
   const size_t idx,
   struct my_type** child);

res_T
my_type_recursive_copy
  (struct mem_allocator *allocator,
   const struct my_type* src,
   struct my_type** dst);

/*******************************************************************************
* Utilities
******************************************************************************/
char
d3_is_zero_eps(const double v[3], const double eps);

char
d3_is_zero(const double v[3]);

char
d33_is_identity_eps(const double v[9], const double eps);

char
d34_eq_eps(const double a[12], const double b[12], const double eps);

void
log_stream(const char* msg, void* ctx);

void
check_memory_allocator(struct mem_allocator* allocator);

#endif /* TEST_SANIM_UTILS_H */

