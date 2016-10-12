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

#ifndef TEST_SANIM_UTILS_H
#define TEST_SANIM_UTILS_H

#include "sanim.h"

#include <rsys/rsys.h>

struct mem_allocator;

/*******************************************************************************
 * Define a custom type of node based on sanim_node
 ******************************************************************************/
struct my_type {
  struct sanim_node node;
  double my_data;
  /* may be some ref count mechanism */
};

res_T
my_type_init(struct mem_allocator *allocator, struct my_type* t);

res_T
my_type_release(struct my_type* t);

res_T
my_type_add_child(struct my_type* t, struct my_type* child);

res_T
my_type_set_translation(struct my_type* t, const double translation[3]);

res_T
my_type_set_rotations(struct my_type* t, const double rotations[3]);

res_T
my_type_get_world_transform(struct my_type* t, double transform[12]);

/*******************************************************************************
* Utilities
******************************************************************************/
char
d3_is_zero_eps(const double v[3], const double eps);

char
d3_is_zero(const double v[3]);

char
d33_is_identity_eps(const double v[9], const double eps);

void
log_stream(const char* msg, void* ctx);

void
check_memory_allocator(struct mem_allocator* allocator);

#endif /* TEST_SANIM_UTILS_H */
