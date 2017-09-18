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

#ifndef SANIM_NODE_C_H
#define SANIM_NODE_C_H

struct mem_allocator;

#include "sanim.h"

#include <rsys/dynamic_array.h>

/* Define the darray_children data structure */
#define DARRAY_NAME children
#define DARRAY_DATA struct sanim_node*
#include <rsys/dynamic_array.h>

/* supplemental data for nodes that are pivots */
struct pivot_data {
  double angleX, angleZ;
  struct sanim_pivot pivot;
  struct sanim_tracking tracking;
};

struct node_data {
  double translation[3];
  double rotations[3];
  struct sanim_node* father; /* can be NULL: root node */
  struct darray_children children;
  struct mem_allocator* allocator;
  struct pivot_data* pivot_data;
};


#endif /* SANIM_NODE_C_H */
