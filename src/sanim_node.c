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

#include "sanim_node_c.h"
#include "sanim_device_c.h"
#include "sanim.h"

#include <rsys/mem_allocator.h>
#include <rsys/ref_count.h>
#include <rsys/double33.h>
#include <rsys/double44.h>

/*******************************************************************************
* Helper functions
******************************************************************************/
static int
is_ascendant
  (const struct sanim_node_data* data, const struct sanim_node_data* asc_data)
{
  ASSERT(data && asc_data);
  while (data) {
    if (data == asc_data) return 1;
    data = data->father;
  }
  return 0;
}

static int
is_after_pivot(const struct sanim_node_data* data) {
  ASSERT(data);
  while (data) {
    if (data->pivot) return 1;
    data = data->father;
  }
  return 0;
}

static double*
node_get_transform(struct sanim_node_data* data,  double transform[16]) {
  double tmp[12];
  ASSERT(data && transform);
  d33_rotation(tmp, SPLIT3(data->rotations));
  d3_set(transform, tmp);
  transform[3] = 0;
  d3_set(transform + 4, tmp + 3);
  transform[7] = 0;
  d3_set(transform + 8, tmp + 6);
  transform[11] = 0;
  d3_set(transform + 12, data->translation);
  transform[15] = 1;
  return transform;
}

static void 
d33_setd44(double dst[12], double src[16]) {
  d3_set(dst, src);
  d3_set(dst + 3, src + 4);
  d3_set(dst + 6, src + 8);
  d3_set(dst + 9, src + 12);
  ASSERT(!src[3] && !src[7] && !src[11] && src[15] == 1);
}

/*******************************************************************************
* Exported ssol_spectrum functions
******************************************************************************/
res_T
sanim_node_add_child
  (struct sanim_node* node,
   struct sanim_node* child)
{
  res_T res = RES_OK;

  if (!node || !child) return RES_BAD_ARG;
  if (child->data->father) return RES_BAD_ARG;
  if (is_ascendant(node->data, child->data)) return RES_BAD_ARG;
  if (child->data->pivot && is_after_pivot(node->data)) return RES_BAD_ARG;

  child->data->father = node->data;
  res = darray_children_push_back(&node->data->children, &child->data);
  if (res != RES_OK) {
    goto error;
  }

exit:
  return res;
error:
  if (child->data) {
    child->data = NULL;
  }
  goto exit;
}

res_T
sanim_node_initialize
  (struct mem_allocator* allocator,
   struct sanim_node* node)
{
  struct mem_allocator* alloc;
  res_T res = RES_OK;

  if (!node) return RES_BAD_ARG;
  alloc = allocator ? allocator : &mem_default_allocator;

  node->data = MEM_CALLOC(alloc, 1, sizeof(struct sanim_node_data));
  if (!node->data) {
    res = RES_MEM_ERR;
    goto error;
  }

  darray_children_init(alloc, &node->data->children);
  node->data->father = NULL;
  node->data->allocator = alloc;
  node->data->pivot = NULL;
  d3_splat(node->data->translation, 0);
  d3_splat(node->data->rotations, 0);

exit:
  return res;
error:
  if (node->data) {
    darray_children_release(&node->data->children);
    node->data = NULL;
  }
  goto exit;
}

res_T
sanim_node_initialize_pivot
  (struct mem_allocator* allocator,
   const struct sanim_pivot* pivot,
   struct sanim_node* node)
{
  res_T res;

  if (!pivot) return RES_BAD_ARG;
  res = sanim_node_initialize(allocator, node);
  if (res != RES_OK) goto error;

  *node->data->pivot = *pivot;

exit:
  return res;
error:
  if (node->data) {
    darray_children_release(&node->data->children);
    node->data = NULL;
  }
  goto exit;
}

res_T
sanim_node_release
  (struct sanim_node* node)
{
  if (!node) return RES_BAD_ARG;
  if (node->data) {
    darray_children_release(&node->data->children);
    MEM_RM(node->data->allocator, node->data);
  }
  return RES_OK;
}

res_T
sanim_node_set_translation
  (struct sanim_node* node,
   const double translation[3])
{
  if (!node || !node->data || !translation) return RES_BAD_ARG;
  d3_set(node->data->translation, translation);
  return RES_OK;
}

res_T
sanim_node_set_rotations
  (struct sanim_node* node,
   const double rotations[3])
{
  if (!node || !node->data || !rotations) return RES_BAD_ARG;
  d3_set(node->data->rotations, rotations);
  return RES_OK;
}

res_T
sanim_node_get_world_transform
  (struct sanim_node* node,
   double transform[12])
{
  double world[16], tmp[16]; /* 4x4 column major matrix */
  struct sanim_node_data* ptr;
  if (!node || !node->data || !transform) return RES_BAD_ARG;
  node_get_transform(node->data, world);
  ptr = node->data->father;
  while (ptr) {
    node_get_transform(ptr, tmp);
    d44_muld44(world, tmp, world);
    ptr = ptr->father;
  }
  d33_setd44(transform, world);
  return RES_OK;
}
