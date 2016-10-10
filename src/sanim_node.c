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

/*******************************************************************************
* Helper functions
******************************************************************************/
static void
node_release(ref_T* ref)
{
  struct sanim_device* dev;
  struct sanim_node* node = CONTAINER_OF(ref, struct sanim_node, ref);
  ASSERT(ref);
  dev = node->dev;
  ASSERT(dev && dev->allocator);
  darray_children_release(&node->children);
  /* FIXME: use refcount for father/children? */
  MEM_RM(dev->allocator, node);
  SANIM(device_ref_put(dev));
}

static int
is_ascendant(const struct sanim_node* node, const struct sanim_node* ascendant)
{
  ASSERT(node && ascendant);
  while (node) {
    if (node == ascendant) return 1;
    node = node->father;
  }
  return 0;
}

/*******************************************************************************
* Exported ssol_spectrum functions
******************************************************************************/
res_T
sanim_node_create
  (struct sanim_device* dev,
   struct sanim_node** out_node)
{
  struct sanim_node* node = NULL;
  res_T res = RES_OK;

  if (!dev || !out_node) {
    res = RES_BAD_ARG;
    goto error;
  }

  node = (struct sanim_node*)MEM_CALLOC
    (dev->allocator, 1, sizeof(struct sanim_node));
  if (!node) {
    res = RES_MEM_ERR;
    goto error;
  }

  darray_children_init(dev->allocator, &node->children);

  SANIM(device_ref_get(dev));
  node->dev = dev;
  ref_init(&node->ref);

exit:
  if (out_node) *out_node = node;
  return res;
error:
  if (node) {
    SANIM(node_ref_put(node));
    node = NULL;
  }
  goto exit;
}

res_T
sanim_node_add_child
  (struct sanim_node* node,
   struct sanim_node* child)
{
  res_T res = RES_OK;

  if (!node || !child) return RES_BAD_ARG;
  if (child->father) {
    log_warning
      (node->dev, "%s: the node has a father already.\n", FUNC_NAME);
    return RES_BAD_ARG;
  }
  if (is_ascendant(node, child)) {
    log_warning
      (node->dev, "%s: creating a cycle.\n", FUNC_NAME);
    return RES_BAD_ARG;
  }

  child->father = node;
  res = darray_children_push_back(&node->children, &child);
  if (res != RES_OK) {
    goto error;
  }
  /* FIXME: use refcount for father/children? */

exit:
  return res;
error:
  child->father = NULL;
  goto exit;
}

res_T
sanim_node_ref_get
  (struct sanim_node* node)
{
  if (!node) return RES_BAD_ARG;
  ref_get(&node->ref);
  return RES_OK;
}

res_T
sanim_node_ref_put
  (struct sanim_node* node)
{
  if (!node) return RES_BAD_ARG;
  ref_put(&node->ref, node_release);
  return RES_OK;
}
