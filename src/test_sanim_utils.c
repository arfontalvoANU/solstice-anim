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

#include <rsys/rsys.h>
#include <rsys/mem_allocator.h>
#include <rsys/logger.h>
#include <rsys/dynamic_array.h>

/* Define the darray_tmp data structure */
#define DARRAY_NAME tmp
#define DARRAY_DATA struct my_type*
#include <rsys/dynamic_array.h>

#include <stdio.h>

res_T
my_type_create(struct mem_allocator *allocator, struct my_type** out)
{
  res_T res = RES_OK;
  struct my_type* t;
  struct mem_allocator* alloc;
  if (!out) return RES_BAD_ARG;
  alloc = allocator ? allocator : &mem_default_allocator;
  t = MEM_CALLOC(alloc, 1, sizeof(struct my_type));
  if (!t) {
    res = RES_MEM_ERR;
    goto error;
  }

  t->my_data = 0;
  t->allocator = alloc;
  ref_init(&t->ref);

  res = sanim_node_initialize(alloc, &t->node);
  if (res != RES_OK) return res;

exit:
  if (out) *out = t;
  return res;
error:
  if (t) {
    my_type_ref_put(t);
    t = NULL;
  }
  goto exit;
}

res_T
my_type_pivot_create
  (struct mem_allocator *allocator, 
   const struct sanim_pivot* pivot,
   const struct sanim_tracking* tracking,
   struct my_type** out)
{
  res_T res = RES_OK;
  struct my_type* t;
  struct mem_allocator* alloc;
  if (!out) return RES_BAD_ARG;
  alloc = allocator ? allocator : &mem_default_allocator;
  t = MEM_CALLOC(alloc, 1, sizeof(struct my_type));
  if (!t) {
    res = RES_MEM_ERR;
    goto error;
  }

  t->my_data = 0;
  t->allocator = alloc;
  ref_init(&t->ref);

  res = sanim_node_initialize_pivot(alloc, pivot, tracking, &t->node);
  if (res != RES_OK) goto error;

exit:
  if (out) *out = t;
  return res;
error:
  if (t) {
    my_type_ref_put(t);
    t = NULL;
  }
  goto exit;
}

res_T
my_type_copy_create
  (const struct my_type* src,
   struct my_type** dst)
{
  struct my_type* t;
  res_T res = RES_OK;
  if (!dst || ! src) return RES_BAD_ARG;
  ASSERT(src->allocator);
  t = MEM_CALLOC(src->allocator, 1, sizeof(struct my_type));
  if (!t) {
    res = RES_MEM_ERR;
    goto error;
  }
  t->allocator = src->allocator;
  ref_init(&t->ref);
  res = sanim_node_copy_initialize(src->allocator, &src->node, &t->node);
  if (res != RES_OK)
    goto error;
  t->my_data = src->my_data;
exit:
  *dst = t;
  return res;
error:
  if (t) {
    my_type_ref_put(t);
    t = NULL;
  }
  goto exit;
}

res_T
my_type_get_father
  (const struct my_type* t,
   const struct my_type** father)
{
  struct sanim_node* tmp;
  res_T res = RES_OK;
  if (!t || !father) return RES_BAD_ARG;
  res = sanim_node_get_father(&t->node, &tmp);
  if (res != RES_OK) return res;
  *father = CONTAINER_OF(tmp, struct my_type, node);
  return RES_OK;
}

res_T
my_type_get_children_count
  (const struct my_type* t,
   size_t* count)
{
  return sanim_node_get_children_count(&t->node, count);
}

res_T
my_type_get_child
  (const struct my_type* t,
   const size_t idx,
   const struct my_type** child)
{
  struct sanim_node* tmp;
  res_T res = RES_OK;
  if (!t || !child) return RES_BAD_ARG;
  res = sanim_node_get_child(&t->node, idx, &tmp);
  if (res != RES_OK) return res;
  *child = CONTAINER_OF(tmp, struct my_type, node);
  return RES_OK;
}

static res_T
my_type_recursive_copy_
  (struct mem_allocator *alloc,
   const struct my_type* src,
   struct darray_tmp* tmp,
   struct my_type** dst)
{
  struct my_type* root;
  size_t i, sz;
  res_T res = RES_OK;

  ASSERT(alloc && src && tmp && dst);
  res = my_type_copy_create(src, &root);
  if (res != RES_OK) goto error;
  darray_tmp_push_back(tmp, &root);
  if (res != RES_OK) goto error;
  CHECK(my_type_get_children_count(src, &sz), RES_OK);
  for (i = 0; i < sz; i++) {
    struct my_type* child;
    struct my_type* node;
    CHECK(my_type_get_child(src, i, &child), RES_OK);
    res = my_type_recursive_copy_(alloc, child, tmp, &node);
    if (res != RES_OK) goto error;
    my_type_add_child(root, node);
    my_type_ref_put(node); /* node is referenced by root */
  }
exit:
  *dst = root;
  return res;
error:
  root = NULL;
  goto exit;
}

res_T
my_type_recursive_copy
(struct mem_allocator *allocator,
  const struct my_type* src,
  struct my_type** dst)
{
  struct darray_tmp tmp;
  struct mem_allocator* alloc;
  res_T res = RES_OK;

  if (!dst || !src) return RES_BAD_ARG;
  if (*dst == src) return RES_BAD_ARG;
  alloc = allocator ? allocator : &mem_default_allocator;
  darray_tmp_init(alloc, &tmp);
  res = my_type_recursive_copy_(alloc, src, &tmp, dst);
  if (res != RES_OK) goto error;
exit:
  darray_tmp_release(&tmp);
  return res;
error:
  {
    struct my_type* const* children;
    size_t i, sz;
    sz = darray_tmp_size_get(&tmp);
    children = darray_tmp_cdata_get(&tmp);
    for (i = 0; i < sz; i++) {
      struct my_type* node = children[i];
      my_type_ref_put(node);
    }
    *dst = NULL;
    goto exit;
  }
}

static void
my_type_release(ref_T* ref)
{
  size_t i, sz;
  struct my_type* t = CONTAINER_OF(ref, struct my_type, ref);
  if (t->node.data) {
    CHECK(my_type_get_children_count(t, &sz), RES_OK);
    for (i = 0; i < sz; i++) {
      struct my_type* node;
      CHECK(my_type_get_child(t, i, &node), RES_OK);
      my_type_ref_put(node);
    }
  }
  SANIM(node_release(&t->node));
  MEM_RM(t->allocator, t);
}

res_T
my_type_ref_get(struct my_type* t)
{
  if (!t) return RES_BAD_ARG;
  ref_get(&t->ref);
  return RES_OK;
}

res_T
my_type_ref_put(struct my_type* t)
{
  if (!t) return RES_BAD_ARG;
  ref_put(&t->ref, my_type_release);
  return RES_OK;
}

res_T
my_type_add_child(struct my_type* t, struct my_type* child) {
  res_T res = RES_OK;
  if (!t || !child) return RES_BAD_ARG;
  res = sanim_node_add_child(&t->node, &child->node);
  if (res != RES_OK) return res;
  my_type_ref_get(child);
  return RES_OK;
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
my_type_get_transform(struct my_type* t, double transform[12]) {
  if (!t || !transform) return RES_BAD_ARG;
  return sanim_node_get_transform(&t->node, transform);
}

res_T
my_type_solve_pivot(struct my_type* t, const double in_dir[3]) {
  return sanim_node_solve_pivot(&t->node, in_dir);
}

res_T
my_type_is_pivot(struct my_type* t, int* pivot) {
  return sanim_node_is_pivot(&t->node, pivot);
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

char
d34_eq_eps(const double a[12], const double b[12], const double eps) {
  int i;
  ASSERT(eps >= 0);
  FOR_EACH(i, 0, 12) {
      if (fabs(a[i] - b[i]) > eps) return 0;
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

