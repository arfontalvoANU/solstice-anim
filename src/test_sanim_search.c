
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

#include "sanim.h"
#include "test_sanim_utils.h"

#include <rsys/double33.h>

struct data {
  struct my_type * result;
};

static res_T
search(const struct sanim_node* n, void* data_, int* found)
{
  int pivot;
  ASSERT(n && found);
  if (!data_) return RES_BAD_ARG;
  SANIM(node_is_pivot(n, &pivot));
  if (pivot) {
    struct my_type* node;
    struct data* data = data_;
    node = CONTAINER_OF(n, struct my_type, node);
    data->result = node;
    *found = 1;
  }
  return RES_OK;
}

int
main(int argc, char** argv)
{
  struct mem_allocator allocator;
  struct my_type *t1, *t2, *t3;
  struct sanim_pivot pivot = SANIM_PIVOT_NULL;
  struct sanim_tracking tracking = SANIM_TRACKING_NULL;
  struct data data;
  int found = 0;
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);

  tracking.policy = TRACKING_SUN;
  pivot.type = PIVOT_SINGLE_AXIS;
  d3(pivot.data.pivot1.ref_normal, 0, 0, 1);

  /* ref_normal not in the YZ plane */
  CHECK(my_type_create(&allocator, &t1), RES_OK);
  CHECK(my_type_pivot_create(&allocator, &pivot, &tracking, &t2), RES_OK);
  CHECK(my_type_create(&allocator, &t3), RES_OK);

  CHECK(my_type_add_child(t1, t2), RES_OK);
  CHECK(my_type_add_child(t2, t3), RES_OK);

  data.result = NULL;

  CHECK(sanim_node_search_tree(NULL, &data, search, &found), RES_BAD_ARG);
  CHECK(sanim_node_search_tree(&t1->node, NULL, search, &found), RES_BAD_ARG);
  CHECK(sanim_node_search_tree(&t1->node, &data, NULL, &found), RES_BAD_ARG);
  CHECK(sanim_node_search_tree(&t1->node, &data, search, NULL), RES_BAD_ARG);
  CHECK(sanim_node_search_tree(&t1->node, &data, search, &found), RES_OK);
  CHECK(data.result, t2);

  data.result = NULL;
  CHECK(sanim_node_search_tree(&t3->node, &data, search, &found), RES_OK);
  CHECK(data.result, NULL);
  
  /* release memory */
  CHECK(my_type_ref_put(t1), RES_OK);
  CHECK(my_type_ref_put(t2), RES_OK);
  CHECK(my_type_ref_put(t3), RES_OK);
  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
