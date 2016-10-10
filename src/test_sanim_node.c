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

#include <rsys/logger.h>

int
main(int argc, char** argv)
{
  struct logger logger;
  struct mem_allocator allocator;
  struct sanim_device* dev;
  struct sanim_node* node1;
  struct sanim_node* node2;
  (void) argc, (void) argv;

  mem_init_proxy_allocator(&allocator, &mem_default_allocator);

  CHECK(logger_init(&allocator, &logger), RES_OK);
  logger_set_stream(&logger, LOG_OUTPUT, log_stream, NULL);
  logger_set_stream(&logger, LOG_ERROR, log_stream, NULL);
  logger_set_stream(&logger, LOG_WARNING, log_stream, NULL);

  CHECK(sanim_device_create
    (&logger, &allocator, SANIM_NTHREADS_DEFAULT, 0, &dev), RES_OK);

  CHECK(sanim_node_create(NULL, &node1), RES_BAD_ARG);
  CHECK(sanim_node_create(dev, NULL), RES_BAD_ARG);
  CHECK(sanim_node_create(dev, &node1), RES_OK);
  CHECK(sanim_node_ref_get(NULL), RES_BAD_ARG);
  CHECK(sanim_node_ref_get(node1), RES_OK);
  CHECK(sanim_node_ref_put(NULL), RES_BAD_ARG);
  CHECK(sanim_node_ref_put(node1), RES_OK);
  CHECK(sanim_node_ref_put(node1), RES_OK);

  CHECK(sanim_node_create(dev, &node1), RES_OK);
  CHECK(sanim_node_create(dev, &node2), RES_OK);

  CHECK(sanim_node_add_child(NULL, node1), RES_BAD_ARG);
  CHECK(sanim_node_add_child(node1, NULL), RES_BAD_ARG);
  CHECK(sanim_node_add_child(node1, node1), RES_BAD_ARG);
  CHECK(sanim_node_add_child(node1, node2), RES_OK);
  CHECK(sanim_node_add_child(node1, node2), RES_BAD_ARG);
  CHECK(sanim_node_add_child(node2, node1), RES_BAD_ARG);

  CHECK(sanim_node_ref_put(node1), RES_OK);
  CHECK(sanim_node_ref_put(node2), RES_OK);

  CHECK(sanim_device_ref_put(dev), RES_OK);

  logger_release(&logger);
  check_memory_allocator(&allocator);
  mem_shutdown_proxy_allocator(&allocator);
  CHECK(mem_allocated_size(), 0);
  return 0;
}
