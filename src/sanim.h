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

#ifndef SANIM_H
#define SANIM_H

#include <rsys/rsys.h>

/* Library symbol management */
#if defined(SANIM_SHARED_BUILD) /* Build shared library */
#define SANIM_API extern EXPORT_SYM
#elif defined(SANIM_STATIC) /* Use/build static library */
#define SANIM_API extern LOCAL_SYM
#else /* Use shared library */
#define SANIM_API extern IMPORT_SYM
#endif

/* Helper macro that asserts if the invocation of the Solstice Anim function `Func'
* returns an error. One should use this macro on Solstice Anim function calls for which
* no explicit error checking is performed */
#ifndef NDEBUG
#define SANIM(Func) ASSERT(sanim_ ## Func == RES_OK)
#else
#define SANIM(Func) sanim_ ## Func
#endif

/* Syntactic sugar used to inform the Solstice Anim library that it can use
* as many threads as CPU cores */
#define SANIM_NTHREADS_DEFAULT (~0u)

/* Forward declaration of external types */
struct logger;
struct mem_allocator;
struct ssp_rng;

/* Opaque Solstice Anim types */
struct sanim_device;
struct sanim_node;

BEGIN_DECLS

/*******************************************************************************
 * Device API - Main entry point of the Solstice Anim library. Applications
 * use the sanim_device to create others Solstice Anim resources.
 ******************************************************************************/
SANIM_API res_T
sanim_device_create
(struct logger* logger, /* May be NULL <=> use default logger */
  struct mem_allocator* allocator, /* May be NULL <=> use default allocator */
  const unsigned nthreads_hint, /* Hint on the number of threads to use */
  const int verbose, /* Make the library more verbose */
  struct sanim_device** dev);

SANIM_API res_T
sanim_device_ref_get
(struct sanim_device* dev);

SANIM_API res_T
sanim_device_ref_put
(struct sanim_device* dev);

/*******************************************************************************
 * Node API.
 ******************************************************************************/
SANIM_API res_T
sanim_node_create
  (struct sanim_device* dev,
   struct sanim_node** node);

SANIM_API res_T
sanim_node_add_child
  (struct sanim_node* node,
   struct sanim_node* child);

SANIM_API res_T
sanim_node_ref_get
  (struct sanim_node* node);

SANIM_API res_T
sanim_node_ref_put
  (struct sanim_node* node);

END_DECLS

#endif /* SANIM_H */
