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
#include <rsys/dynamic_array.h>

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
struct node_data;

/* sanim_node type for building own node types */
struct sanim_node {
  struct node_data* data;
};

/* types to describe pivots */
enum pivot_type {
  PIVOT_SINGLE_AXIS,
  PIVOT_TWO_AXIS,

  PIVOT_TYPES_COUNT__
};

/* pivot with X rotation axis */
struct sanim_pivot_1 {
  double ref_point[3]; /* in post-pivot local space */
  double ref_normal[3]; /* normal without rotation; used to compute output dir */
};

/* pivot with Z then X rotation axis */
struct sanim_pivot_2 {
  double spacing; /* distance between Z and X rotation axis */
  double ref_point[3]; /* in post-pivot local space */
  /* ref_normal is <0,1,0> */
};

struct sanim_pivot {
  enum pivot_type type;
  union {
    struct sanim_pivot_1 pivot1;
    struct sanim_pivot_2 pivot2;
  } data;
};

/* types to describe tracking policies */
enum tracking_policy {
  TRACKING_SUN, /* orient the device to face the sun */
  TRACKING_POINT, /* direct the output flux towards a point */
  TRACKING_OUT_DIR, /* direct the output flux towards a given dir */

  TRACKING_TYPES_COUNT
};

struct sanim_policy_sun {
  char unused;
};

struct sanim_policy_point {
  /* target can be in local space (ie in the same system than the pivot)
   * or in world space */
  double target[3];
  char target_is_local;
};

struct sanim_policy_out_dir {
  double u[3]; /* in world space */
};

struct sanim_tracking {
  enum tracking_policy policy;
  union {
    struct sanim_policy_sun sun;
    struct sanim_policy_point point;
    struct sanim_policy_out_dir out_dir;
  } data;
};
BEGIN_DECLS

/*******************************************************************************
 * Node API.
 ******************************************************************************/
SANIM_API res_T
sanim_node_initialize
  (struct mem_allocator* allocator, /* May be NULL <=> use default allocator */
   struct sanim_node* node);

SANIM_API res_T
sanim_node_initialize_pivot
  (struct mem_allocator* allocator, /* May be NULL <=> use default allocator */
   const struct sanim_pivot* pivot,
   const struct sanim_tracking* tracking,
   struct sanim_node* node);

SANIM_API res_T
sanim_node_solve_pivot
  (struct sanim_node* node,
   const double in_dir[3]);

SANIM_API res_T
sanim_node_release
  (struct sanim_node* node);

SANIM_API res_T
sanim_node_add_child
  (struct sanim_node* father,
   struct sanim_node* child);

SANIM_API res_T
sanim_node_set_translation
  (struct sanim_node* node,
   const double translation[3]);

SANIM_API res_T
sanim_node_set_rotations
  (struct sanim_node* node,
   const double rotations[3]); /* XYZ convention */

SANIM_API res_T
sanim_node_get_transform
  (struct sanim_node* node,
   double transform[12]);  /* 3x4 column major matrix */

END_DECLS

#endif /* SANIM_H */
