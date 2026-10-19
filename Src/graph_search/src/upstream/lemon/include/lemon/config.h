/* This file is a vendored configuration header for the LEMON Dijkstra
 * include set (S-04, OpenROAD copy at commit 31d79d686f776a62ace1925cb2e40e84a80d1394).
 *
 * The upstream CMake build generates this file. The values below reproduce the
 * probe configuration (`LEMON_VERSION "1.3.1"`, LP/MIP backends disabled) with
 * one documented deviation: LEMON_USE_PTHREAD is left undefined, so
 * lemon/bits/lock.h selects its no-op single-threaded Lock. The vendored
 * Dijkstra include set does not use the lock for any cross-thread state.
 *
 * Do not edit by hand; regenerate through the component's provenance process.
 * See ../../../PROVENANCE.md (Src/graph_search/src/upstream/lemon/PROVENANCE.md).
 */
#ifndef LEMON_CONFIG_H
#define LEMON_CONFIG_H

#define LEMON_VERSION "1.3.1"
#define LEMON_HAVE_LONG_LONG 1

/* #undef LEMON_HAVE_LP */
/* #undef LEMON_HAVE_MIP */
/* #undef LEMON_HAVE_GLPK */
/* #undef LEMON_HAVE_CPLEX */
/* #undef LEMON_HAVE_SOPLEX */
/* #undef LEMON_HAVE_CLP */
/* #undef LEMON_HAVE_CBC */

#define _LEMON_CPLEX 1
#define _LEMON_CLP 2
#define _LEMON_GLPK 3
#define _LEMON_SOPLEX 4
#define _LEMON_CBC 5

/* #undef LEMON_DEFAULT_LP */
/* #undef LEMON_DEFAULT_MIP */

/* #undef LEMON_USE_PTHREAD */
/* #undef LEMON_USE_WIN32_THREADS */

#endif /* LEMON_CONFIG_H */
