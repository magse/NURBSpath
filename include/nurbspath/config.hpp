#pragma once

/**
 * @file config.hpp
 * @brief Local nurbspath version and Git metadata.
 *
 * This source-tree header is the no-CMake fallback for release `v0.1.0`.
 * CMake generates a header with refreshed commit and worktree values on every
 * configure and installs that generated copy. The guarded Git-state macros
 * may be overridden before inclusion.
 */

/** @brief Library semantic-version major component. */
#define NURBSPATH_VERSION_MAJOR 0

/** @brief Library semantic-version minor component. */
#define NURBSPATH_VERSION_MINOR 1

/** @brief Library semantic-version patch component. */
#define NURBSPATH_VERSION_PATCH 0

/** @brief Complete library semantic-version string. */
#define NURBSPATH_VERSION_STRING "0.1.0"

/** @brief Integer library version encoded as `major*10000 + minor*100 + patch`. */
#define NURBSPATH_VERSION_NUMBER 100

#ifndef NURBSPATH_GIT_COMMIT
/** @brief Full Git commit hash, unavailable in the checked-in fallback. */
#define NURBSPATH_GIT_COMMIT "unavailable"
#endif

#ifndef NURBSPATH_GIT_COMMIT_SHORT
/** @brief Short Git commit hash, unavailable in the checked-in fallback. */
#define NURBSPATH_GIT_COMMIT_SHORT "unavailable"
#endif

#ifndef NURBSPATH_GIT_DESCRIBE
/** @brief Git tag/hash description including the dirty-worktree suffix. */
#define NURBSPATH_GIT_DESCRIBE "v0.1.0"
#endif

#ifndef NURBSPATH_GIT_DIRTY
/** @brief One when uncommitted or untracked repository changes were detected. */
#define NURBSPATH_GIT_DIRTY 0
#endif

#ifndef NURBSPATH_GIT_COMMIT_AVAILABLE
/** @brief One when this header contains a resolvable HEAD commit. */
#define NURBSPATH_GIT_COMMIT_AVAILABLE 0
#endif

#ifndef NURBSPATH_GIT_VERSION
/** @brief Library version combined with the current Git description. */
#define NURBSPATH_GIT_VERSION "0.1.0+v0.1.0"
#endif

#ifndef NURBSPATH_CONFIG_GENERATED
/** @brief One for the CMake-refreshed header and zero for this local fallback. */
#define NURBSPATH_CONFIG_GENERATED 0
#endif
