//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_CONSTANTS_H
#define JOLTC_CONSTANTS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/types.h>
#include <stdint.h>

#if !defined(__cpp_constexpr) && __STDC_VERSION__ < 202311L
#define constexpr const
#endif

static constexpr float JPH_DefaultCollisionTolerance = 1.0e-4f;
static constexpr float JPH_DefaultPenetrationTolerance = 1.0e-4f;
static constexpr float JPH_DefaultConvexRadius = 0.05f;
static constexpr float JPH_CapsuleProjectionSlop = 0.02f;
static constexpr int JPH_MaxPhysicsJobs = 2048;
static constexpr int JPH_MaxPhysicsBarriers = 8;
static constexpr uint32_t JPH_CollisionGroup_InvalidGroup = ~0u;
static constexpr uint32_t JPH_CollisionGroup_InvalidSubGroup = ~0u;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_CONSTANTS_H
