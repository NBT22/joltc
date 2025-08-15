//
// Created by NBT22 on 8/14/25.
//

#pragma once

#ifdef JPH_DOUBLE_PRECISION
#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <joltc/Math/RVec3.h>

static inline JPH::RVec3 ToJolt(const JPH_RVec3 &vec)
{
    return {vec.x, vec.y, vec.z};
}

static inline JPH::RVec3 ToJolt(const JPH_RVec3 *vec)
{
    return {vec->x, vec->y, vec->z};
}

static inline void FromJolt(const JPH::RVec3 &vec, JPH_RVec3 *result)
{
    result->x = vec.GetX();
    result->y = vec.GetY();
    result->z = vec.GetZ();
}
#else
#include <Math/Vector3.hpp> // NOLINT(*-include-cleaner)
#endif
