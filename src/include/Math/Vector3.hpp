//
// Created by NBT22 on 8/14/25.
//

#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <joltc/Math/Vector3.h>

static inline JPH::Vec3 ToJolt(const Vector3 &vec)
{
    return {vec.x, vec.y, vec.z};
}

static inline JPH::Vec3 ToJolt(const Vector3 *vec)
{
    return {vec->x, vec->y, vec->z};
}

static inline void FromJolt(const JPH::Vec3 &vec, Vector3 *result)
{
    result->x = vec.GetX();
    result->y = vec.GetY();
    result->z = vec.GetZ();
}
