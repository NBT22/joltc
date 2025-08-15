//
// Created by NBT22 on 8/14/25.
//

#pragma once

#include <joltc/Math/Quat.h>
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>

static inline JPH::Quat ToJolt(const JPH_Quat *quat)
{
    return {quat->x, quat->y, quat->z, quat->w};
}

static inline void FromJolt(const JPH::Quat &quat, JPH_Quat *result)
{
    result->x = quat.GetX();
    result->y = quat.GetY();
    result->z = quat.GetZ();
    result->w = quat.GetW();
}
