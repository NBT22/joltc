//
// Created by NBT22 on 8/14/25.
//

#pragma once

#include <joltc/Math/Quat.h>
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>

static inline JPH::Quat ToJolt(const JPH_Quat *quat)
{
    return {(*quat)[0], (*quat)[1], (*quat)[2], (*quat)[3]};
}

static inline void FromJolt(const JPH::Quat &quat, JPH_Quat *result)
{
    memcpy(*result, quat.mValue.mF32, sizeof(JPH_Quat));
}
