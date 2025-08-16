//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Geometry/AABox.h>
#include <Jolt/Jolt.h>
#include <Jolt/Geometry/AABox.h>
#include <Math/Vector3.hpp>

static inline JPH::AABox ToJolt(const JPH_AABox *value)
{
    return {ToJolt(value->min), ToJolt(value->max)};
}

static inline void FromJolt(const JPH::AABox &value, JPH_AABox *result)
{
    FromJolt(value.mMin, &result->min);
    FromJolt(value.mMax, &result->max);
}
