//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Body/MassProperties.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Math/Mat44.hpp>

static inline JPH::MassProperties ToJolt(const JPH_MassProperties *properties)
{
    JPH::MassProperties result{};
    if (properties == nullptr)
    {
        return result;
    }
    result.mMass = properties->mass;
    result.mInertia = ToJolt(&properties->inertia);
    return result;
}

static inline void FromJolt(const JPH::MassProperties &jolt, JPH_MassProperties *result)
{
    result->mass = jolt.mMass;
    FromJolt(jolt.mInertia, &result->inertia);
}
