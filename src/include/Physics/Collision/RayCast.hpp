//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/RayCast.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>
#include <Jolt/Physics/Collision/RayCast.h>

static inline JPH::RayCastSettings ToJolt(const JPH_RayCastSettings *settings)
{
    JPH::RayCastSettings result{};
    if (settings == nullptr)
    {
        return result;
    }
    result.mBackFaceModeTriangles = static_cast<JPH::EBackFaceMode>(settings->backFaceModeTriangles);
    result.mBackFaceModeConvex = static_cast<JPH::EBackFaceMode>(settings->backFaceModeConvex);
    result.mTreatConvexAsSolid = settings->treatConvexAsSolid;
    return result;
}
