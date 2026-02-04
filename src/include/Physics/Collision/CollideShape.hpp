//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/enums.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ActiveEdgeMode.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>
#include <Jolt/Physics/Collision/CollectFacesMode.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Math/Vector3.hpp>

static inline const JPH::CollideShapeResult &ToJolt(const JPH_CollideShapeResult *collideShapeResult)
{
    static const JPH::CollideShapeResult defaultResult{};
    return collideShapeResult != nullptr ? *reinterpret_cast<const JPH::CollideShapeResult *>(collideShapeResult)
                                         : defaultResult;
}

static inline void FromJolt(const JPH::CollideShapeResult *collideShapeResult, const JPH_CollideShapeResult *&result)
{
    result = reinterpret_cast<const JPH_CollideShapeResult *>(collideShapeResult);
}


static inline JPH::CollideSettingsBase ToJolt(const JPH_CollideSettingsBase *settings)
{
    JPH::CollideSettingsBase result{};
    if (settings != nullptr)
    {
        result.mActiveEdgeMode = static_cast<JPH::EActiveEdgeMode>(settings->activeEdgeMode);
        result.mCollectFacesMode = static_cast<JPH::ECollectFacesMode>(settings->collectFacesMode);
        result.mCollisionTolerance = settings->collisionTolerance;
        result.mPenetrationTolerance = settings->penetrationTolerance;
        result.mActiveEdgeMovementDirection = ToJolt(settings->activeEdgeMovementDirection);
    }
    return result;
}

static inline void FromJolt(const JPH::CollideSettingsBase &settings, JPH_CollideSettingsBase &result)
{
    result.activeEdgeMode = static_cast<JPH_ActiveEdgeMode>(settings.mActiveEdgeMode);
    result.collectFacesMode = static_cast<JPH_CollectFacesMode>(settings.mCollectFacesMode);
    result.collisionTolerance = settings.mCollisionTolerance;
    result.penetrationTolerance = settings.mPenetrationTolerance;
    FromJolt(settings.mActiveEdgeMovementDirection, &result.activeEdgeMovementDirection);
}


static inline JPH::CollideShapeSettings ToJolt(const JPH_CollideShapeSettings *settings)
{
    JPH::CollideShapeSettings result{};
    if (settings != nullptr)
    {
        result.mMaxSeparationDistance = settings->maxSeparationDistance;
        result.mBackFaceMode = static_cast<JPH::EBackFaceMode>(settings->backFaceMode);
        result.mActiveEdgeMode = static_cast<JPH::EActiveEdgeMode>(settings->base.activeEdgeMode);
        result.mCollectFacesMode = static_cast<JPH::ECollectFacesMode>(settings->base.collectFacesMode);
        result.mCollisionTolerance = settings->base.collisionTolerance;
        result.mPenetrationTolerance = settings->base.penetrationTolerance;
        result.mActiveEdgeMovementDirection = ToJolt(settings->base.activeEdgeMovementDirection);
    }
    return result;
}

static inline JPH_CollideShapeSettings FromJolt(const JPH::CollideShapeSettings &joltSettings)
{
    JPH_CollideShapeSettings result{};
    result.maxSeparationDistance = joltSettings.mMaxSeparationDistance;
    result.backFaceMode = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceMode);
    result.base.activeEdgeMode = static_cast<JPH_ActiveEdgeMode>(joltSettings.mActiveEdgeMode);
    result.base.collectFacesMode = static_cast<JPH_CollectFacesMode>(joltSettings.mCollectFacesMode);
    result.base.collisionTolerance = joltSettings.mCollisionTolerance;
    result.base.penetrationTolerance = joltSettings.mPenetrationTolerance;
    FromJolt(joltSettings.mActiveEdgeMovementDirection, &result.base.activeEdgeMovementDirection);
    return result;
}
