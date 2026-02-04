//
// Created by NBT22 on 2/3/26.
//

#include <joltc/enums.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Math/Vector3.hpp>

void JPH_CollideSettingsBase_Init(JPH_CollideSettingsBase *settings)
{
    JPH_ASSERT(settings);
    static const JPH::CollideSettingsBase joltSettings{};
    settings->activeEdgeMode = static_cast<JPH_ActiveEdgeMode>(joltSettings.mActiveEdgeMode);
    settings->collectFacesMode = static_cast<JPH_CollectFacesMode>(joltSettings.mCollectFacesMode);
    settings->collisionTolerance = joltSettings.mCollisionTolerance;
    settings->penetrationTolerance = joltSettings.mPenetrationTolerance;
    FromJolt(joltSettings.mActiveEdgeMovementDirection, &settings->activeEdgeMovementDirection);
}

void JPH_CollideShapeSettings_Init(JPH_CollideShapeSettings *settings)
{
    JPH_ASSERT(settings);
    static const JPH::CollideShapeSettings joltSettings{};
    settings->maxSeparationDistance = joltSettings.mMaxSeparationDistance;
    settings->backFaceMode = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceMode);
    JPH_CollideSettingsBase_Init(&settings->base);
}
