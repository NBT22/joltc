//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/enums.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ActiveEdgeMode.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>
#include <Jolt/Physics/Collision/CollectFacesMode.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Math/Vector3.hpp>

static inline JPH::ShapeCastSettings ToJolt(const JPH_ShapeCastSettings *settings)
{
    JPH::ShapeCastSettings result{};
    if (settings != nullptr)
    {
        result.mBackFaceModeTriangles = static_cast<JPH::EBackFaceMode>(settings->backFaceModeTriangles);
        result.mBackFaceModeConvex = static_cast<JPH::EBackFaceMode>(settings->backFaceModeConvex);
        result.mUseShrunkenShapeAndConvexRadius = settings->useShrunkenShapeAndConvexRadius;
        result.mReturnDeepestPoint = settings->returnDeepestPoint;
        result.mActiveEdgeMode = static_cast<JPH::EActiveEdgeMode>(settings->base.activeEdgeMode);
        result.mCollectFacesMode = static_cast<JPH::ECollectFacesMode>(settings->base.collectFacesMode);
        result.mCollisionTolerance = settings->base.collisionTolerance;
        result.mPenetrationTolerance = settings->base.penetrationTolerance;
        result.mActiveEdgeMovementDirection = ToJolt(settings->base.activeEdgeMovementDirection);
    }
    return result;
}

static inline JPH_ShapeCastSettings FromJolt(const JPH::ShapeCastSettings &joltSettings)
{
    JPH_ShapeCastSettings result{};
    result.backFaceModeTriangles = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeTriangles);
    result.backFaceModeConvex = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeConvex);
    result.useShrunkenShapeAndConvexRadius = joltSettings.mUseShrunkenShapeAndConvexRadius;
    result.returnDeepestPoint = joltSettings.mReturnDeepestPoint;
    result.base.activeEdgeMode = static_cast<JPH_ActiveEdgeMode>(joltSettings.mActiveEdgeMode);
    result.base.collectFacesMode = static_cast<JPH_CollectFacesMode>(joltSettings.mCollectFacesMode);
    result.base.collisionTolerance = joltSettings.mCollisionTolerance;
    result.base.penetrationTolerance = joltSettings.mPenetrationTolerance;
    FromJolt(joltSettings.mActiveEdgeMovementDirection, &result.base.activeEdgeMovementDirection);
    return result;
}
