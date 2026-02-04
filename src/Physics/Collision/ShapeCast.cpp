//
// Created by NBT22 on 2/3/26.
//

#include <joltc/enums.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

void JPH_ShapeCastSettings_Init(JPH_ShapeCastSettings *settings)
{
    JPH_ASSERT(settings);
    static const JPH::ShapeCastSettings joltSettings{};
    settings->backFaceModeTriangles = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeTriangles);
    settings->backFaceModeConvex = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeConvex);
    settings->useShrunkenShapeAndConvexRadius = joltSettings.mUseShrunkenShapeAndConvexRadius;
    settings->returnDeepestPoint = joltSettings.mReturnDeepestPoint;
    JPH_CollideSettingsBase_Init(&settings->base);
}
