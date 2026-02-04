//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_SHAPECAST_H
#define JOLTC_SHAPECAST_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>

typedef struct JPH_ShapeCastResult
{
        Vector3 contactPointOn1;
        Vector3 contactPointOn2;
        Vector3 penetrationAxis;
        float penetrationDepth;
        JPH_SubShapeID subShapeID1;
        JPH_SubShapeID subShapeID2;
        JPH_BodyID bodyID2;
        float fraction;
        bool isBackFaceHit;
} JPH_ShapeCastResult;

typedef struct JPH_ShapeCastSettings
{
        JPH_CollideSettingsBase base; /* Inherits JPH_CollideSettingsBase */

        /// How backfacing triangles should be treated (should we report moving from back to front for triangle based shapes, e.g. for MeshShape/HeightFieldShape?)
        JPH_BackFaceMode backFaceModeTriangles /* = JPH_BackFaceMode_IgnoreBackFaces*/;

        /// How backfacing convex objects should be treated (should we report starting inside an object and moving out?)
        JPH_BackFaceMode backFaceModeConvex /* = JPH_BackFaceMode_IgnoreBackFaces*/;

        /// Indicates if we want to shrink the shape by the convex radius and then expand it again. This speeds up collision detection and gives a more accurate normal at the cost of a more 'rounded' shape.
        bool useShrunkenShapeAndConvexRadius /* = false*/;

        /// When true, and the shape is intersecting at the beginning of the cast (fraction = 0) then this will calculate the deepest penetration point (costing additional CPU time)
        bool returnDeepestPoint /* = false*/;
} JPH_ShapeCastSettings;

typedef void JPH_CastShapeResultCallback(void *context, const JPH_ShapeCastResult *result);

JPH_CAPI void JPH_ShapeCastSettings_Init(JPH_ShapeCastSettings *settings);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_SHAPECAST_H
