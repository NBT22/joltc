//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_SHAPE_H
#define JOLTC_SHAPE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>
#include <joltc/Geometry/AABox.h>
#include <joltc/Math/Mat44.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/MassProperties.h>
#include <joltc/Physics/Collision/CastResult.h>
#include <joltc/Physics/Collision/CollidePointResult.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/RayCast.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct JPH_SupportingFace
{
        uint32_t count;
        Vector3 vertices[32];
} JPH_SupportingFace;

typedef float JPH_CastRayCollectorCallback(void *context, const JPH_RayCastResult *result);
typedef float JPH_CastShapeCollectorCallback(void *context, const JPH_ShapeCastResult *result);
typedef float JPH_CollidePointCollectorCallback(void *context, const JPH_CollidePointResult *result);
typedef float JPH_CollideShapeCollectorCallback(void *context, const JPH_CollideShapeResult *result);

typedef struct JPH_ShapeSettings JPH_ShapeSettings;
typedef struct JPH_Shape JPH_Shape;

JPH_CAPI void JPH_ShapeSettings_Destroy(JPH_ShapeSettings *settings);
JPH_CAPI uint64_t JPH_ShapeSettings_GetUserData(const JPH_ShapeSettings *settings);
JPH_CAPI void JPH_ShapeSettings_SetUserData(JPH_ShapeSettings *settings, uint64_t userData);

JPH_CAPI void JPH_Shape_Destroy(JPH_Shape *shape);
JPH_CAPI JPH_ShapeType JPH_Shape_GetType(const JPH_Shape *shape);
JPH_CAPI JPH_ShapeSubType JPH_Shape_GetSubType(const JPH_Shape *shape);
JPH_CAPI uint64_t JPH_Shape_GetUserData(const JPH_Shape *shape);
JPH_CAPI void JPH_Shape_SetUserData(JPH_Shape *shape, uint64_t userData);
JPH_CAPI bool JPH_Shape_MustBeStatic(const JPH_Shape *shape);
JPH_CAPI void JPH_Shape_GetCenterOfMass(const JPH_Shape *shape, Vector3 *result);
JPH_CAPI void JPH_Shape_GetLocalBounds(const JPH_Shape *shape, JPH_AABox *result);
JPH_CAPI uint32_t JPH_Shape_GetSubShapeIDBitsRecursive(const JPH_Shape *shape);
JPH_CAPI void JPH_Shape_GetWorldSpaceBounds(const JPH_Shape *shape,
                                            const JPH_RMat44 *centerOfMassTransform,
                                            const Vector3 *scale,
                                            JPH_AABox *result);
JPH_CAPI float JPH_Shape_GetInnerRadius(const JPH_Shape *shape);
JPH_CAPI void JPH_Shape_GetMassProperties(const JPH_Shape *shape, JPH_MassProperties *result);
JPH_CAPI const JPH_Shape *JPH_Shape_GetLeafShape(const JPH_Shape *shape,
                                                 JPH_SubShapeID subShapeID,
                                                 JPH_SubShapeID *remainder);
JPH_CAPI const JPH_PhysicsMaterial *JPH_Shape_GetMaterial(const JPH_Shape *shape, JPH_SubShapeID subShapeID);
JPH_CAPI void JPH_Shape_GetSurfaceNormal(const JPH_Shape *shape,
                                         JPH_SubShapeID subShapeID,
                                         const Vector3 *localPosition,
                                         Vector3 *normal);
JPH_CAPI void JPH_Shape_GetSupportingFace(const JPH_Shape *shape,
                                          JPH_SubShapeID subShapeID,
                                          const Vector3 *direction,
                                          const Vector3 *scale,
                                          const JPH_Mat44 *centerOfMassTransform,
                                          JPH_SupportingFace *outVertices);
JPH_CAPI float JPH_Shape_GetVolume(const JPH_Shape *shape);
JPH_CAPI bool JPH_Shape_IsValidScale(const JPH_Shape *shape, const Vector3 *scale);
JPH_CAPI void JPH_Shape_MakeScaleValid(const JPH_Shape *shape, const Vector3 *scale, Vector3 *result);
JPH_CAPI JPH_Shape *JPH_Shape_ScaleShape(const JPH_Shape *shape, const Vector3 *scale);
JPH_CAPI bool JPH_Shape_CastRay(const JPH_Shape *shape,
                                const Vector3 *origin,
                                const Vector3 *direction,
                                JPH_RayCastResult *hit);
JPH_CAPI bool JPH_Shape_CastRay2(const JPH_Shape *shape,
                                 const Vector3 *origin,
                                 const Vector3 *direction,
                                 const JPH_RayCastSettings *rayCastSettings,
                                 JPH_CollisionCollectorType collectorType,
                                 JPH_CastRayResultCallback *callback,
                                 void *userData,
                                 const JPH_ShapeFilter *shapeFilter);
JPH_CAPI bool JPH_Shape_CollidePoint(const JPH_Shape *shape, const Vector3 *point, const JPH_ShapeFilter *shapeFilter);
JPH_CAPI bool JPH_Shape_CollidePoint2(const JPH_Shape *shape,
                                      const Vector3 *point,
                                      JPH_CollisionCollectorType collectorType,
                                      JPH_CollidePointResultCallback *callback,
                                      void *userData,
                                      const JPH_ShapeFilter *shapeFilter);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_SHAPE_H
