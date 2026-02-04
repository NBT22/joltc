//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_NARROWPHASEQUERY_H
#define JOLTC_NARROWPHASEQUERY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Transform.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/BodyFilter.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Body/BodyInterface.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/CastResult.h>
#include <joltc/Physics/Collision/CollidePointResult.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/RayCast.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>

typedef struct JPH_NarrowPhaseQuery JPH_NarrowPhaseQuery;

JPH_CAPI bool JPH_NarrowPhaseQuery_CastRay_GAME(const JPH_NarrowPhaseQuery *query,
                                                const Transform *transform,
                                                float maxDistance,
                                                JPH_RayCastResult *result,
                                                const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                const JPH_ObjectLayerFilter *objectLayerFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CastRay2_GAME(const JPH_NarrowPhaseQuery *query,
                                                 const JPH_BodyInterface *bodyInterface,
                                                 JPH_BodyID bodyId,
                                                 float maxDistance,
                                                 JPH_RayCastResult *result,
                                                 Vector3 *hitPointOffset,
                                                 const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                 const JPH_ObjectLayerFilter *objectLayerFilter,
                                                 const JPH_BodyFilter *bodyFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CastRay(const JPH_NarrowPhaseQuery *query,
                                           const JPH_RVec3 *origin,
                                           const Vector3 *direction,
                                           JPH_RayCastResult *hit,
                                           const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                           const JPH_ObjectLayerFilter *objectLayerFilter,
                                           const JPH_BodyFilter *bodyFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CastRay2(const JPH_NarrowPhaseQuery *query,
                                            const JPH_RVec3 *origin,
                                            const Vector3 *direction,
                                            const JPH_RayCastSettings *rayCastSettings,
                                            JPH_CastRayCollectorCallback *callback,
                                            void *userData,
                                            const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                            const JPH_ObjectLayerFilter *objectLayerFilter,
                                            const JPH_BodyFilter *bodyFilter,
                                            const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CastRay3(const JPH_NarrowPhaseQuery *query,
                                            const JPH_RVec3 *origin,
                                            const Vector3 *direction,
                                            const JPH_RayCastSettings *rayCastSettings,
                                            JPH_CollisionCollectorType collectorType,
                                            JPH_CastRayResultCallback *callback,
                                            void *userData,
                                            const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                            const JPH_ObjectLayerFilter *objectLayerFilter,
                                            const JPH_BodyFilter *bodyFilter,
                                            const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CollidePoint(const JPH_NarrowPhaseQuery *query,
                                                const JPH_RVec3 *point,
                                                JPH_CollidePointCollectorCallback *callback,
                                                void *userData,
                                                const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                const JPH_ObjectLayerFilter *objectLayerFilter,
                                                const JPH_BodyFilter *bodyFilter,
                                                const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CollidePoint2(const JPH_NarrowPhaseQuery *query,
                                                 const JPH_RVec3 *point,
                                                 JPH_CollisionCollectorType collectorType,
                                                 JPH_CollidePointResultCallback *callback,
                                                 void *userData,
                                                 const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                 const JPH_ObjectLayerFilter *objectLayerFilter,
                                                 const JPH_BodyFilter *bodyFilter,
                                                 const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CollideShape(const JPH_NarrowPhaseQuery *query,
                                                const JPH_Shape *shape,
                                                const Vector3 *scale,
                                                const JPH_RMat44 *centerOfMassTransform,
                                                const JPH_CollideShapeSettings *settings,
                                                JPH_RVec3 *baseOffset,
                                                JPH_CollideShapeCollectorCallback *callback,
                                                void *userData,
                                                const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                const JPH_ObjectLayerFilter *objectLayerFilter,
                                                const JPH_BodyFilter *bodyFilter,
                                                const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CollideShape2(const JPH_NarrowPhaseQuery *query,
                                                 const JPH_Shape *shape,
                                                 const Vector3 *scale,
                                                 const JPH_RMat44 *centerOfMassTransform,
                                                 const JPH_CollideShapeSettings *settings,
                                                 JPH_RVec3 *baseOffset,
                                                 JPH_CollisionCollectorType collectorType,
                                                 JPH_CollideShapeResultCallback *callback,
                                                 void *userData,
                                                 const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                 const JPH_ObjectLayerFilter *objectLayerFilter,
                                                 const JPH_BodyFilter *bodyFilter,
                                                 const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CastShape(const JPH_NarrowPhaseQuery *query,
                                             const JPH_Shape *shape,
                                             const JPH_RMat44 *worldTransform,
                                             const Vector3 *direction,
                                             const JPH_ShapeCastSettings *settings,
                                             JPH_RVec3 *baseOffset,
                                             JPH_CastShapeCollectorCallback *callback,
                                             void *userData,
                                             const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                             const JPH_ObjectLayerFilter *objectLayerFilter,
                                             const JPH_BodyFilter *bodyFilter,
                                             const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_NarrowPhaseQuery_CastShape2(const JPH_NarrowPhaseQuery *query,
                                              const JPH_Shape *shape,
                                              const JPH_RMat44 *worldTransform,
                                              const Vector3 *direction,
                                              const JPH_ShapeCastSettings *settings,
                                              JPH_RVec3 *baseOffset,
                                              JPH_CollisionCollectorType collectorType,
                                              JPH_CastShapeResultCallback *callback,
                                              void *userData,
                                              const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                              const JPH_ObjectLayerFilter *objectLayerFilter,
                                              const JPH_BodyFilter *bodyFilter,
                                              const JPH_ShapeFilter *shapeFilter);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_NARROWPHASEQUERY_H
