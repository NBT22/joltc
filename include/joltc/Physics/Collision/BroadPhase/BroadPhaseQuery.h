//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_BROADPHASEQUERY_H
#define JOLTC_BROADPHASEQUERY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Collision/CastResult.h>
#include <joltc/enums.h>
#include <joltc/Geometry/AABox.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/ObjectLayer.h>

typedef float JPH_RayCastBodyCollectorCallback(void *context, const JPH_BroadPhaseCastResult *result);
typedef float JPH_CollideShapeBodyCollectorCallback(void *context, JPH_BodyID result);

typedef struct JPH_BroadPhaseQuery JPH_BroadPhaseQuery;

JPH_CAPI bool JPH_BroadPhaseQuery_CastRay(const JPH_BroadPhaseQuery *query,
                                          const Vector3 *origin,
                                          const Vector3 *direction,
                                          JPH_RayCastBodyCollectorCallback *callback,
                                          void *userData,
                                          const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                          const JPH_ObjectLayerFilter *objectLayerFilter);

JPH_CAPI bool JPH_BroadPhaseQuery_CastRay2(const JPH_BroadPhaseQuery *query,
                                           const Vector3 *origin,
                                           const Vector3 *direction,
                                           JPH_CollisionCollectorType collectorType,
                                           JPH_RayCastBodyResultCallback *callback,
                                           void *userData,
                                           const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                           const JPH_ObjectLayerFilter *objectLayerFilter);

JPH_CAPI bool JPH_BroadPhaseQuery_CollideAABox(const JPH_BroadPhaseQuery *query,
                                               const JPH_AABox *box,
                                               JPH_CollideShapeBodyCollectorCallback *callback,
                                               void *userData,
                                               const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                               const JPH_ObjectLayerFilter *objectLayerFilter);

JPH_CAPI bool JPH_BroadPhaseQuery_CollideSphere(const JPH_BroadPhaseQuery *query,
                                                const Vector3 *center,
                                                float radius,
                                                JPH_CollideShapeBodyCollectorCallback *callback,
                                                void *userData,
                                                const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                const JPH_ObjectLayerFilter *objectLayerFilter);

JPH_CAPI bool JPH_BroadPhaseQuery_CollidePoint(const JPH_BroadPhaseQuery *query,
                                               const Vector3 *point,
                                               JPH_CollideShapeBodyCollectorCallback *callback,
                                               void *userData,
                                               const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                               const JPH_ObjectLayerFilter *objectLayerFilter);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BROADPHASEQUERY_H
