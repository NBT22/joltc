//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_BODY_H
#define JOLTC_BODY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Geometry/AABox.h>
#include <joltc/Math/Mat44.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/MotionProperties.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/types.h>
#include <joltc/enums.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct JPH_Body JPH_Body;

JPH_CAPI JPH_BodyId JPH_Body_GetID(const JPH_Body *body);
JPH_CAPI JPH_BodyType JPH_Body_GetBodyType(const JPH_Body *body);
JPH_CAPI bool JPH_Body_IsRigidBody(const JPH_Body *body);
JPH_CAPI bool JPH_Body_IsSoftBody(const JPH_Body *body);
JPH_CAPI bool JPH_Body_IsActive(const JPH_Body *body);
JPH_CAPI bool JPH_Body_IsStatic(const JPH_Body *body);
JPH_CAPI bool JPH_Body_IsKinematic(const JPH_Body *body);
JPH_CAPI bool JPH_Body_IsDynamic(const JPH_Body *body);
JPH_CAPI bool JPH_Body_CanBeKinematicOrDynamic(const JPH_Body *body);

JPH_CAPI void JPH_Body_SetIsSensor(JPH_Body *body, bool value);
JPH_CAPI bool JPH_Body_IsSensor(const JPH_Body *body);

JPH_CAPI void JPH_Body_SetCollideKinematicVsNonDynamic(JPH_Body *body, bool value);
JPH_CAPI bool JPH_Body_GetCollideKinematicVsNonDynamic(const JPH_Body *body);

JPH_CAPI void JPH_Body_SetUseManifoldReduction(JPH_Body *body, bool value);
JPH_CAPI bool JPH_Body_GetUseManifoldReduction(const JPH_Body *body);
JPH_CAPI bool JPH_Body_GetUseManifoldReductionWithBody(const JPH_Body *body, const JPH_Body *other);

JPH_CAPI void JPH_Body_SetApplyGyroscopicForce(JPH_Body *body, bool value);
JPH_CAPI bool JPH_Body_GetApplyGyroscopicForce(const JPH_Body *body);

JPH_CAPI void JPH_Body_SetEnhancedInternalEdgeRemoval(JPH_Body *body, bool value);
JPH_CAPI bool JPH_Body_GetEnhancedInternalEdgeRemoval(const JPH_Body *body);
JPH_CAPI bool JPH_Body_GetEnhancedInternalEdgeRemovalWithBody(const JPH_Body *body, const JPH_Body *other);

JPH_CAPI JPH_MotionType JPH_Body_GetMotionType(const JPH_Body *body);
JPH_CAPI void JPH_Body_SetMotionType(JPH_Body *body, JPH_MotionType motionType);

JPH_CAPI JPH_BroadPhaseLayer JPH_Body_GetBroadPhaseLayer(const JPH_Body *body);
JPH_CAPI JPH_ObjectLayer JPH_Body_GetObjectLayer(const JPH_Body *body);

JPH_CAPI void JPH_Body_GetCollisionGroup(const JPH_Body *body, JPH_CollisionGroup *result);
JPH_CAPI void JPH_Body_SetCollisionGroup(JPH_Body *body, const JPH_CollisionGroup *value);

JPH_CAPI bool JPH_Body_GetAllowSleeping(JPH_Body *body);
JPH_CAPI void JPH_Body_SetAllowSleeping(JPH_Body *body, bool allowSleeping);
JPH_CAPI void JPH_Body_ResetSleepTimer(JPH_Body *body);

JPH_CAPI float JPH_Body_GetFriction(const JPH_Body *body);
JPH_CAPI void JPH_Body_SetFriction(JPH_Body *body, float friction);
JPH_CAPI float JPH_Body_GetRestitution(const JPH_Body *body);
JPH_CAPI void JPH_Body_SetRestitution(JPH_Body *body, float restitution);
JPH_CAPI void JPH_Body_GetLinearVelocity(JPH_Body *body, Vector3 *velocity);
JPH_CAPI void JPH_Body_SetLinearVelocity(JPH_Body *body, const Vector3 *velocity);
JPH_CAPI void JPH_Body_SetLinearVelocityClamped(JPH_Body *body, const Vector3 *velocity);
JPH_CAPI void JPH_Body_GetAngularVelocity(JPH_Body *body, Vector3 *velocity);
JPH_CAPI void JPH_Body_SetAngularVelocity(JPH_Body *body, const Vector3 *velocity);
JPH_CAPI void JPH_Body_SetAngularVelocityClamped(JPH_Body *body, const Vector3 *velocity);

JPH_CAPI void JPH_Body_GetPointVelocityCOM(JPH_Body *body, const Vector3 *pointRelativeToCOM, Vector3 *velocity);
JPH_CAPI void JPH_Body_GetPointVelocity(JPH_Body *body, const JPH_RVec3 *point, Vector3 *velocity);

JPH_CAPI void JPH_Body_AddForce(JPH_Body *body, const Vector3 *force);
JPH_CAPI void JPH_Body_AddForceAtPosition(JPH_Body *body, const Vector3 *force, const JPH_RVec3 *position);
JPH_CAPI void JPH_Body_AddTorque(JPH_Body *body, const Vector3 *force);
JPH_CAPI void JPH_Body_GetAccumulatedForce(JPH_Body *body, Vector3 *force);
JPH_CAPI void JPH_Body_GetAccumulatedTorque(JPH_Body *body, Vector3 *force);
JPH_CAPI void JPH_Body_ResetForce(JPH_Body *body);
JPH_CAPI void JPH_Body_ResetTorque(JPH_Body *body);
JPH_CAPI void JPH_Body_ResetMotion(JPH_Body *body);

JPH_CAPI void JPH_Body_GetInverseInertia(JPH_Body *body, JPH_Mat44 *result);

JPH_CAPI void JPH_Body_AddImpulse(JPH_Body *body, const Vector3 *impulse);
JPH_CAPI void JPH_Body_AddImpulseAtPosition(JPH_Body *body, const Vector3 *impulse, const JPH_RVec3 *position);
JPH_CAPI void JPH_Body_AddAngularImpulse(JPH_Body *body, const Vector3 *angularImpulse);
JPH_CAPI void JPH_Body_MoveKinematic(JPH_Body *body,
                                     const JPH_RVec3 *targetPosition,
                                     const JPH_Quat *targetRotation,
                                     float deltaTime);
JPH_CAPI bool JPH_Body_ApplyBuoyancyImpulse(JPH_Body *body,
                                            const JPH_RVec3 *surfacePosition,
                                            const Vector3 *surfaceNormal,
                                            float buoyancy,
                                            float linearDrag,
                                            float angularDrag,
                                            const Vector3 *fluidVelocity,
                                            const Vector3 *gravity,
                                            float deltaTime);

JPH_CAPI bool JPH_Body_IsInBroadPhase(JPH_Body *body);
JPH_CAPI bool JPH_Body_IsCollisionCacheInvalid(JPH_Body *body);

JPH_CAPI const JPH_Shape *JPH_Body_GetShape(JPH_Body *body);

JPH_CAPI void JPH_Body_GetPosition(const JPH_Body *body, JPH_RVec3 *result);
JPH_CAPI void JPH_Body_GetRotation(const JPH_Body *body, JPH_Quat *result);
JPH_CAPI void JPH_Body_GetWorldTransform(const JPH_Body *body, JPH_RMat44 *result);
JPH_CAPI void JPH_Body_GetCenterOfMassPosition(const JPH_Body *body, JPH_RVec3 *result);
JPH_CAPI void JPH_Body_GetCenterOfMassTransform(const JPH_Body *body, JPH_RMat44 *result);
JPH_CAPI void JPH_Body_GetInverseCenterOfMassTransform(const JPH_Body *body, JPH_RMat44 *result);

JPH_CAPI void JPH_Body_GetWorldSpaceBounds(const JPH_Body *body, JPH_AABox *result);
JPH_CAPI void JPH_Body_GetWorldSpaceSurfaceNormal(const JPH_Body *body,
                                                  JPH_SubShapeId subShapeID,
                                                  const JPH_RVec3 *position,
                                                  Vector3 *normal);

JPH_CAPI JPH_MotionProperties *JPH_Body_GetMotionProperties(JPH_Body *body);
JPH_CAPI JPH_MotionProperties *JPH_Body_GetMotionPropertiesUnchecked(JPH_Body *body);

JPH_CAPI void JPH_Body_SetUserData(JPH_Body *body, uint64_t userData);
JPH_CAPI uint64_t JPH_Body_GetUserData(const JPH_Body *body);

JPH_CAPI JPH_Body *JPH_Body_GetFixedToWorldBody(void);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BODY_H
