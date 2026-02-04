//
// Created by NBT22 on 8/15/25.
//

#include <cstdint>
#include <joltc/enums.h>
#include <joltc/Geometry/AABox.h>
#include <joltc/Math/Mat44.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/Body.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Body/MotionProperties.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Jolt.h>
#include <Geometry/AABox.hpp>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Math/Mat44.hpp>
#include <Math/Quat.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/Body.hpp>
#include <Physics/Body/MotionProperties.hpp>
#include <Physics/Collision/CollisionGroup.hpp>
#include <Physics/Collision/Shape/Shape.hpp>

JPH_BodyID JPH_Body_GetID(const JPH_Body *body)
{
    return AsBody(body)->GetID().GetIndexAndSequenceNumber();
}

JPH_BodyType JPH_Body_GetBodyType(const JPH_Body *body)
{
    return static_cast<JPH_BodyType>(AsBody(body)->GetBodyType());
}

bool JPH_Body_IsRigidBody(const JPH_Body *body)
{
    return AsBody(body)->IsRigidBody();
}

bool JPH_Body_IsSoftBody(const JPH_Body *body)
{
    return AsBody(body)->IsSoftBody();
}

bool JPH_Body_IsActive(const JPH_Body *body)
{
    return AsBody(body)->IsActive();
}

bool JPH_Body_IsStatic(const JPH_Body *body)
{
    return AsBody(body)->IsStatic();
}

bool JPH_Body_IsKinematic(const JPH_Body *body)
{
    return AsBody(body)->IsKinematic();
}

bool JPH_Body_IsDynamic(const JPH_Body *body)
{
    return AsBody(body)->IsDynamic();
}

bool JPH_Body_CanBeKinematicOrDynamic(const JPH_Body *body)
{
    return AsBody(body)->CanBeKinematicOrDynamic();
}

void JPH_Body_SetIsSensor(JPH_Body *body, const bool value)
{
    AsBody(body)->SetIsSensor(value);
}

bool JPH_Body_IsSensor(const JPH_Body *body)
{
    return AsBody(body)->IsSensor();
}

void JPH_Body_SetCollideKinematicVsNonDynamic(JPH_Body *body, const bool value)
{
    AsBody(body)->SetCollideKinematicVsNonDynamic(value);
}

bool JPH_Body_GetCollideKinematicVsNonDynamic(const JPH_Body *body)
{
    return AsBody(body)->GetCollideKinematicVsNonDynamic();
}

void JPH_Body_SetUseManifoldReduction(JPH_Body *body, const bool value)
{
    AsBody(body)->SetUseManifoldReduction(value);
}

bool JPH_Body_GetUseManifoldReduction(const JPH_Body *body)
{
    return AsBody(body)->GetUseManifoldReduction();
}

bool JPH_Body_GetUseManifoldReductionWithBody(const JPH_Body *body, const JPH_Body *other)
{
    return AsBody(body)->GetUseManifoldReductionWithBody(*AsBody(other));
}

void JPH_Body_SetApplyGyroscopicForce(JPH_Body *body, const bool value)
{
    AsBody(body)->SetApplyGyroscopicForce(value);
}

bool JPH_Body_GetApplyGyroscopicForce(const JPH_Body *body)
{
    return AsBody(body)->GetApplyGyroscopicForce();
}

void JPH_Body_SetEnhancedInternalEdgeRemoval(JPH_Body *body, const bool value)
{
    AsBody(body)->SetEnhancedInternalEdgeRemoval(value);
}

bool JPH_Body_GetEnhancedInternalEdgeRemoval(const JPH_Body *body)
{
    return AsBody(body)->GetEnhancedInternalEdgeRemoval();
}

bool JPH_Body_GetEnhancedInternalEdgeRemovalWithBody(const JPH_Body *body, const JPH_Body *other)
{
    return AsBody(body)->GetEnhancedInternalEdgeRemovalWithBody(*AsBody(other));
}

JPH_MotionType JPH_Body_GetMotionType(const JPH_Body *body)
{
    return static_cast<JPH_MotionType>(AsBody(body)->GetMotionType());
}

void JPH_Body_SetMotionType(JPH_Body *body, JPH_MotionType motionType)
{
    AsBody(body)->SetMotionType(static_cast<JPH::EMotionType>(motionType));
}

JPH_BroadPhaseLayer JPH_Body_GetBroadPhaseLayer(const JPH_Body *body)
{
    return static_cast<JPH_BroadPhaseLayer>(AsBody(body)->GetBroadPhaseLayer());
}

JPH_ObjectLayer JPH_Body_GetObjectLayer(const JPH_Body *body)
{
    return AsBody(body)->GetObjectLayer();
}

void JPH_Body_GetCollisionGroup(const JPH_Body *body, JPH_CollisionGroup *result)
{
    FromJolt(AsBody(body)->GetCollisionGroup(), result);
}

void JPH_Body_SetCollisionGroup(JPH_Body *body, const JPH_CollisionGroup *value)
{
    AsBody(body)->SetCollisionGroup(ToJolt(value));
}

bool JPH_Body_GetAllowSleeping(JPH_Body *body)
{
    return AsBody(body)->GetAllowSleeping();
}

void JPH_Body_SetAllowSleeping(JPH_Body *body, const bool allowSleeping)
{
    AsBody(body)->SetAllowSleeping(allowSleeping);
}

void JPH_Body_ResetSleepTimer(JPH_Body *body)
{
    AsBody(body)->ResetSleepTimer();
}

float JPH_Body_GetFriction(const JPH_Body *body)
{
    return AsBody(body)->GetFriction();
}

void JPH_Body_SetFriction(JPH_Body *body, const float friction)
{
    AsBody(body)->SetFriction(friction);
}

float JPH_Body_GetRestitution(const JPH_Body *body)
{
    return reinterpret_cast<const JPH::Body *>(body)->GetRestitution();
}

void JPH_Body_SetRestitution(JPH_Body *body, const float restitution)
{
    reinterpret_cast<JPH::Body *>(body)->SetRestitution(restitution);
}

void JPH_Body_GetLinearVelocity(JPH_Body *body, Vector3 *velocity)
{
    const JPH::Vec3 joltVector = reinterpret_cast<JPH::Body *>(body)->GetLinearVelocity();
    FromJolt(joltVector, velocity);
}

void JPH_Body_SetLinearVelocity(JPH_Body *body, const Vector3 *velocity)
{
    reinterpret_cast<JPH::Body *>(body)->SetLinearVelocity(ToJolt(velocity));
}

void JPH_Body_SetLinearVelocityClamped(JPH_Body *body, const Vector3 *velocity)
{
    reinterpret_cast<JPH::Body *>(body)->SetLinearVelocityClamped(ToJolt(velocity));
}

void JPH_Body_GetAngularVelocity(JPH_Body *body, Vector3 *velocity)
{
    const JPH::Vec3 joltVector = reinterpret_cast<JPH::Body *>(body)->GetAngularVelocity();
    FromJolt(joltVector, velocity);
}

void JPH_Body_SetAngularVelocity(JPH_Body *body, const Vector3 *velocity)
{
    AsBody(body)->SetAngularVelocity(ToJolt(velocity));
}

void JPH_Body_SetAngularVelocityClamped(JPH_Body *body, const Vector3 *velocity)
{
    AsBody(body)->SetAngularVelocityClamped(ToJolt(velocity));
}

void JPH_Body_GetPointVelocityCOM(JPH_Body *body, const Vector3 *pointRelativeToCOM, Vector3 *velocity)
{
    FromJolt(AsBody(body)->GetPointVelocityCOM(ToJolt(pointRelativeToCOM)), velocity);
}

void JPH_Body_GetPointVelocity(JPH_Body *body, const JPH_RVec3 *point, Vector3 *velocity)
{
    FromJolt(AsBody(body)->GetPointVelocity(ToJolt(point)), velocity);
}

void JPH_Body_AddForce(JPH_Body *body, const Vector3 *force)
{
    AsBody(body)->AddForce(ToJolt(force));
}

void JPH_Body_AddForceAtPosition(JPH_Body *body, const Vector3 *force, const JPH_RVec3 *position)
{
    AsBody(body)->AddForce(ToJolt(force), ToJolt(position));
}

void JPH_Body_AddTorque(JPH_Body *body, const Vector3 *force)
{
    AsBody(body)->AddTorque(ToJolt(force));
}

void JPH_Body_GetAccumulatedForce(JPH_Body *body, Vector3 *force)
{
    const JPH::Vec3 joltVector = AsBody(body)->GetAccumulatedForce();
    FromJolt(joltVector, force);
}

void JPH_Body_GetAccumulatedTorque(JPH_Body *body, Vector3 *force)
{
    const JPH::Vec3 joltVector = AsBody(body)->GetAccumulatedTorque();
    FromJolt(joltVector, force);
}

void JPH_Body_ResetForce(JPH_Body *body)
{
    AsBody(body)->ResetForce();
}

void JPH_Body_ResetTorque(JPH_Body *body)
{
    AsBody(body)->ResetTorque();
}

void JPH_Body_ResetMotion(JPH_Body *body)
{
    AsBody(body)->ResetMotion();
}

void JPH_Body_GetInverseInertia(JPH_Body *body, JPH_Mat44 *result)
{
    FromJolt(AsBody(body)->GetInverseInertia(), result);
}

void JPH_Body_AddImpulse(JPH_Body *body, const Vector3 *impulse)
{
    AsBody(body)->AddImpulse(ToJolt(impulse));
}

void JPH_Body_AddImpulseAtPosition(JPH_Body *body, const Vector3 *impulse, const JPH_RVec3 *position)
{
    AsBody(body)->AddImpulse(ToJolt(impulse), ToJolt(position));
}

void JPH_Body_AddAngularImpulse(JPH_Body *body, const Vector3 *angularImpulse)
{
    AsBody(body)->AddAngularImpulse(ToJolt(angularImpulse));
}

void JPH_Body_MoveKinematic(JPH_Body *body,
                            const JPH_RVec3 *targetPosition,
                            const JPH_Quat *targetRotation,
                            const float deltaTime)
{
    AsBody(body)->MoveKinematic(ToJolt(targetPosition), ToJolt(targetRotation), deltaTime);
}

bool JPH_Body_ApplyBuoyancyImpulse(JPH_Body *body,
                                   const JPH_RVec3 *surfacePosition,
                                   const Vector3 *surfaceNormal,
                                   const float buoyancy,
                                   const float linearDrag,
                                   const float angularDrag,
                                   const Vector3 *fluidVelocity,
                                   const Vector3 *gravity,
                                   const float deltaTime)
{
    return AsBody(body)->ApplyBuoyancyImpulse(ToJolt(surfacePosition),
                                              ToJolt(surfaceNormal),
                                              buoyancy,
                                              linearDrag,
                                              angularDrag,
                                              ToJolt(fluidVelocity),
                                              ToJolt(gravity),
                                              deltaTime);
}

bool JPH_Body_IsInBroadPhase(JPH_Body *body)
{
    return AsBody(body)->IsInBroadPhase();
}

bool JPH_Body_IsCollisionCacheInvalid(JPH_Body *body)
{
    return AsBody(body)->IsCollisionCacheInvalid();
}

const JPH_Shape *JPH_Body_GetShape(JPH_Body *body)
{
    return ToShape(AsBody(body)->GetShape());
}

void JPH_Body_GetPosition(const JPH_Body *body, JPH_RVec3 *result)
{
    FromJolt(AsBody(body)->GetPosition(), result);
}

void JPH_Body_GetRotation(const JPH_Body *body, JPH_Quat *result)
{
    FromJolt(AsBody(body)->GetRotation(), result);
}

void JPH_Body_GetWorldTransform(const JPH_Body *body, JPH_RMat44 *result)
{
    FromJolt(AsBody(body)->GetWorldTransform(), result);
}

void JPH_Body_GetCenterOfMassPosition(const JPH_Body *body, JPH_RVec3 *result)
{
    FromJolt(AsBody(body)->GetCenterOfMassPosition(), result);
}

void JPH_Body_GetCenterOfMassTransform(const JPH_Body *body, JPH_RMat44 *result)
{
    FromJolt(AsBody(body)->GetCenterOfMassTransform(), result);
}

void JPH_Body_GetInverseCenterOfMassTransform(const JPH_Body *body, JPH_RMat44 *result)
{
    FromJolt(AsBody(body)->GetInverseCenterOfMassTransform(), result);
}

void JPH_Body_GetWorldSpaceBounds(const JPH_Body *body, JPH_AABox *result)
{
    FromJolt(AsBody(body)->GetWorldSpaceBounds(), result);
}

void JPH_Body_GetWorldSpaceSurfaceNormal(const JPH_Body *body,
                                         const JPH_SubShapeID subShapeID,
                                         const JPH_RVec3 *position,
                                         Vector3 *normal)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    const JPH::Vec3 joltNormal = AsBody(body)->GetWorldSpaceSurfaceNormal(joltSubShapeID, ToJolt(position));
    FromJolt(joltNormal, normal);
}

JPH_MotionProperties *JPH_Body_GetMotionProperties(JPH_Body *body)
{
    return ToMotionProperties(AsBody(body)->GetMotionProperties());
}

JPH_MotionProperties *JPH_Body_GetMotionPropertiesUnchecked(JPH_Body *body)
{
    return ToMotionProperties(AsBody(body)->GetMotionPropertiesUnchecked());
}

void JPH_Body_SetUserData(JPH_Body *body, const uint64_t userData)
{
    AsBody(body)->SetUserData(userData);
}

uint64_t JPH_Body_GetUserData(const JPH_Body *body)
{
    return AsBody(body)->GetUserData();
}

JPH_Body *JPH_Body_GetFixedToWorldBody()
{
    return ToBody(&JPH::Body::sFixedToWorld);
}
