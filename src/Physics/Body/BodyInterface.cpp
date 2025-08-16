//
// Created by NBT22 on 8/15/25.
//

#include <cstdint>
#include <joltc/enums.h>
#include <joltc/Math/Mat44.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/Body.h>
#include <joltc/Physics/Body/BodyCreationSettings.h>
#include <joltc/Physics/Body/BodyInterface.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <joltc/types.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/EActivation.h>
#include <Math/Mat44.hpp>
#include <Math/Quat.hpp>
#include <Math/RMat44.hpp> // NOLINT(*-include-cleaner)
#include <Math/Vector3.hpp>
#include <Physics/Body/Body.hpp>
#include <Physics/Body/BodyCreationSettings.hpp>
#include <Physics/Body/BodyInterface.hpp>
#include <Physics/Collision/CollisionGroup.hpp>
#include <Physics/Collision/PhysicsMaterial.hpp>
#include <Physics/Collision/Shape/Shape.hpp>
#include <Physics/SoftBody/SoftBodyCreationSettings.hpp>

JPH_Body *JPH_BodyInterface_CreateBody(JPH_BodyInterface *interface, const JPH_BodyCreationSettings *settings)
{
    return ToBody(AsBodyInterface(interface)->CreateBody(*AsBodyCreationSettings(settings)));
}

JPH_Body *JPH_BodyInterface_CreateBodyWithID(JPH_BodyInterface *interface,
                                             const JPH_BodyId bodyID,
                                             const JPH_BodyCreationSettings *settings)
{
    return ToBody(AsBodyInterface(interface)->CreateBodyWithID(JPH::BodyID(bodyID), *AsBodyCreationSettings(settings)));
}

JPH_Body *JPH_BodyInterface_CreateBodyWithoutID(JPH_BodyInterface *interface, const JPH_BodyCreationSettings *settings)
{
    return ToBody(AsBodyInterface(interface)->CreateBodyWithoutID(*AsBodyCreationSettings(settings)));
}

void JPH_BodyInterface_DestroyBodyWithoutID(JPH_BodyInterface *interface, JPH_Body *body)
{
    AsBodyInterface(interface)->DestroyBodyWithoutID(AsBody(body));
}

bool JPH_BodyInterface_AssignBodyID(JPH_BodyInterface *interface, JPH_Body *body)
{
    return AsBodyInterface(interface)->AssignBodyID(AsBody(body));
}

bool JPH_BodyInterface_AssignBodyID2(JPH_BodyInterface *interface, JPH_Body *body, const JPH_BodyId bodyID)
{
    return AsBodyInterface(interface)->AssignBodyID(AsBody(body), JPH::BodyID(bodyID));
}

JPH_Body *JPH_BodyInterface_UnassignBodyID(JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return ToBody(AsBodyInterface(interface)->UnassignBodyID(JPH::BodyID(bodyID)));
}

JPH_BodyId JPH_BodyInterface_CreateAndAddBody(JPH_BodyInterface *interface,
                                              const JPH_BodyCreationSettings *settings,
                                              JPH_Activation activationMode)
{
    const JPH::BodyID
            bodyID = AsBodyInterface(interface)
                             ->CreateAndAddBody(*reinterpret_cast<const JPH::BodyCreationSettings *>(settings),
                                                static_cast<JPH::EActivation>(activationMode));

    return bodyID.GetIndexAndSequenceNumber();
}

void JPH_BodyInterface_DestroyBody(JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    AsBodyInterface(interface)->DestroyBody(JPH::BodyID(bodyID));
}

JPH_Body *JPH_BodyInterface_CreateSoftBody(JPH_BodyInterface *interface, const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(interface)->CreateSoftBody(*AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_Body *JPH_BodyInterface_CreateSoftBodyWithID(JPH_BodyInterface *interface,
                                                 const JPH_BodyId bodyID,
                                                 const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(interface)->CreateSoftBodyWithID(JPH::BodyID(bodyID),
                                                                       *AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_Body *JPH_BodyInterface_CreateSoftBodyWithoutID(JPH_BodyInterface *interface,
                                                    const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(interface)->CreateSoftBodyWithoutID(*AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_BodyId JPH_BodyInterface_CreateAndAddSoftBody(JPH_BodyInterface *interface,
                                                  const JPH_SoftBodyCreationSettings *settings,
                                                  JPH_Activation activationMode)
{
    const JPH::BodyID bodyID = AsBodyInterface(interface)
                                       ->CreateAndAddSoftBody(*AsSoftBodyCreationSettings(settings),
                                                              static_cast<JPH::EActivation>(activationMode));
    return bodyID.GetIndexAndSequenceNumber();
}

void JPH_BodyInterface_AddBody(JPH_BodyInterface *interface, JPH_BodyId bodyID, JPH_Activation activationMode)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(interface)->AddBody(joltBodyID, static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_RemoveBody(JPH_BodyInterface *interface, JPH_BodyId bodyID)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(interface)->RemoveBody(joltBodyID);
}

void JPH_BodyInterface_RemoveAndDestroyBody(JPH_BodyInterface *interface, JPH_BodyId bodyID)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(interface)->RemoveBody(joltBodyID);
    AsBodyInterface(interface)->DestroyBody(joltBodyID);
}

bool JPH_BodyInterface_IsActive(JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return AsBodyInterface(interface)->IsActive(JPH::BodyID(bodyID));
}

bool JPH_BodyInterface_IsAdded(JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return AsBodyInterface(interface)->IsAdded(JPH::BodyID(bodyID));
}

JPH_BodyType JPH_BodyInterface_GetBodyType(JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return static_cast<JPH_BodyType>(AsBodyInterface(interface)->GetBodyType(JPH::BodyID(bodyID)));
}

void JPH_BodyInterface_SetLinearVelocity(JPH_BodyInterface *interface, const JPH_BodyId bodyID, const Vector3 *velocity)
{
    AsBodyInterface(interface)->SetLinearVelocity(JPH::BodyID(bodyID), ToJolt(velocity));
}

void JPH_BodyInterface_GetLinearVelocity(JPH_BodyInterface *interface, const JPH_BodyId bodyID, Vector3 *velocity)
{
    FromJolt(AsBodyInterface(interface)->GetLinearVelocity(JPH::BodyID(bodyID)), velocity);
}

void JPH_BodyInterface_GetCenterOfMassPosition(JPH_BodyInterface *interface,
                                               const JPH_BodyId bodyID,
                                               JPH_RVec3 *position)
{
    FromJolt(AsBodyInterface(interface)->GetCenterOfMassPosition(JPH::BodyID(bodyID)), position);
}

JPH_MotionType JPH_BodyInterface_GetMotionType(JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return static_cast<JPH_MotionType>(AsBodyInterface(interface)->GetMotionType(JPH::BodyID(bodyID)));
}

void JPH_BodyInterface_SetMotionType(JPH_BodyInterface *interface,
                                     const JPH_BodyId bodyID,
                                     JPH_MotionType motionType,
                                     JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetMotionType(JPH::BodyID(bodyID),
                                              static_cast<JPH::EMotionType>(motionType),
                                              static_cast<JPH::EActivation>(activationMode));
}

float JPH_BodyInterface_GetRestitution(const JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return AsBodyInterface(interface)->GetRestitution(JPH::BodyID(bodyID));
}

void JPH_BodyInterface_SetRestitution(JPH_BodyInterface *interface, const JPH_BodyId bodyID, const float restitution)
{
    AsBodyInterface(interface)->SetRestitution(JPH::BodyID(bodyID), restitution);
}

float JPH_BodyInterface_GetFriction(const JPH_BodyInterface *interface, const JPH_BodyId bodyID)
{
    return AsBodyInterface(interface)->GetFriction(JPH::BodyID(bodyID));
}

void JPH_BodyInterface_SetFriction(JPH_BodyInterface *interface, const JPH_BodyId bodyID, const float friction)
{
    AsBodyInterface(interface)->SetFriction(JPH::BodyID(bodyID), friction);
}

void JPH_BodyInterface_SetPosition(JPH_BodyInterface *interface,
                                   const JPH_BodyId bodyId,
                                   const JPH_RVec3 *position,
                                   JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetPosition(JPH::BodyID(bodyId),
                                            ToJolt(position),
                                            static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetPosition(JPH_BodyInterface *interface, const JPH_BodyId bodyId, JPH_RVec3 *result)
{
    FromJolt(AsBodyInterface(interface)->GetPosition(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetRotation(JPH_BodyInterface *interface,
                                   const JPH_BodyId bodyId,
                                   const JPH_Quat *rotation,
                                   JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetRotation(JPH::BodyID(bodyId),
                                            ToJolt(rotation),
                                            static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetRotation(JPH_BodyInterface *interface, const JPH_BodyId bodyId, JPH_Quat *result)
{
    FromJolt(AsBodyInterface(interface)->GetRotation(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetPositionAndRotation(JPH_BodyInterface *interface,
                                              const JPH_BodyId bodyId,
                                              const JPH_RVec3 *position,
                                              const JPH_Quat *rotation,
                                              JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetPositionAndRotation(JPH::BodyID(bodyId),
                                                       ToJolt(position),
                                                       ToJolt(rotation),
                                                       static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_SetPositionAndRotationWhenChanged(JPH_BodyInterface *interface,
                                                         const JPH_BodyId bodyId,
                                                         const JPH_RVec3 *position,
                                                         const JPH_Quat *rotation,
                                                         JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetPositionAndRotationWhenChanged(JPH::BodyID(bodyId),
                                                                  ToJolt(position),
                                                                  ToJolt(rotation),
                                                                  static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetPositionAndRotation(JPH_BodyInterface *interface,
                                              const JPH_BodyId bodyId,
                                              JPH_RVec3 *position,
                                              JPH_Quat *rotation)
{
    JPH::RVec3 joltPosition;
    JPH::Quat joltRotation{};
    AsBodyInterface(interface)->GetPositionAndRotation(JPH::BodyID(bodyId), joltPosition, joltRotation);
    FromJolt(joltPosition, position);
    FromJolt(joltRotation, rotation);
}

void JPH_BodyInterface_SetPositionRotationAndVelocity(JPH_BodyInterface *interface,
                                                      const JPH_BodyId bodyId,
                                                      const JPH_RVec3 *position,
                                                      const JPH_Quat *rotation,
                                                      const Vector3 *linearVelocity,
                                                      const Vector3 *angularVelocity)
{
    AsBodyInterface(interface)->SetPositionRotationAndVelocity(JPH::BodyID(bodyId),
                                                               ToJolt(position),
                                                               ToJolt(rotation),
                                                               ToJolt(linearVelocity),
                                                               ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetCollisionGroup(JPH_BodyInterface *interface,
                                         const JPH_BodyId bodyId,
                                         JPH_CollisionGroup *result)
{
    FromJolt(AsBodyInterface(interface)->GetCollisionGroup(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetCollisionGroup(JPH_BodyInterface *interface,
                                         const JPH_BodyId bodyId,
                                         const JPH_CollisionGroup *group)
{
    AsBodyInterface(interface)->SetCollisionGroup(JPH::BodyID(bodyId), ToJolt(group));
}

const JPH_Shape *JPH_BodyInterface_GetShape(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    const JPH::Shape *shape = AsBodyInterface(interface)->GetShape(JPH::BodyID(bodyId)).GetPtr();
    return ToShape(shape);
}

void JPH_BodyInterface_SetShape(JPH_BodyInterface *interface,
                                const JPH_BodyId bodyId,
                                const JPH_Shape *shape,
                                const bool updateMassProperties,
                                JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetShape(JPH::BodyID(bodyId),
                                         AsShape(shape),
                                         updateMassProperties,
                                         static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_NotifyShapeChanged(JPH_BodyInterface *interface,
                                          const JPH_BodyId bodyId,
                                          const Vector3 *previousCenterOfMass,
                                          const bool updateMassProperties,
                                          JPH_Activation activationMode)
{
    AsBodyInterface(interface)->NotifyShapeChanged(JPH::BodyID(bodyId),
                                                   ToJolt(previousCenterOfMass),
                                                   updateMassProperties,
                                                   static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_ActivateBody(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    AsBodyInterface(interface)->ActivateBody(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_DeactivateBody(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    AsBodyInterface(interface)->DeactivateBody(JPH::BodyID(bodyId));
}

JPH_ObjectLayer JPH_BodyInterface_GetObjectLayer(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    return AsBodyInterface(interface)->GetObjectLayer(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetObjectLayer(JPH_BodyInterface *interface,
                                      const JPH_BodyId bodyId,
                                      const JPH_ObjectLayer layer)
{
    AsBodyInterface(interface)->SetObjectLayer(JPH::BodyID(bodyId), layer);
}

void JPH_BodyInterface_GetWorldTransform(JPH_BodyInterface *interface, const JPH_BodyId bodyId, JPH_RMat44 *result)
{
    const JPH::RMat44 &mat = AsBodyInterface(interface)->GetWorldTransform(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_GetCenterOfMassTransform(JPH_BodyInterface *interface,
                                                const JPH_BodyId bodyId,
                                                JPH_RMat44 *result)
{
    const JPH::RMat44 &mat = AsBodyInterface(interface)->GetCenterOfMassTransform(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_MoveKinematic(JPH_BodyInterface *interface,
                                     const JPH_BodyId bodyId,
                                     const JPH_RVec3 *targetPosition,
                                     const JPH_Quat *targetRotation,
                                     const float deltaTime)
{
    AsBodyInterface(interface)->MoveKinematic(JPH::BodyID(bodyId),
                                              ToJolt(targetPosition),
                                              ToJolt(targetRotation),
                                              deltaTime);
}

bool JPH_BodyInterface_ApplyBuoyancyImpulse(JPH_BodyInterface *interface,
                                            const JPH_BodyId bodyId,
                                            const JPH_RVec3 *surfacePosition,
                                            const Vector3 *surfaceNormal,
                                            const float buoyancy,
                                            const float linearDrag,
                                            const float angularDrag,
                                            const Vector3 *fluidVelocity,
                                            const Vector3 *gravity,
                                            const float deltaTime)
{
    return AsBodyInterface(interface)->ApplyBuoyancyImpulse(JPH::BodyID(bodyId),
                                                            ToJolt(surfacePosition),
                                                            ToJolt(surfaceNormal),
                                                            buoyancy,
                                                            linearDrag,
                                                            angularDrag,
                                                            ToJolt(fluidVelocity),
                                                            ToJolt(gravity),
                                                            deltaTime);
}

void JPH_BodyInterface_SetLinearAndAngularVelocity(JPH_BodyInterface *interface,
                                                   const JPH_BodyId bodyId,
                                                   const Vector3 *linearVelocity,
                                                   const Vector3 *angularVelocity)
{
    AsBodyInterface(interface)->SetLinearAndAngularVelocity(JPH::BodyID(bodyId),
                                                            ToJolt(linearVelocity),
                                                            ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetLinearAndAngularVelocity(JPH_BodyInterface *interface,
                                                   const JPH_BodyId bodyId,
                                                   Vector3 *linearVelocity,
                                                   Vector3 *angularVelocity)
{
    JPH::Vec3 linear{};
    JPH::Vec3 angular{};
    AsBodyInterface(interface)->GetLinearAndAngularVelocity(JPH::BodyID(bodyId), linear, angular);
    FromJolt(linear, linearVelocity);
    FromJolt(angular, angularVelocity);
}

void JPH_BodyInterface_AddLinearVelocity(JPH_BodyInterface *interface,
                                         const JPH_BodyId bodyId,
                                         const Vector3 *linearVelocity)
{
    AsBodyInterface(interface)->AddLinearVelocity(JPH::BodyID(bodyId), ToJolt(linearVelocity));
}

void JPH_BodyInterface_AddLinearAndAngularVelocity(JPH_BodyInterface *interface,
                                                   const JPH_BodyId bodyId,
                                                   const Vector3 *linearVelocity,
                                                   const Vector3 *angularVelocity)
{
    AsBodyInterface(interface)->AddLinearAndAngularVelocity(JPH::BodyID(bodyId),
                                                            ToJolt(linearVelocity),
                                                            ToJolt(angularVelocity));
}

void JPH_BodyInterface_SetAngularVelocity(JPH_BodyInterface *interface,
                                          const JPH_BodyId bodyId,
                                          const Vector3 *angularVelocity)
{
    AsBodyInterface(interface)->SetAngularVelocity(JPH::BodyID(bodyId), ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetAngularVelocity(JPH_BodyInterface *interface,
                                          const JPH_BodyId bodyId,
                                          Vector3 *angularVelocity)
{
    const JPH::Vec3 result = AsBodyInterface(interface)->GetAngularVelocity(JPH::BodyID(bodyId));
    FromJolt(result, angularVelocity);
}

void JPH_BodyInterface_GetPointVelocity(JPH_BodyInterface *interface,
                                        const JPH_BodyId bodyId,
                                        const JPH_RVec3 *point,
                                        Vector3 *velocity)
{
    const JPH::Vec3 result = AsBodyInterface(interface)->GetPointVelocity(JPH::BodyID(bodyId), ToJolt(point));
    FromJolt(result, velocity);
}

void JPH_BodyInterface_AddForce(JPH_BodyInterface *interface, const JPH_BodyId bodyId, const Vector3 *force)
{
    AsBodyInterface(interface)->AddForce(JPH::BodyID(bodyId), ToJolt(force));
}

void JPH_BodyInterface_AddForce2(JPH_BodyInterface *interface,
                                 const JPH_BodyId bodyId,
                                 const Vector3 *force,
                                 const JPH_RVec3 *point)
{
    AsBodyInterface(interface)->AddForce(JPH::BodyID(bodyId), ToJolt(force), ToJolt(point));
}

void JPH_BodyInterface_AddTorque(JPH_BodyInterface *interface, const JPH_BodyId bodyId, const Vector3 *torque)
{
    AsBodyInterface(interface)->AddTorque(JPH::BodyID(bodyId), ToJolt(torque));
}

void JPH_BodyInterface_AddForceAndTorque(JPH_BodyInterface *interface,
                                         const JPH_BodyId bodyId,
                                         const Vector3 *force,
                                         const Vector3 *torque)
{
    AsBodyInterface(interface)->AddForceAndTorque(JPH::BodyID(bodyId), ToJolt(force), ToJolt(torque));
}

void JPH_BodyInterface_AddImpulse(JPH_BodyInterface *interface, const JPH_BodyId bodyId, const Vector3 *impulse)
{
    AsBodyInterface(interface)->AddImpulse(JPH::BodyID(bodyId), ToJolt(impulse));
}

void JPH_BodyInterface_AddImpulse2(JPH_BodyInterface *interface,
                                   const JPH_BodyId bodyId,
                                   const Vector3 *impulse,
                                   const JPH_RVec3 *point)
{
    AsBodyInterface(interface)->AddImpulse(JPH::BodyID(bodyId), ToJolt(impulse), ToJolt(point));
}

void JPH_BodyInterface_AddAngularImpulse(JPH_BodyInterface *interface,
                                         const JPH_BodyId bodyId,
                                         const Vector3 *angularImpulse)
{
    AsBodyInterface(interface)->AddAngularImpulse(JPH::BodyID(bodyId), ToJolt(angularImpulse));
}

void JPH_BodyInterface_SetMotionQuality(JPH_BodyInterface *interface,
                                        const JPH_BodyId bodyId,
                                        JPH_MotionQuality quality)
{
    AsBodyInterface(interface)->SetMotionQuality(JPH::BodyID(bodyId), static_cast<JPH::EMotionQuality>(quality));
}

JPH_MotionQuality JPH_BodyInterface_GetMotionQuality(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    return static_cast<JPH_MotionQuality>(AsBodyInterface(interface)->GetMotionQuality(JPH::BodyID(bodyId)));
}

void JPH_BodyInterface_GetInverseInertia(JPH_BodyInterface *interface, const JPH_BodyId bodyId, JPH_Mat44 *result)
{
    const JPH::Mat44 &mat = AsBodyInterface(interface)->GetInverseInertia(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_SetGravityFactor(JPH_BodyInterface *interface, const JPH_BodyId bodyId, const float value)
{
    AsBodyInterface(interface)->SetGravityFactor(JPH::BodyID(bodyId), value);
}

float JPH_BodyInterface_GetGravityFactor(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    return AsBodyInterface(interface)->GetGravityFactor(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetUseManifoldReduction(JPH_BodyInterface *interface, const JPH_BodyId bodyId, const bool value)
{
    AsBodyInterface(interface)->SetUseManifoldReduction(JPH::BodyID(bodyId), value);
}

bool JPH_BodyInterface_GetUseManifoldReduction(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    return AsBodyInterface(interface)->GetUseManifoldReduction(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetUserData(JPH_BodyInterface *interface, const JPH_BodyId bodyId, const uint64_t userData)
{
    AsBodyInterface(interface)->SetUserData(JPH::BodyID(bodyId), userData);
}

uint64_t JPH_BodyInterface_GetUserData(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    return AsBodyInterface(interface)->GetUserData(JPH::BodyID(bodyId));
}

const JPH_PhysicsMaterial *JPH_BodyInterface_GetMaterial(JPH_BodyInterface *interface,
                                                         const JPH_BodyId bodyId,
                                                         const JPH_SubShapeId subShapeID)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    return ToPhysicsMaterial(AsBodyInterface(interface)->GetMaterial(JPH::BodyID(bodyId), joltSubShapeID));
}

void JPH_BodyInterface_InvalidateContactCache(JPH_BodyInterface *interface, const JPH_BodyId bodyId)
{
    AsBodyInterface(interface)->InvalidateContactCache(JPH::BodyID(bodyId));
}
