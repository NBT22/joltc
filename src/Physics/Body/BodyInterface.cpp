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
#include <joltc/Physics/Body/BodyCreationSettings.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Body/BodyInterface.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <joltc/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Jolt.h>
#include <Geometry/AABox.hpp>
#include <Jolt/Core/Array.h>
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
#include <Math/RMat44.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/Body.hpp>
#include <Physics/Body/BodyCreationSettings.hpp>
#include <Physics/Body/BodyInterface.hpp>
#include <Physics/Collision/BroadPhase/BroadPhaseLayer.hpp>
#include <Physics/Collision/CollisionGroup.hpp>
#include <Physics/Collision/ObjectLayer.hpp>
#include <Physics/Collision/PhysicsMaterial.hpp>
#include <Physics/Collision/Shape/Shape.hpp>
#include <Physics/SoftBody/SoftBodyCreationSettings.hpp>

JPH_Body *JPH_BodyInterface_CreateBody(JPH_BodyInterface *bodyInterface, const JPH_BodyCreationSettings *settings)
{
    return ToBody(AsBodyInterface(bodyInterface)->CreateBody(*AsBodyCreationSettings(settings)));
}

JPH_Body *JPH_BodyInterface_CreateBodyWithID(JPH_BodyInterface *bodyInterface,
                                             const JPH_BodyID bodyID,
                                             const JPH_BodyCreationSettings *settings)
{
    return ToBody(
            AsBodyInterface(bodyInterface)->CreateBodyWithID(JPH::BodyID(bodyID), *AsBodyCreationSettings(settings)));
}

JPH_Body *JPH_BodyInterface_CreateBodyWithoutID(JPH_BodyInterface *bodyInterface,
                                                const JPH_BodyCreationSettings *settings)
{
    return ToBody(AsBodyInterface(bodyInterface)->CreateBodyWithoutID(*AsBodyCreationSettings(settings)));
}

void JPH_BodyInterface_DestroyBodyWithoutID(JPH_BodyInterface *bodyInterface, JPH_Body *body)
{
    AsBodyInterface(bodyInterface)->DestroyBodyWithoutID(AsBody(body));
}

bool JPH_BodyInterface_AssignBodyID(JPH_BodyInterface *bodyInterface, JPH_Body *body)
{
    return AsBodyInterface(bodyInterface)->AssignBodyID(AsBody(body));
}

bool JPH_BodyInterface_AssignBodyID2(JPH_BodyInterface *bodyInterface, JPH_Body *body, const JPH_BodyID bodyID)
{
    return AsBodyInterface(bodyInterface)->AssignBodyID(AsBody(body), JPH::BodyID(bodyID));
}

JPH_Body *JPH_BodyInterface_UnassignBodyID(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return ToBody(AsBodyInterface(bodyInterface)->UnassignBodyID(JPH::BodyID(bodyID)));
}

JPH_BodyID JPH_BodyInterface_CreateAndAddBody(JPH_BodyInterface *bodyInterface,
                                              const JPH_BodyCreationSettings *settings,
                                              JPH_Activation activationMode)
{
    const JPH::BodyID bodyID =
            AsBodyInterface(bodyInterface)
                    ->CreateAndAddBody(*reinterpret_cast<const JPH::BodyCreationSettings *>(settings),
                                       static_cast<JPH::EActivation>(activationMode));

    return bodyID.GetIndexAndSequenceNumber();
}

void JPH_BodyInterface_DestroyBody(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    AsBodyInterface(bodyInterface)->DestroyBody(JPH::BodyID(bodyID));
}

JPH_Body *JPH_BodyInterface_CreateSoftBody(JPH_BodyInterface *bodyInterface,
                                           const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(bodyInterface)->CreateSoftBody(*AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_Body *JPH_BodyInterface_CreateSoftBodyWithID(JPH_BodyInterface *bodyInterface,
                                                 const JPH_BodyID bodyID,
                                                 const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(bodyInterface)
                              ->CreateSoftBodyWithID(JPH::BodyID(bodyID), *AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_Body *JPH_BodyInterface_CreateSoftBodyWithoutID(JPH_BodyInterface *bodyInterface,
                                                    const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(bodyInterface)->CreateSoftBodyWithoutID(*AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_BodyID JPH_BodyInterface_CreateAndAddSoftBody(JPH_BodyInterface *bodyInterface,
                                                  const JPH_SoftBodyCreationSettings *settings,
                                                  JPH_Activation activationMode)
{
    const JPH::BodyID bodyID = AsBodyInterface(bodyInterface)
                                       ->CreateAndAddSoftBody(*AsSoftBodyCreationSettings(settings),
                                                              static_cast<JPH::EActivation>(activationMode));
    return bodyID.GetIndexAndSequenceNumber();
}

void JPH_BodyInterface_AddBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID, JPH_Activation activationMode)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(bodyInterface)->AddBody(joltBodyID, static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_RemoveBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(bodyInterface)->RemoveBody(joltBodyID);
}

void JPH_BodyInterface_RemoveAndDestroyBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(bodyInterface)->RemoveBody(joltBodyID);
    AsBodyInterface(bodyInterface)->DestroyBody(joltBodyID);
}

bool JPH_BodyInterface_IsActive(const JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(bodyInterface)->IsActive(JPH::BodyID(bodyID));
}

bool JPH_BodyInterface_IsAdded(const JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(bodyInterface)->IsAdded(JPH::BodyID(bodyID));
}

JPH_BodyType JPH_BodyInterface_GetBodyType(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return static_cast<JPH_BodyType>(AsBodyInterface(bodyInterface)->GetBodyType(JPH::BodyID(bodyID)));
}

void JPH_BodyInterface_SetLinearVelocity(JPH_BodyInterface *bodyInterface,
                                         const JPH_BodyID bodyID,
                                         const Vector3 *velocity)
{
    AsBodyInterface(bodyInterface)->SetLinearVelocity(JPH::BodyID(bodyID), ToJolt(velocity));
}

void JPH_BodyInterface_GetLinearVelocity(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID, Vector3 *velocity)
{
    FromJolt(AsBodyInterface(bodyInterface)->GetLinearVelocity(JPH::BodyID(bodyID)), velocity);
}

void JPH_BodyInterface_GetCenterOfMassPosition(JPH_BodyInterface *bodyInterface,
                                               const JPH_BodyID bodyID,
                                               JPH_RVec3 *position)
{
    FromJolt(AsBodyInterface(bodyInterface)->GetCenterOfMassPosition(JPH::BodyID(bodyID)), position);
}

JPH_MotionType JPH_BodyInterface_GetMotionType(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return static_cast<JPH_MotionType>(AsBodyInterface(bodyInterface)->GetMotionType(JPH::BodyID(bodyID)));
}

void JPH_BodyInterface_SetMotionType(JPH_BodyInterface *bodyInterface,
                                     const JPH_BodyID bodyID,
                                     JPH_MotionType motionType,
                                     JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->SetMotionType(JPH::BodyID(bodyID),
                            static_cast<JPH::EMotionType>(motionType),
                            static_cast<JPH::EActivation>(activationMode));
}

float JPH_BodyInterface_GetRestitution(const JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(bodyInterface)->GetRestitution(JPH::BodyID(bodyID));
}

void JPH_BodyInterface_SetRestitution(JPH_BodyInterface *bodyInterface,
                                      const JPH_BodyID bodyID,
                                      const float restitution)
{
    AsBodyInterface(bodyInterface)->SetRestitution(JPH::BodyID(bodyID), restitution);
}

float JPH_BodyInterface_GetFriction(const JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(bodyInterface)->GetFriction(JPH::BodyID(bodyID));
}

void JPH_BodyInterface_SetFriction(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID, const float friction)
{
    AsBodyInterface(bodyInterface)->SetFriction(JPH::BodyID(bodyID), friction);
}

void JPH_BodyInterface_SetPosition(JPH_BodyInterface *bodyInterface,
                                   const JPH_BodyID bodyId,
                                   const JPH_RVec3 *position,
                                   JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->SetPosition(JPH::BodyID(bodyId), ToJolt(position), static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetPosition(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, JPH_RVec3 *result)
{
    FromJolt(AsBodyInterface(bodyInterface)->GetPosition(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetRotation(JPH_BodyInterface *bodyInterface,
                                   const JPH_BodyID bodyId,
                                   const JPH_Quat *rotation,
                                   JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->SetRotation(JPH::BodyID(bodyId), ToJolt(rotation), static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetRotation(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, JPH_Quat *result)
{
    FromJolt(AsBodyInterface(bodyInterface)->GetRotation(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetPositionAndRotation(JPH_BodyInterface *bodyInterface,
                                              const JPH_BodyID bodyId,
                                              const JPH_RVec3 *position,
                                              const JPH_Quat *rotation,
                                              JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->SetPositionAndRotation(JPH::BodyID(bodyId),
                                     ToJolt(position),
                                     ToJolt(rotation),
                                     static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_SetPositionAndRotationWhenChanged(JPH_BodyInterface *bodyInterface,
                                                         const JPH_BodyID bodyId,
                                                         const JPH_RVec3 *position,
                                                         const JPH_Quat *rotation,
                                                         JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->SetPositionAndRotationWhenChanged(JPH::BodyID(bodyId),
                                                ToJolt(position),
                                                ToJolt(rotation),
                                                static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetPositionAndRotation(JPH_BodyInterface *bodyInterface,
                                              const JPH_BodyID bodyId,
                                              JPH_RVec3 *position,
                                              JPH_Quat *rotation)
{
    JPH::RVec3 joltPosition;
    JPH::Quat joltRotation{};
    AsBodyInterface(bodyInterface)->GetPositionAndRotation(JPH::BodyID(bodyId), joltPosition, joltRotation);
    FromJolt(joltPosition, position);
    FromJolt(joltRotation, rotation);
}

void JPH_BodyInterface_SetPositionRotationAndVelocity(JPH_BodyInterface *bodyInterface,
                                                      const JPH_BodyID bodyId,
                                                      const JPH_RVec3 *position,
                                                      const JPH_Quat *rotation,
                                                      const Vector3 *linearVelocity,
                                                      const Vector3 *angularVelocity)
{
    AsBodyInterface(bodyInterface)
            ->SetPositionRotationAndVelocity(JPH::BodyID(bodyId),
                                             ToJolt(position),
                                             ToJolt(rotation),
                                             ToJolt(linearVelocity),
                                             ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetCollisionGroup(JPH_BodyInterface *bodyInterface,
                                         const JPH_BodyID bodyId,
                                         JPH_CollisionGroup *result)
{
    FromJolt(AsBodyInterface(bodyInterface)->GetCollisionGroup(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetCollisionGroup(JPH_BodyInterface *bodyInterface,
                                         const JPH_BodyID bodyId,
                                         const JPH_CollisionGroup *group)
{
    AsBodyInterface(bodyInterface)->SetCollisionGroup(JPH::BodyID(bodyId), ToJolt(group));
}

const JPH_Shape *JPH_BodyInterface_GetShape(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    const JPH::Shape *shape = AsBodyInterface(bodyInterface)->GetShape(JPH::BodyID(bodyId)).GetPtr();
    return ToShape(shape);
}

void JPH_BodyInterface_SetShape(JPH_BodyInterface *bodyInterface,
                                const JPH_BodyID bodyId,
                                const JPH_Shape *shape,
                                const bool updateMassProperties,
                                JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->SetShape(JPH::BodyID(bodyId),
                       AsShape(shape),
                       updateMassProperties,
                       static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_NotifyShapeChanged(JPH_BodyInterface *bodyInterface,
                                          const JPH_BodyID bodyId,
                                          const Vector3 *previousCenterOfMass,
                                          const bool updateMassProperties,
                                          JPH_Activation activationMode)
{
    AsBodyInterface(bodyInterface)
            ->NotifyShapeChanged(JPH::BodyID(bodyId),
                                 ToJolt(previousCenterOfMass),
                                 updateMassProperties,
                                 static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_ActivateBody(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    AsBodyInterface(bodyInterface)->ActivateBody(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_ActivateBodies(JPH_BodyInterface *bodyInterface, const JPH_BodyID *bodyIDs, const int count)
{
    JPH::Array<JPH::BodyID> joltBodyIDs;
    joltBodyIDs.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        joltBodyIDs.push_back(JPH::BodyID(bodyIDs[i]));
    }

    AsBodyInterface(bodyInterface)->ActivateBodies(joltBodyIDs.data(), count);
}

void JPH_BodyInterface_ActivateBodiesInAABox(JPH_BodyInterface *bodyInterface,
                                             const JPH_AABox *box,
                                             const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                             const JPH_ObjectLayerFilter *objectLayerFilter)
{
    AsBodyInterface(bodyInterface)
            ->ActivateBodiesInAABox(ToJolt(box), ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));
}

void JPH_BodyInterface_DeactivateBody(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    AsBodyInterface(bodyInterface)->DeactivateBody(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_DeactivateBodies(JPH_BodyInterface *bodyInterface, const JPH_BodyID *bodyIDs, const int count)
{
    JPH::Array<JPH::BodyID> joltBodyIDs;

    for (int i = 0; i < count; ++i)
    {
        joltBodyIDs.push_back(JPH::BodyID(bodyIDs[i]));
    }

    AsBodyInterface(bodyInterface)->DeactivateBodies(joltBodyIDs.data(), count);
}

void JPH_BodyInterface_ResetSleepTimer(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyID)
{
    AsBodyInterface(bodyInterface)->ResetSleepTimer(JPH::BodyID(bodyID));
}

JPH_ObjectLayer JPH_BodyInterface_GetObjectLayer(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(bodyInterface)->GetObjectLayer(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetObjectLayer(JPH_BodyInterface *bodyInterface,
                                      const JPH_BodyID bodyId,
                                      const JPH_ObjectLayer layer)
{
    AsBodyInterface(bodyInterface)->SetObjectLayer(JPH::BodyID(bodyId), layer);
}

void JPH_BodyInterface_GetWorldTransform(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, JPH_RMat44 *result)
{
    const JPH::RMat44 &mat = AsBodyInterface(bodyInterface)->GetWorldTransform(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_GetCenterOfMassTransform(JPH_BodyInterface *bodyInterface,
                                                const JPH_BodyID bodyId,
                                                JPH_RMat44 *result)
{
    const JPH::RMat44 &mat = AsBodyInterface(bodyInterface)->GetCenterOfMassTransform(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_MoveKinematic(JPH_BodyInterface *bodyInterface,
                                     const JPH_BodyID bodyId,
                                     const JPH_RVec3 *targetPosition,
                                     const JPH_Quat *targetRotation,
                                     const float deltaTime)
{
    AsBodyInterface(bodyInterface)
            ->MoveKinematic(JPH::BodyID(bodyId), ToJolt(targetPosition), ToJolt(targetRotation), deltaTime);
}

bool JPH_BodyInterface_ApplyBuoyancyImpulse(JPH_BodyInterface *bodyInterface,
                                            const JPH_BodyID bodyId,
                                            const JPH_RVec3 *surfacePosition,
                                            const Vector3 *surfaceNormal,
                                            const float buoyancy,
                                            const float linearDrag,
                                            const float angularDrag,
                                            const Vector3 *fluidVelocity,
                                            const Vector3 *gravity,
                                            const float deltaTime)
{
    return AsBodyInterface(bodyInterface)
            ->ApplyBuoyancyImpulse(JPH::BodyID(bodyId),
                                   ToJolt(surfacePosition),
                                   ToJolt(surfaceNormal),
                                   buoyancy,
                                   linearDrag,
                                   angularDrag,
                                   ToJolt(fluidVelocity),
                                   ToJolt(gravity),
                                   deltaTime);
}

void JPH_BodyInterface_SetLinearAndAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                   const JPH_BodyID bodyId,
                                                   const Vector3 *linearVelocity,
                                                   const Vector3 *angularVelocity)
{
    AsBodyInterface(bodyInterface)
            ->SetLinearAndAngularVelocity(JPH::BodyID(bodyId), ToJolt(linearVelocity), ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetLinearAndAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                   const JPH_BodyID bodyId,
                                                   Vector3 *linearVelocity,
                                                   Vector3 *angularVelocity)
{
    JPH::Vec3 linear{};
    JPH::Vec3 angular{};
    AsBodyInterface(bodyInterface)->GetLinearAndAngularVelocity(JPH::BodyID(bodyId), linear, angular);
    FromJolt(linear, linearVelocity);
    FromJolt(angular, angularVelocity);
}

void JPH_BodyInterface_AddLinearVelocity(JPH_BodyInterface *bodyInterface,
                                         const JPH_BodyID bodyId,
                                         const Vector3 *linearVelocity)
{
    AsBodyInterface(bodyInterface)->AddLinearVelocity(JPH::BodyID(bodyId), ToJolt(linearVelocity));
}

void JPH_BodyInterface_AddLinearAndAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                   const JPH_BodyID bodyId,
                                                   const Vector3 *linearVelocity,
                                                   const Vector3 *angularVelocity)
{
    AsBodyInterface(bodyInterface)
            ->AddLinearAndAngularVelocity(JPH::BodyID(bodyId), ToJolt(linearVelocity), ToJolt(angularVelocity));
}

void JPH_BodyInterface_SetAngularVelocity(JPH_BodyInterface *bodyInterface,
                                          const JPH_BodyID bodyId,
                                          const Vector3 *angularVelocity)
{
    AsBodyInterface(bodyInterface)->SetAngularVelocity(JPH::BodyID(bodyId), ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetAngularVelocity(JPH_BodyInterface *bodyInterface,
                                          const JPH_BodyID bodyId,
                                          Vector3 *angularVelocity)
{
    const JPH::Vec3 result = AsBodyInterface(bodyInterface)->GetAngularVelocity(JPH::BodyID(bodyId));
    FromJolt(result, angularVelocity);
}

void JPH_BodyInterface_GetPointVelocity(JPH_BodyInterface *bodyInterface,
                                        const JPH_BodyID bodyId,
                                        const JPH_RVec3 *point,
                                        Vector3 *velocity)
{
    const JPH::Vec3 result = AsBodyInterface(bodyInterface)->GetPointVelocity(JPH::BodyID(bodyId), ToJolt(point));
    FromJolt(result, velocity);
}

void JPH_BodyInterface_AddForce(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, const Vector3 *force)
{
    AsBodyInterface(bodyInterface)->AddForce(JPH::BodyID(bodyId), ToJolt(force));
}

void JPH_BodyInterface_AddForce2(JPH_BodyInterface *bodyInterface,
                                 const JPH_BodyID bodyId,
                                 const Vector3 *force,
                                 const JPH_RVec3 *point)
{
    AsBodyInterface(bodyInterface)->AddForce(JPH::BodyID(bodyId), ToJolt(force), ToJolt(point));
}

void JPH_BodyInterface_AddTorque(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, const Vector3 *torque)
{
    AsBodyInterface(bodyInterface)->AddTorque(JPH::BodyID(bodyId), ToJolt(torque));
}

void JPH_BodyInterface_AddForceAndTorque(JPH_BodyInterface *bodyInterface,
                                         const JPH_BodyID bodyId,
                                         const Vector3 *force,
                                         const Vector3 *torque)
{
    AsBodyInterface(bodyInterface)->AddForceAndTorque(JPH::BodyID(bodyId), ToJolt(force), ToJolt(torque));
}

void JPH_BodyInterface_AddImpulse(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, const Vector3 *impulse)
{
    AsBodyInterface(bodyInterface)->AddImpulse(JPH::BodyID(bodyId), ToJolt(impulse));
}

void JPH_BodyInterface_AddImpulse2(JPH_BodyInterface *bodyInterface,
                                   const JPH_BodyID bodyId,
                                   const Vector3 *impulse,
                                   const JPH_RVec3 *point)
{
    AsBodyInterface(bodyInterface)->AddImpulse(JPH::BodyID(bodyId), ToJolt(impulse), ToJolt(point));
}

void JPH_BodyInterface_AddAngularImpulse(JPH_BodyInterface *bodyInterface,
                                         const JPH_BodyID bodyId,
                                         const Vector3 *angularImpulse)
{
    AsBodyInterface(bodyInterface)->AddAngularImpulse(JPH::BodyID(bodyId), ToJolt(angularImpulse));
}

void JPH_BodyInterface_SetMotionQuality(JPH_BodyInterface *bodyInterface,
                                        const JPH_BodyID bodyId,
                                        JPH_MotionQuality quality)
{
    AsBodyInterface(bodyInterface)->SetMotionQuality(JPH::BodyID(bodyId), static_cast<JPH::EMotionQuality>(quality));
}

JPH_MotionQuality JPH_BodyInterface_GetMotionQuality(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    return static_cast<JPH_MotionQuality>(AsBodyInterface(bodyInterface)->GetMotionQuality(JPH::BodyID(bodyId)));
}

void JPH_BodyInterface_GetInverseInertia(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, JPH_Mat44 *result)
{
    const JPH::Mat44 &mat = AsBodyInterface(bodyInterface)->GetInverseInertia(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_SetGravityFactor(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, const float value)
{
    AsBodyInterface(bodyInterface)->SetGravityFactor(JPH::BodyID(bodyId), value);
}

float JPH_BodyInterface_GetGravityFactor(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(bodyInterface)->GetGravityFactor(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetUseManifoldReduction(JPH_BodyInterface *bodyInterface,
                                               const JPH_BodyID bodyId,
                                               const bool value)
{
    AsBodyInterface(bodyInterface)->SetUseManifoldReduction(JPH::BodyID(bodyId), value);
}

bool JPH_BodyInterface_GetUseManifoldReduction(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(bodyInterface)->GetUseManifoldReduction(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetUserData(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, const uint64_t userData)
{
    AsBodyInterface(bodyInterface)->SetUserData(JPH::BodyID(bodyId), userData);
}

uint64_t JPH_BodyInterface_GetUserData(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(bodyInterface)->GetUserData(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetIsSensor(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId, const bool value)
{
    AsBodyInterface(bodyInterface)->SetIsSensor(JPH::BodyID(bodyId), value);
}

bool JPH_BodyInterface_IsSensor(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(bodyInterface)->IsSensor(JPH::BodyID(bodyId));
}

const JPH_PhysicsMaterial *JPH_BodyInterface_GetMaterial(JPH_BodyInterface *bodyInterface,
                                                         const JPH_BodyID bodyId,
                                                         const JPH_SubShapeID subShapeID)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    return ToPhysicsMaterial(AsBodyInterface(bodyInterface)->GetMaterial(JPH::BodyID(bodyId), joltSubShapeID));
}

void JPH_BodyInterface_InvalidateContactCache(JPH_BodyInterface *bodyInterface, const JPH_BodyID bodyId)
{
    AsBodyInterface(bodyInterface)->InvalidateContactCache(JPH::BodyID(bodyId));
}
