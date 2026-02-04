//
// Created by NBT22 on 8/14/25.
//

#ifndef JOLTC_BODYINTERFACE_H
#define JOLTC_BODYINTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif

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
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <joltc/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct JPH_BodyInterface JPH_BodyInterface;

JPH_CAPI void JPH_BodyInterface_DestroyBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI JPH_BodyID JPH_BodyInterface_CreateAndAddBody(JPH_BodyInterface *bodyInterface,
                                                       const JPH_BodyCreationSettings *settings,
                                                       JPH_Activation activationMode);
JPH_CAPI JPH_Body *JPH_BodyInterface_CreateBody(JPH_BodyInterface *bodyInterface,
                                                const JPH_BodyCreationSettings *settings);
JPH_CAPI JPH_Body *JPH_BodyInterface_CreateBodyWithID(JPH_BodyInterface *bodyInterface,
                                                      JPH_BodyID bodyID,
                                                      const JPH_BodyCreationSettings *settings);
JPH_CAPI JPH_Body *JPH_BodyInterface_CreateBodyWithoutID(JPH_BodyInterface *bodyInterface,
                                                         const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyInterface_DestroyBodyWithoutID(JPH_BodyInterface *bodyInterface, JPH_Body *body);
JPH_CAPI bool JPH_BodyInterface_AssignBodyID(JPH_BodyInterface *bodyInterface, JPH_Body *body);
JPH_CAPI bool JPH_BodyInterface_AssignBodyID2(JPH_BodyInterface *bodyInterface, JPH_Body *body, JPH_BodyID bodyID);
JPH_CAPI JPH_Body *JPH_BodyInterface_UnassignBodyID(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);

JPH_CAPI JPH_Body *JPH_BodyInterface_CreateSoftBody(JPH_BodyInterface *bodyInterface,
                                                    const JPH_SoftBodyCreationSettings *settings);
JPH_CAPI JPH_Body *JPH_BodyInterface_CreateSoftBodyWithID(JPH_BodyInterface *bodyInterface,
                                                          JPH_BodyID bodyID,
                                                          const JPH_SoftBodyCreationSettings *settings);
JPH_CAPI JPH_Body *JPH_BodyInterface_CreateSoftBodyWithoutID(JPH_BodyInterface *bodyInterface,
                                                             const JPH_SoftBodyCreationSettings *settings);
JPH_CAPI JPH_BodyID JPH_BodyInterface_CreateAndAddSoftBody(JPH_BodyInterface *bodyInterface,
                                                           const JPH_SoftBodyCreationSettings *settings,
                                                           JPH_Activation activationMode);

JPH_CAPI void JPH_BodyInterface_AddBody(JPH_BodyInterface *bodyInterface,
                                        JPH_BodyID bodyID,
                                        JPH_Activation activationMode);
JPH_CAPI void JPH_BodyInterface_RemoveBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI void JPH_BodyInterface_RemoveAndDestroyBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI bool JPH_BodyInterface_IsActive(const JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI bool JPH_BodyInterface_IsAdded(const JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI JPH_BodyType JPH_BodyInterface_GetBodyType(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);

JPH_CAPI void JPH_BodyInterface_SetLinearVelocity(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyID,
                                                  const Vector3 *velocity);
JPH_CAPI void JPH_BodyInterface_GetLinearVelocity(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyID,
                                                  Vector3 *velocity);
JPH_CAPI void JPH_BodyInterface_GetCenterOfMassPosition(JPH_BodyInterface *bodyInterface,
                                                        JPH_BodyID bodyID,
                                                        JPH_RVec3 *position);

JPH_CAPI JPH_MotionType JPH_BodyInterface_GetMotionType(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI void JPH_BodyInterface_SetMotionType(JPH_BodyInterface *bodyInterface,
                                              JPH_BodyID bodyID,
                                              JPH_MotionType motionType,
                                              JPH_Activation activationMode);

JPH_CAPI float JPH_BodyInterface_GetRestitution(const JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI void JPH_BodyInterface_SetRestitution(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID, float restitution);

JPH_CAPI float JPH_BodyInterface_GetFriction(const JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);
JPH_CAPI void JPH_BodyInterface_SetFriction(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID, float friction);

JPH_CAPI void JPH_BodyInterface_SetPosition(JPH_BodyInterface *bodyInterface,
                                            JPH_BodyID bodyId,
                                            const JPH_RVec3 *position,
                                            JPH_Activation activationMode);
JPH_CAPI void JPH_BodyInterface_GetPosition(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, JPH_RVec3 *result);

JPH_CAPI void JPH_BodyInterface_SetRotation(JPH_BodyInterface *bodyInterface,
                                            JPH_BodyID bodyId,
                                            const JPH_Quat *rotation,
                                            JPH_Activation activationMode);
JPH_CAPI void JPH_BodyInterface_GetRotation(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, JPH_Quat *result);

JPH_CAPI void JPH_BodyInterface_SetPositionAndRotation(JPH_BodyInterface *bodyInterface,
                                                       JPH_BodyID bodyId,
                                                       const JPH_RVec3 *position,
                                                       const JPH_Quat *rotation,
                                                       JPH_Activation activationMode);
JPH_CAPI void JPH_BodyInterface_SetPositionAndRotationWhenChanged(JPH_BodyInterface *bodyInterface,
                                                                  JPH_BodyID bodyId,
                                                                  const JPH_RVec3 *position,
                                                                  const JPH_Quat *rotation,
                                                                  JPH_Activation activationMode);
JPH_CAPI void JPH_BodyInterface_GetPositionAndRotation(JPH_BodyInterface *bodyInterface,
                                                       JPH_BodyID bodyId,
                                                       JPH_RVec3 *position,
                                                       JPH_Quat *rotation);
JPH_CAPI void JPH_BodyInterface_SetPositionRotationAndVelocity(JPH_BodyInterface *bodyInterface,
                                                               JPH_BodyID bodyId,
                                                               const JPH_RVec3 *position,
                                                               const JPH_Quat *rotation,
                                                               const Vector3 *linearVelocity,
                                                               const Vector3 *angularVelocity);

JPH_CAPI void JPH_BodyInterface_GetCollisionGroup(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  JPH_CollisionGroup *result);
JPH_CAPI void JPH_BodyInterface_SetCollisionGroup(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  const JPH_CollisionGroup *group);

JPH_CAPI const JPH_Shape *JPH_BodyInterface_GetShape(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);
JPH_CAPI void JPH_BodyInterface_SetShape(JPH_BodyInterface *bodyInterface,
                                         JPH_BodyID bodyId,
                                         const JPH_Shape *shape,
                                         bool updateMassProperties,
                                         JPH_Activation activationMode);
JPH_CAPI void JPH_BodyInterface_NotifyShapeChanged(JPH_BodyInterface *bodyInterface,
                                                   JPH_BodyID bodyId,
                                                   const Vector3 *previousCenterOfMass,
                                                   bool updateMassProperties,
                                                   JPH_Activation activationMode);

JPH_CAPI void JPH_BodyInterface_ActivateBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);
JPH_CAPI void JPH_BodyInterface_ActivateBodies(JPH_BodyInterface *bodyInterface, const JPH_BodyID *bodyIDs, int count);
JPH_CAPI void JPH_BodyInterface_ActivateBodiesInAABox(JPH_BodyInterface *bodyInterface,
                                                      const JPH_AABox *box,
                                                      const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                                      const JPH_ObjectLayerFilter *objectLayerFilter);
JPH_CAPI void JPH_BodyInterface_DeactivateBody(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);
// TODO (merge): Implementations
JPH_CAPI void JPH_BodyInterface_DeactivateBodies(JPH_BodyInterface *bodyInterface,
                                                 const JPH_BodyID *bodyIDs,
                                                 uint32_t count);
JPH_CAPI void JPH_BodyInterface_ResetSleepTimer(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyID);

JPH_CAPI JPH_ObjectLayer JPH_BodyInterface_GetObjectLayer(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);
JPH_CAPI void JPH_BodyInterface_SetObjectLayer(JPH_BodyInterface *bodyInterface,
                                               JPH_BodyID bodyId,
                                               JPH_ObjectLayer layer);

JPH_CAPI void JPH_BodyInterface_GetWorldTransform(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  JPH_RMat44 *result);
JPH_CAPI void JPH_BodyInterface_GetCenterOfMassTransform(JPH_BodyInterface *bodyInterface,
                                                         JPH_BodyID bodyId,
                                                         JPH_RMat44 *result);

JPH_CAPI void JPH_BodyInterface_MoveKinematic(JPH_BodyInterface *bodyInterface,
                                              JPH_BodyID bodyId,
                                              const JPH_RVec3 *targetPosition,
                                              const JPH_Quat *targetRotation,
                                              float deltaTime);
JPH_CAPI bool JPH_BodyInterface_ApplyBuoyancyImpulse(JPH_BodyInterface *bodyInterface,
                                                     JPH_BodyID bodyId,
                                                     const JPH_RVec3 *surfacePosition,
                                                     const Vector3 *surfaceNormal,
                                                     float buoyancy,
                                                     float linearDrag,
                                                     float angularDrag,
                                                     const Vector3 *fluidVelocity,
                                                     const Vector3 *gravity,
                                                     float deltaTime);

JPH_CAPI void JPH_BodyInterface_SetLinearAndAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                            JPH_BodyID bodyId,
                                                            const Vector3 *linearVelocity,
                                                            const Vector3 *angularVelocity);
JPH_CAPI void JPH_BodyInterface_GetLinearAndAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                            JPH_BodyID bodyId,
                                                            Vector3 *linearVelocity,
                                                            Vector3 *angularVelocity);

JPH_CAPI void JPH_BodyInterface_AddLinearVelocity(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  const Vector3 *linearVelocity);
JPH_CAPI void JPH_BodyInterface_AddLinearAndAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                            JPH_BodyID bodyId,
                                                            const Vector3 *linearVelocity,
                                                            const Vector3 *angularVelocity);

JPH_CAPI void JPH_BodyInterface_SetAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                   JPH_BodyID bodyId,
                                                   const Vector3 *angularVelocity);
JPH_CAPI void JPH_BodyInterface_GetAngularVelocity(JPH_BodyInterface *bodyInterface,
                                                   JPH_BodyID bodyId,
                                                   Vector3 *angularVelocity);

JPH_CAPI void JPH_BodyInterface_GetPointVelocity(JPH_BodyInterface *bodyInterface,
                                                 JPH_BodyID bodyId,
                                                 const JPH_RVec3 *point,
                                                 Vector3 *velocity);

JPH_CAPI void JPH_BodyInterface_AddForce(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, const Vector3 *force);
JPH_CAPI void JPH_BodyInterface_AddForce2(JPH_BodyInterface *bodyInterface,
                                          JPH_BodyID bodyId,
                                          const Vector3 *force,
                                          const JPH_RVec3 *point);
JPH_CAPI void JPH_BodyInterface_AddTorque(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, const Vector3 *torque);
JPH_CAPI void JPH_BodyInterface_AddForceAndTorque(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  const Vector3 *force,
                                                  const Vector3 *torque);

JPH_CAPI void JPH_BodyInterface_AddImpulse(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, const Vector3 *impulse);
JPH_CAPI void JPH_BodyInterface_AddImpulse2(JPH_BodyInterface *bodyInterface,
                                            JPH_BodyID bodyId,
                                            const Vector3 *impulse,
                                            const JPH_RVec3 *point);
JPH_CAPI void JPH_BodyInterface_AddAngularImpulse(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  const Vector3 *angularImpulse);

JPH_CAPI void JPH_BodyInterface_SetMotionQuality(JPH_BodyInterface *bodyInterface,
                                                 JPH_BodyID bodyId,
                                                 JPH_MotionQuality quality);
JPH_CAPI JPH_MotionQuality JPH_BodyInterface_GetMotionQuality(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);

JPH_CAPI void JPH_BodyInterface_GetInverseInertia(JPH_BodyInterface *bodyInterface,
                                                  JPH_BodyID bodyId,
                                                  JPH_Mat44 *result);

JPH_CAPI void JPH_BodyInterface_SetGravityFactor(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, float value);
JPH_CAPI float JPH_BodyInterface_GetGravityFactor(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);

JPH_CAPI void JPH_BodyInterface_SetUseManifoldReduction(JPH_BodyInterface *bodyInterface,
                                                        JPH_BodyID bodyId,
                                                        bool value);
JPH_CAPI bool JPH_BodyInterface_GetUseManifoldReduction(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);

JPH_CAPI void JPH_BodyInterface_SetUserData(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, uint64_t userData);
JPH_CAPI uint64_t JPH_BodyInterface_GetUserData(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);

// TODO (merge): Implementations
JPH_CAPI void JPH_BodyInterface_SetIsSensor(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId, bool value);
JPH_CAPI bool JPH_BodyInterface_IsSensor(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);

JPH_CAPI const JPH_PhysicsMaterial *JPH_BodyInterface_GetMaterial(JPH_BodyInterface *bodyInterface,
                                                                  JPH_BodyID bodyId,
                                                                  JPH_SubShapeID subShapeID);

JPH_CAPI void JPH_BodyInterface_InvalidateContactCache(JPH_BodyInterface *bodyInterface, JPH_BodyID bodyId);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BODYINTERFACE_H
