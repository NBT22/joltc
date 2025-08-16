//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_BODYCREATIONSETTINGS_H
#define JOLTC_BODYCREATIONSETTINGS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Transform.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/types.h>

typedef struct JPH_BodyCreationSettings JPH_BodyCreationSettings;

JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create_GAME(const JPH_ShapeSettings *shapeSettings,
                                                                        const Transform *transform,
                                                                        JPH_MotionType motionType,
                                                                        JPH_ObjectLayer objectLayer,
                                                                        void *userData);
JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create2_GAME(const JPH_Shape *shape,
                                                                         const Transform *transform,
                                                                         JPH_MotionType motionType,
                                                                         JPH_ObjectLayer objectLayer,
                                                                         void *userData);
JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create(void);
JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create2(const JPH_ShapeSettings *shapeSettings,
                                                                    const JPH_RVec3 *position,
                                                                    const JPH_Quat *rotation,
                                                                    JPH_MotionType motionType,
                                                                    JPH_ObjectLayer objectLayer);
JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create3(const JPH_Shape *shape,
                                                                    const JPH_RVec3 *position,
                                                                    const JPH_Quat *rotation,
                                                                    JPH_MotionType motionType,
                                                                    JPH_ObjectLayer objectLayer);
JPH_CAPI void JPH_BodyCreationSettings_Destroy(JPH_BodyCreationSettings *settings);

JPH_CAPI void JPH_BodyCreationSettings_GetPosition(JPH_BodyCreationSettings *settings, JPH_RVec3 *result);
JPH_CAPI void JPH_BodyCreationSettings_SetPosition(JPH_BodyCreationSettings *settings, const JPH_RVec3 *value);

JPH_CAPI void JPH_BodyCreationSettings_GetRotation(JPH_BodyCreationSettings *settings, JPH_Quat *result);
JPH_CAPI void JPH_BodyCreationSettings_SetRotation(JPH_BodyCreationSettings *settings, const JPH_Quat *value);

JPH_CAPI void JPH_BodyCreationSettings_GetLinearVelocity(JPH_BodyCreationSettings *settings, Vector3 *velocity);
JPH_CAPI void JPH_BodyCreationSettings_SetLinearVelocity(JPH_BodyCreationSettings *settings, const Vector3 *velocity);

JPH_CAPI void JPH_BodyCreationSettings_GetAngularVelocity(JPH_BodyCreationSettings *settings, Vector3 *velocity);
JPH_CAPI void JPH_BodyCreationSettings_SetAngularVelocity(JPH_BodyCreationSettings *settings, const Vector3 *velocity);

JPH_CAPI uint64_t JPH_BodyCreationSettings_GetUserData(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetUserData(JPH_BodyCreationSettings *settings, uint64_t value);

JPH_CAPI JPH_ObjectLayer JPH_BodyCreationSettings_GetObjectLayer(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetObjectLayer(JPH_BodyCreationSettings *settings, JPH_ObjectLayer value);

JPH_CAPI JPH_MotionType JPH_BodyCreationSettings_GetMotionType(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetMotionType(JPH_BodyCreationSettings *settings, JPH_MotionType value);

JPH_CAPI JPH_AllowedDOFs JPH_BodyCreationSettings_GetAllowedDOFs(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetAllowedDOFs(JPH_BodyCreationSettings *settings, JPH_AllowedDOFs value);

JPH_CAPI bool JPH_BodyCreationSettings_GetAllowDynamicOrKinematic(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetAllowDynamicOrKinematic(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI bool JPH_BodyCreationSettings_GetIsSensor(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetIsSensor(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI bool JPH_BodyCreationSettings_GetCollideKinematicVsNonDynamic(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetCollideKinematicVsNonDynamic(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI bool JPH_BodyCreationSettings_GetUseManifoldReduction(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetUseManifoldReduction(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI bool JPH_BodyCreationSettings_GetApplyGyroscopicForce(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetApplyGyroscopicForce(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI JPH_MotionQuality JPH_BodyCreationSettings_GetMotionQuality(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetMotionQuality(JPH_BodyCreationSettings *settings, JPH_MotionQuality value);

JPH_CAPI bool JPH_BodyCreationSettings_GetEnhancedInternalEdgeRemoval(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI bool JPH_BodyCreationSettings_GetAllowSleeping(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetAllowSleeping(JPH_BodyCreationSettings *settings, bool value);

JPH_CAPI float JPH_BodyCreationSettings_GetFriction(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetFriction(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI float JPH_BodyCreationSettings_GetRestitution(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetRestitution(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI float JPH_BodyCreationSettings_GetLinearDamping(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetLinearDamping(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI float JPH_BodyCreationSettings_GetAngularDamping(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetAngularDamping(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI float JPH_BodyCreationSettings_GetMaxLinearVelocity(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetMaxLinearVelocity(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI float JPH_BodyCreationSettings_GetMaxAngularVelocity(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetMaxAngularVelocity(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI float JPH_BodyCreationSettings_GetGravityFactor(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetGravityFactor(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI uint32_t JPH_BodyCreationSettings_GetNumVelocityStepsOverride(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetNumVelocityStepsOverride(JPH_BodyCreationSettings *settings, uint32_t value);

JPH_CAPI uint32_t JPH_BodyCreationSettings_GetNumPositionStepsOverride(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetNumPositionStepsOverride(JPH_BodyCreationSettings *settings, uint32_t value);

JPH_CAPI JPH_OverrideMassProperties
JPH_BodyCreationSettings_GetOverrideMassProperties(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetOverrideMassProperties(JPH_BodyCreationSettings *settings,
                                                                 JPH_OverrideMassProperties value);

JPH_CAPI float JPH_BodyCreationSettings_GetInertiaMultiplier(const JPH_BodyCreationSettings *settings);
JPH_CAPI void JPH_BodyCreationSettings_SetInertiaMultiplier(JPH_BodyCreationSettings *settings, float value);

JPH_CAPI void JPH_BodyCreationSettings_GetMassPropertiesOverride(const JPH_BodyCreationSettings *settings,
                                                                 JPH_MassProperties *result);
JPH_CAPI void JPH_BodyCreationSettings_SetMassPropertiesOverride(JPH_BodyCreationSettings *settings,
                                                                 const JPH_MassProperties *massProperties);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BODYCREATIONSETTINGS_H
