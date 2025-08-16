//
// Created by NBT22 on 8/15/25.
//

#include <cstdint>
#include <joltc/enums.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Transform.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/BodyCreationSettings.h>
#include <joltc/Physics/Body/MassProperties.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/types.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Math/Quat.hpp>
#include <Math/Transform.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/BodyCreationSettings.hpp>
#include <Physics/Body/MassProperties.hpp>
#include <Physics/Collision/Shape/Shape.hpp>

JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create_GAME(const JPH_ShapeSettings *shapeSettings,
                                                                        const Transform *transform,
                                                                        JPH_MotionType motionType,
                                                                        const JPH_ObjectLayer objectLayer,
                                                                        void *userData)
{
    const JPH::ShapeSettings *joltShapeSettings = AsShapeSettings(shapeSettings);
    JPH::Vec3 position{};
    JPH::Quat rotation{};
    ToJolt(transform, position, rotation);
    const JPH::EMotionType joltMotionType = static_cast<JPH::EMotionType>(motionType);
    JPH::BodyCreationSettings *const bodyCreationSettings = new JPH::BodyCreationSettings(joltShapeSettings,
                                                                                          position,
                                                                                          rotation,
                                                                                          joltMotionType,
                                                                                          objectLayer);
    bodyCreationSettings->mUserData = reinterpret_cast<uint64_t>(userData);
    return ToBodyCreationSettings(bodyCreationSettings);
}
JPH_CAPI JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create2_GAME(const JPH_Shape *shape,
                                                                         const Transform *transform,
                                                                         JPH_MotionType motionType,
                                                                         const JPH_ObjectLayer objectLayer,
                                                                         void *userData)
{
    JPH::Vec3 position{};
    JPH::Quat rotation{};
    ToJolt(transform, position, rotation);
    const JPH::EMotionType joltMotionType = static_cast<JPH::EMotionType>(motionType);
    JPH::BodyCreationSettings *const bodyCreationSettings = new JPH::BodyCreationSettings(AsShape(shape),
                                                                                          position,
                                                                                          rotation,
                                                                                          joltMotionType,
                                                                                          objectLayer);
    bodyCreationSettings->mUserData = reinterpret_cast<uint64_t>(userData);
    return ToBodyCreationSettings(bodyCreationSettings);
}
JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create()
{
    JPH::BodyCreationSettings *const bodyCreationSettings = new JPH::BodyCreationSettings();
    return ToBodyCreationSettings(bodyCreationSettings);
}

JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create2(const JPH_ShapeSettings *shapeSettings,
                                                           const JPH_RVec3 *position,
                                                           const JPH_Quat *rotation,
                                                           JPH_MotionType motionType,
                                                           const JPH_ObjectLayer objectLayer)
{
    const JPH::ShapeSettings *joltShapeSettings = AsShapeSettings(shapeSettings);
    const JPH::Quat joltRotation = rotation != nullptr ? ToJolt(rotation) : JPH::Quat::sIdentity();
    const JPH::EMotionType joltMotionType = static_cast<JPH::EMotionType>(motionType);
    JPH::BodyCreationSettings *const bodyCreationSettings = new JPH::BodyCreationSettings(joltShapeSettings,
                                                                                          ToJolt(position),
                                                                                          joltRotation,
                                                                                          joltMotionType,
                                                                                          objectLayer);
    return ToBodyCreationSettings(bodyCreationSettings);
}

JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create3(const JPH_Shape *shape,
                                                           const JPH_RVec3 *position,
                                                           const JPH_Quat *rotation,
                                                           JPH_MotionType motionType,
                                                           const JPH_ObjectLayer objectLayer)
{
    const JPH::Quat joltRotation = rotation != nullptr ? ToJolt(rotation) : JPH::Quat::sIdentity();
    const JPH::EMotionType joltMotionType = static_cast<JPH::EMotionType>(motionType);
    JPH::BodyCreationSettings *const bodyCreationSettings = new JPH::BodyCreationSettings(AsShape(shape),
                                                                                          ToJolt(position),
                                                                                          joltRotation,
                                                                                          joltMotionType,
                                                                                          objectLayer);
    return ToBodyCreationSettings(bodyCreationSettings);
}
void JPH_BodyCreationSettings_Destroy(JPH_BodyCreationSettings *settings)
{
    if (settings != nullptr)
    {
        delete AsBodyCreationSettings(settings);
    }
}

void JPH_BodyCreationSettings_GetPosition(JPH_BodyCreationSettings *settings, JPH_RVec3 *result)
{
    FromJolt(AsBodyCreationSettings(settings)->mPosition, result);
}

void JPH_BodyCreationSettings_SetPosition(JPH_BodyCreationSettings *settings, const JPH_RVec3 *value)
{
    AsBodyCreationSettings(settings)->mPosition = ToJolt(value);
}

void JPH_BodyCreationSettings_GetRotation(JPH_BodyCreationSettings *settings, JPH_Quat *result)
{
    FromJolt(AsBodyCreationSettings(settings)->mRotation, result);
}

void JPH_BodyCreationSettings_SetRotation(JPH_BodyCreationSettings *settings, const JPH_Quat *value)
{
    AsBodyCreationSettings(settings)->mRotation = ToJolt(value);
}

void JPH_BodyCreationSettings_GetLinearVelocity(JPH_BodyCreationSettings *settings, Vector3 *velocity)
{
    FromJolt(AsBodyCreationSettings(settings)->mLinearVelocity, velocity);
}

void JPH_BodyCreationSettings_SetLinearVelocity(JPH_BodyCreationSettings *settings, const Vector3 *velocity)
{
    AsBodyCreationSettings(settings)->mLinearVelocity = ToJolt(velocity);
}

void JPH_BodyCreationSettings_GetAngularVelocity(JPH_BodyCreationSettings *settings, Vector3 *velocity)
{
    FromJolt(AsBodyCreationSettings(settings)->mAngularVelocity, velocity);
}

void JPH_BodyCreationSettings_SetAngularVelocity(JPH_BodyCreationSettings *settings, const Vector3 *velocity)
{
    JPH_ASSERT(settings);

    AsBodyCreationSettings(settings)->mAngularVelocity = ToJolt(velocity);
}

uint64_t JPH_BodyCreationSettings_GetUserData(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mUserData;
}

void JPH_BodyCreationSettings_SetUserData(JPH_BodyCreationSettings *settings, const uint64_t value)
{
    AsBodyCreationSettings(settings)->mUserData = value;
}

JPH_ObjectLayer JPH_BodyCreationSettings_GetObjectLayer(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mObjectLayer;
}

void JPH_BodyCreationSettings_SetObjectLayer(JPH_BodyCreationSettings *settings, const JPH_ObjectLayer value)
{
    AsBodyCreationSettings(settings)->mObjectLayer = value;
}

JPH_MotionType JPH_BodyCreationSettings_GetMotionType(const JPH_BodyCreationSettings *settings)
{
    return static_cast<JPH_MotionType>(AsBodyCreationSettings(settings)->mMotionType);
}

void JPH_BodyCreationSettings_SetMotionType(JPH_BodyCreationSettings *settings, JPH_MotionType value)
{
    AsBodyCreationSettings(settings)->mMotionType = static_cast<JPH::EMotionType>(value);
}

JPH_AllowedDOFs JPH_BodyCreationSettings_GetAllowedDOFs(const JPH_BodyCreationSettings *settings)
{
    return static_cast<JPH_AllowedDOFs>(AsBodyCreationSettings(settings)->mAllowedDOFs);
}

void JPH_BodyCreationSettings_SetAllowedDOFs(JPH_BodyCreationSettings *settings, JPH_AllowedDOFs value)
{
    AsBodyCreationSettings(settings)->mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(value);
}

bool JPH_BodyCreationSettings_GetAllowDynamicOrKinematic(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mAllowDynamicOrKinematic;
}

void JPH_BodyCreationSettings_SetAllowDynamicOrKinematic(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mAllowDynamicOrKinematic = value;
}

bool JPH_BodyCreationSettings_GetIsSensor(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mIsSensor;
}

void JPH_BodyCreationSettings_SetIsSensor(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mIsSensor = value;
}

bool JPH_BodyCreationSettings_GetCollideKinematicVsNonDynamic(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mCollideKinematicVsNonDynamic;
}

void JPH_BodyCreationSettings_SetCollideKinematicVsNonDynamic(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mCollideKinematicVsNonDynamic = value;
}

bool JPH_BodyCreationSettings_GetUseManifoldReduction(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mUseManifoldReduction;
}

void JPH_BodyCreationSettings_SetUseManifoldReduction(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mUseManifoldReduction = value;
}

bool JPH_BodyCreationSettings_GetApplyGyroscopicForce(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mApplyGyroscopicForce;
}

void JPH_BodyCreationSettings_SetApplyGyroscopicForce(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mApplyGyroscopicForce = value;
}

JPH_MotionQuality JPH_BodyCreationSettings_GetMotionQuality(const JPH_BodyCreationSettings *settings)
{
    return static_cast<JPH_MotionQuality>(AsBodyCreationSettings(settings)->mMotionQuality);
}

void JPH_BodyCreationSettings_SetMotionQuality(JPH_BodyCreationSettings *settings, JPH_MotionQuality value)
{
    AsBodyCreationSettings(settings)->mMotionQuality = static_cast<JPH::EMotionQuality>(value);
}

bool JPH_BodyCreationSettings_GetEnhancedInternalEdgeRemoval(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mEnhancedInternalEdgeRemoval;
}

void JPH_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mEnhancedInternalEdgeRemoval = value;
}

bool JPH_BodyCreationSettings_GetAllowSleeping(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mAllowSleeping;
}

void JPH_BodyCreationSettings_SetAllowSleeping(JPH_BodyCreationSettings *settings, const bool value)
{
    AsBodyCreationSettings(settings)->mAllowSleeping = value;
}

float JPH_BodyCreationSettings_GetFriction(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mFriction;
}

void JPH_BodyCreationSettings_SetFriction(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mFriction = value;
}

float JPH_BodyCreationSettings_GetRestitution(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mRestitution;
}

void JPH_BodyCreationSettings_SetRestitution(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mRestitution = value;
}

float JPH_BodyCreationSettings_GetLinearDamping(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mLinearDamping;
}

void JPH_BodyCreationSettings_SetLinearDamping(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mLinearDamping = value;
}

float JPH_BodyCreationSettings_GetAngularDamping(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mAngularDamping;
}

void JPH_BodyCreationSettings_SetAngularDamping(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mAngularDamping = value;
}

float JPH_BodyCreationSettings_GetMaxLinearVelocity(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mMaxLinearVelocity;
}

void JPH_BodyCreationSettings_SetMaxLinearVelocity(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mMaxLinearVelocity = value;
}

float JPH_BodyCreationSettings_GetMaxAngularVelocity(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mMaxAngularVelocity;
}

void JPH_BodyCreationSettings_SetMaxAngularVelocity(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mMaxAngularVelocity = value;
}

float JPH_BodyCreationSettings_GetGravityFactor(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mGravityFactor;
}

void JPH_BodyCreationSettings_SetGravityFactor(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mGravityFactor = value;
}

uint32_t JPH_BodyCreationSettings_GetNumVelocityStepsOverride(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mNumVelocityStepsOverride;
}

void JPH_BodyCreationSettings_SetNumVelocityStepsOverride(JPH_BodyCreationSettings *settings, const uint32_t value)
{
    AsBodyCreationSettings(settings)->mNumVelocityStepsOverride = value;
}

uint32_t JPH_BodyCreationSettings_GetNumPositionStepsOverride(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mNumPositionStepsOverride;
}

void JPH_BodyCreationSettings_SetNumPositionStepsOverride(JPH_BodyCreationSettings *settings, const uint32_t value)
{
    AsBodyCreationSettings(settings)->mNumPositionStepsOverride = value;
}

JPH_OverrideMassProperties JPH_BodyCreationSettings_GetOverrideMassProperties(const JPH_BodyCreationSettings *settings)
{
    return static_cast<JPH_OverrideMassProperties>(AsBodyCreationSettings(settings)->mOverrideMassProperties);
}

void JPH_BodyCreationSettings_SetOverrideMassProperties(JPH_BodyCreationSettings *settings,
                                                        JPH_OverrideMassProperties value)
{
    AsBodyCreationSettings(settings)->mOverrideMassProperties = static_cast<JPH::EOverrideMassProperties>(value);
}

float JPH_BodyCreationSettings_GetInertiaMultiplier(const JPH_BodyCreationSettings *settings)
{
    return AsBodyCreationSettings(settings)->mInertiaMultiplier;
}

void JPH_BodyCreationSettings_SetInertiaMultiplier(JPH_BodyCreationSettings *settings, const float value)
{
    AsBodyCreationSettings(settings)->mInertiaMultiplier = value;
}

void JPH_BodyCreationSettings_GetMassPropertiesOverride(const JPH_BodyCreationSettings *settings,
                                                        JPH_MassProperties *result)
{
    FromJolt(AsBodyCreationSettings(settings)->mMassPropertiesOverride, result);
}

void JPH_BodyCreationSettings_SetMassPropertiesOverride(JPH_BodyCreationSettings *settings,
                                                        const JPH_MassProperties *massProperties)
{
    AsBodyCreationSettings(settings)->mMassPropertiesOverride = ToJolt(massProperties);
}
