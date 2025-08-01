// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

// ReSharper disable CppDFATimeOver

#include "joltc.h"
#include <cstdarg>
#include <iostream>
#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/RegisterTypes.h>

// NOLINTBEGIN(*-macro-parentheses)
#define DEF_MAP_DECL(JoltType, c_type) \
    static inline const JPH::JoltType &As##JoltType(const c_type &t) \
    { \
        return reinterpret_cast<const JPH::JoltType &>(t); \
    } \
    static inline const JPH::JoltType *As##JoltType(const c_type *t) \
    { \
        return reinterpret_cast<const JPH::JoltType *>(t); \
    } \
    static inline JPH::JoltType &As##JoltType(c_type &t) \
    { \
        return reinterpret_cast<JPH::JoltType &>(t); \
    } \
    static inline JPH::JoltType *As##JoltType(c_type *t) \
    { \
        return reinterpret_cast<JPH::JoltType *>(t); \
    } \
    static inline const c_type &To##JoltType(const JPH::JoltType &t) \
    { \
        return reinterpret_cast<const c_type &>(t); \
    } \
    static inline const c_type *To##JoltType(const JPH::JoltType *t) \
    { \
        return reinterpret_cast<const c_type *>(t); \
    } \
    static inline c_type &To##JoltType(JPH::JoltType &t) \
    { \
        return reinterpret_cast<c_type &>(t); \
    } \
    static inline c_type *To##JoltType(JPH::JoltType *t) \
    { \
        return reinterpret_cast<c_type *>(t); \
    }
// NOLINTEND(*-macro-parentheses)

//DEF_MAP_DECL(Quat, JPH_Quat)
DEF_MAP_DECL(ContactManifold, JPH_ContactManifold)
DEF_MAP_DECL(BodyCreationSettings, JPH_BodyCreationSettings)
DEF_MAP_DECL(SoftBodyCreationSettings, JPH_SoftBodyCreationSettings)
DEF_MAP_DECL(Body, JPH_Body)
DEF_MAP_DECL(BodyInterface, JPH_BodyInterface)
DEF_MAP_DECL(BodyLockInterface, JPH_BodyLockInterface)
DEF_MAP_DECL(MotionProperties, JPH_MotionProperties)
DEF_MAP_DECL(BroadPhaseQuery, JPH_BroadPhaseQuery)
DEF_MAP_DECL(NarrowPhaseQuery, JPH_NarrowPhaseQuery)
DEF_MAP_DECL(PhysicsMaterial, JPH_PhysicsMaterial)
DEF_MAP_DECL(Shape, JPH_Shape)
DEF_MAP_DECL(ShapeSettings, JPH_ShapeSettings)
DEF_MAP_DECL(EmptyShape, JPH_EmptyShape)
DEF_MAP_DECL(EmptyShapeSettings, JPH_EmptyShapeSettings)
DEF_MAP_DECL(CompoundShape, JPH_CompoundShape)
DEF_MAP_DECL(CompoundShapeSettings, JPH_CompoundShapeSettings)
DEF_MAP_DECL(MutableCompoundShape, JPH_MutableCompoundShape)
DEF_MAP_DECL(MutableCompoundShapeSettings, JPH_MutableCompoundShapeSettings)
DEF_MAP_DECL(MeshShape, JPH_MeshShape)
DEF_MAP_DECL(MeshShapeSettings, JPH_MeshShapeSettings)
DEF_MAP_DECL(HeightFieldShape, JPH_HeightFieldShape)
DEF_MAP_DECL(HeightFieldShapeSettings, JPH_HeightFieldShapeSettings)
DEF_MAP_DECL(Constraint, JPH_Constraint)
DEF_MAP_DECL(TwoBodyConstraint, JPH_TwoBodyConstraint)
DEF_MAP_DECL(FixedConstraint, JPH_FixedConstraint)
DEF_MAP_DECL(DistanceConstraint, JPH_DistanceConstraint)
DEF_MAP_DECL(PointConstraint, JPH_PointConstraint)
DEF_MAP_DECL(HingeConstraint, JPH_HingeConstraint)
DEF_MAP_DECL(SliderConstraint, JPH_SliderConstraint)
DEF_MAP_DECL(ConeConstraint, JPH_ConeConstraint)
DEF_MAP_DECL(SwingTwistConstraint, JPH_SwingTwistConstraint)
DEF_MAP_DECL(SixDOFConstraint, JPH_SixDOFConstraint)
DEF_MAP_DECL(GearConstraint, JPH_GearConstraint)
DEF_MAP_DECL(Character, JPH_Character)
DEF_MAP_DECL(CharacterVirtual, JPH_CharacterVirtual)
DEF_MAP_DECL(Skeleton, JPH_Skeleton)
DEF_MAP_DECL(RagdollSettings, JPH_RagdollSettings)
DEF_MAP_DECL(Ragdoll, JPH_Ragdoll)
DEF_MAP_DECL(GroupFilter, JPH_GroupFilter)
DEF_MAP_DECL(GroupFilterTable, JPH_GroupFilterTable)

DEF_MAP_DECL(PhysicsStepListener, JPH_PhysicsStepListener)

// Vehicle
DEF_MAP_DECL(WheelSettings, JPH_WheelSettings)
DEF_MAP_DECL(Wheel, JPH_Wheel)

DEF_MAP_DECL(VehicleControllerSettings, JPH_VehicleControllerSettings)
DEF_MAP_DECL(VehicleController, JPH_VehicleController)

DEF_MAP_DECL(WheelSettingsWV, JPH_WheelSettingsWV)
DEF_MAP_DECL(WheelWV, JPH_WheelWV)
DEF_MAP_DECL(WheeledVehicleControllerSettings, JPH_WheeledVehicleControllerSettings)
DEF_MAP_DECL(WheeledVehicleController, JPH_WheeledVehicleController)

DEF_MAP_DECL(MotorcycleControllerSettings, JPH_MotorcycleControllerSettings)
DEF_MAP_DECL(MotorcycleController, JPH_MotorcycleController)

DEF_MAP_DECL(WheelSettingsTV, JPH_WheelSettingsTV)
DEF_MAP_DECL(WheelTV, JPH_WheelTV)
DEF_MAP_DECL(TrackedVehicleControllerSettings, JPH_TrackedVehicleControllerSettings)
DEF_MAP_DECL(TrackedVehicleController, JPH_TrackedVehicleController)

DEF_MAP_DECL(VehicleTransmissionSettings, JPH_VehicleTransmissionSettings)
DEF_MAP_DECL(VehicleCollisionTester, JPH_VehicleCollisionTester)
DEF_MAP_DECL(VehicleCollisionTesterRay, JPH_VehicleCollisionTesterRay)
DEF_MAP_DECL(VehicleCollisionTesterCastSphere, JPH_VehicleCollisionTesterCastSphere)
DEF_MAP_DECL(VehicleCollisionTesterCastCylinder, JPH_VehicleCollisionTesterCastCylinder)
DEF_MAP_DECL(VehicleConstraint, JPH_VehicleConstraint)

// Callback for traces, connect this to your own trace function if you have one
static JPH_TraceFunc s_TraceFunc = nullptr;

static void TraceImpl(const char *fmt, ...)
{
    // Format the message
    va_list list;
    va_start(list, fmt);
    char buffer[1024]; // NOLINT(*-avoid-c-arrays)
    vsnprintf(buffer, sizeof(buffer), fmt, list);
    va_end(list);

    // Print to the TTY
    if (s_TraceFunc != nullptr)
    {
        s_TraceFunc(buffer);
    } else
    {
        std::cout << buffer << std::endl;
    }
}

#ifdef JPH_ENABLE_ASSERTS
static JPH_AssertFailureFunc s_AssertFailureFunc = nullptr;

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, const uint32_t inLine)
{
    if (s_AssertFailureFunc != nullptr)
    {
        return s_AssertFailureFunc(inExpression, inMessage, inFile, inLine);
    }

    // Print to the TTY
    std::cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr ? inMessage : "");
    std::cout << std::endl;

    // Breakpoint
    return true;
}

#endif // JPH_ENABLE_ASSERTS

// TODO: The functions in this region could use with optimization from std::copy_n
#pragma region FromJoltFunctions

static inline void FromJolt(const JPH::Vec3 &vec, JPH_Vec3 *result)
{
    result->x = vec.GetX();
    result->y = vec.GetY();
    result->z = vec.GetZ();
}

static inline JPH_Vec3 FromJolt(const JPH::Vec3 &vec)
{
    return {vec.GetX(), vec.GetY(), vec.GetZ()};
}

static inline void FromJolt(const JPH::Quat &quat, JPH_Quat *result)
{
    result->x = quat.GetX();
    result->y = quat.GetY();
    result->z = quat.GetZ();
    result->w = quat.GetW();
}

static inline void FromJolt(const JPH::Plane &value, JPH_Plane *result)
{
    FromJolt(value.GetNormal(), &result->normal);
    result->distance = value.GetConstant();
}

static inline void FromJolt(const JPH::AABox &value, JPH_AABox *result)
{
    FromJolt(value.mMin, &result->min);
    FromJolt(value.mMax, &result->max);
}

static inline void FromJolt(const JPH::Mat44 &matrix, JPH_Matrix4x4 *result)
{
    const JPH::Vec4 column0 = matrix.GetColumn4(0);
    const JPH::Vec4 column1 = matrix.GetColumn4(1);
    const JPH::Vec4 column2 = matrix.GetColumn4(2);
    const JPH::Vec3 translation = matrix.GetTranslation();

    result->m11 = column0.GetX();
    result->m12 = column0.GetY();
    result->m13 = column0.GetZ();
    result->m14 = column0.GetW();

    result->m21 = column1.GetX();
    result->m22 = column1.GetY();
    result->m23 = column1.GetZ();
    result->m24 = column1.GetW();

    result->m31 = column2.GetX();
    result->m32 = column2.GetY();
    result->m33 = column2.GetZ();
    result->m34 = column2.GetW();

    result->m41 = translation.GetX();
    result->m42 = translation.GetY();
    result->m43 = translation.GetZ();
    result->m44 = 1.0;
}

#ifdef JPH_DOUBLE_PRECISION
static inline void FromJolt(const RVec3 &vec, JPH_RVec3 *result)
{
    result->x = vec.GetX();
    result->y = vec.GetY();
    result->z = vec.GetZ();
}

static inline JPH_RVec3 FromJolt(const RVec3 &vec)
{
    return {vec.GetX(), vec.GetY(), vec.GetZ()};
}

static inline void FromJolt(const DMat44 &matrix, JPH_RMatrix4x4 *result)
{
    Vec4 column0 = matrix.GetColumn4(0);
    Vec4 column1 = matrix.GetColumn4(1);
    Vec4 column2 = matrix.GetColumn4(2);
    DVec3 translation = matrix.GetTranslation();

    result->m11 = column0.GetX();
    result->m12 = column0.GetY();
    result->m13 = column0.GetZ();
    result->m14 = column0.GetW();

    result->m21 = column1.GetX();
    result->m22 = column1.GetY();
    result->m23 = column1.GetZ();
    result->m24 = column1.GetW();

    result->m31 = column2.GetX();
    result->m32 = column2.GetY();
    result->m33 = column2.GetZ();
    result->m34 = column2.GetW();

    result->m41 = translation.GetX();
    result->m42 = translation.GetY();
    result->m43 = translation.GetZ();
    result->m44 = 1.0f;
}
#endif // JPH_DOUBLE_PRECISION

static inline void FromJolt(const JPH::MassProperties &jolt, JPH_MassProperties *result)
{
    result->mass = jolt.mMass;
    FromJolt(jolt.mInertia, &result->inertia);
}

static inline void FromJolt(const JPH::SpringSettings &jolt, JPH_SpringSettings *result)
{
    result->mode = static_cast<JPH_SpringMode>(jolt.mMode);
    result->frequencyOrStiffness = jolt.mFrequency;
    result->damping = jolt.mDamping;
}

static inline void FromJolt(const JPH::MotorSettings &jolt, JPH_MotorSettings *result)
{
    FromJolt(jolt.mSpringSettings, &result->springSettings);
    result->minForceLimit = jolt.mMinForceLimit;
    result->maxForceLimit = jolt.mMaxForceLimit;
    result->minTorqueLimit = jolt.mMinTorqueLimit;
    result->maxTorqueLimit = jolt.mMaxTorqueLimit;
}

static inline const JPH_PhysicsMaterial *FromJolt(const JPH::PhysicsMaterial *joltMaterial)
{
    return joltMaterial != nullptr ? ToPhysicsMaterial(joltMaterial) : nullptr;
}

static inline void FromJolt(const JPH::CharacterVirtual::Contact &jolt, JPH_CharacterVirtualContact *result)
{
    result->hash = jolt.GetHash();
    result->bodyB = jolt.mBodyB.GetIndexAndSequenceNumber();
    result->characterIDB = jolt.mCharacterIDB.GetValue();
    result->subShapeIDB = jolt.mSubShapeIDB.GetValue();
    FromJolt(jolt.mPosition, &result->position);
    FromJolt(jolt.mLinearVelocity, &result->linearVelocity);
    FromJolt(jolt.mContactNormal, &result->contactNormal);
    FromJolt(jolt.mSurfaceNormal, &result->surfaceNormal);
    result->distance = jolt.mDistance;
    result->fraction = jolt.mFraction;
    result->motionTypeB = static_cast<JPH_MotionType>(jolt.mMotionTypeB);
    result->isSensorB = jolt.mIsSensorB;
    result->characterB = ToCharacterVirtual(jolt.mCharacterB);
    result->userData = jolt.mUserData;
    result->material = ToPhysicsMaterial(jolt.mMaterial);
    result->hadCollision = jolt.mHadCollision;
    result->wasDiscarded = jolt.mWasDiscarded;
    result->canPushCharacter = jolt.mCanPushCharacter;
}

static inline void FromJolt(const JPH::Skeleton::Joint &jolt, JPH_SkeletonJoint *result)
{
    result->name = jolt.mName.c_str();
    result->parentName = jolt.mParentName.c_str();
    result->parentJointIndex = jolt.mParentJointIndex;
}

static inline JPH_CollideShapeResult FromJolt(const JPH::CollideShapeResult &jolt)
{
    JPH_CollideShapeResult result{};
    FromJolt(jolt.mContactPointOn1, &result.contactPointOn1);
    FromJolt(jolt.mContactPointOn2, &result.contactPointOn2);
    FromJolt(jolt.mPenetrationAxis, &result.penetrationAxis);
    result.penetrationDepth = jolt.mPenetrationDepth;
    result.subShapeID1 = jolt.mSubShapeID1.GetValue();
    result.subShapeID2 = jolt.mSubShapeID2.GetValue();
    result.bodyID2 = jolt.mBodyID2.GetIndexAndSequenceNumber();

    if (!jolt.mShape1Face.empty())
    {
        result.shape1FaceCount = jolt.mShape1Face.size();
        result.shape1Faces = static_cast<JPH_Vec3 *>(malloc(sizeof(JPH_Vec3) * result.shape1FaceCount));
        for (uint32_t i = 0; i < result.shape1FaceCount; i++)
        {
            FromJolt(jolt.mShape1Face[i], &result.shape1Faces[i]);
        }
    }

    if (!jolt.mShape2Face.empty())
    {
        result.shape2FaceCount = jolt.mShape2Face.size();
        result.shape2Faces = static_cast<JPH_Vec3 *>(malloc(sizeof(JPH_Vec3) * result.shape2FaceCount));
        for (uint32_t i = 0; i < result.shape2FaceCount; i++)
        {
            FromJolt(jolt.mShape2Face[i], &result.shape2Faces[i]);
        }
    }

    return result;
}

static inline void FromJolt(const JPH::CollisionGroup &jolt, JPH_CollisionGroup *result)
{
    result->groupFilter = ToGroupFilter(jolt.GetGroupFilter());
    result->groupID = jolt.GetGroupID();
    result->subGroupID = jolt.GetSubGroupID();
}

#pragma endregion FromJoltFunctions

// TODO: The functions in this region could use with optimization from std::copy_n
#pragma region ToJoltFunctions

static inline JPH::Vec3 ToJolt(const JPH_Vec3 &vec)
{
    return {vec.x, vec.y, vec.z};
}

static inline JPH::Vec3 ToJolt(const JPH_Vec3 *vec)
{
    return {vec->x, vec->y, vec->z};
}

// static inline JPH::Vec3 ToJolt(const float value[3])
// {
//     return {value[0], value[1], value[2]};
// }

static inline JPH::Quat ToJolt(const JPH_Quat *quat)
{
    return {quat->x, quat->y, quat->z, quat->w};
}

static inline JPH::Plane ToJolt(const JPH_Plane *value)
{
    return {ToJolt(value->normal), value->distance};
}

static inline JPH::Mat44 ToJolt(const JPH_Matrix4x4 *matrix)
{
    JPH::Mat44 result{};
    result.SetColumn4(0, JPH::Vec4(matrix->m11, matrix->m12, matrix->m13, matrix->m14));
    result.SetColumn4(1, JPH::Vec4(matrix->m21, matrix->m22, matrix->m23, matrix->m24));
    result.SetColumn4(2, JPH::Vec4(matrix->m31, matrix->m32, matrix->m33, matrix->m34));
    result.SetColumn4(3, JPH::Vec4(matrix->m41, matrix->m42, matrix->m43, matrix->m44));
    return result;
}

static inline JPH::Float3 ToJoltFloat3(const JPH_Vec3 &vec)
{
    return {vec.x, vec.y, vec.z};
}

static inline JPH::AABox ToJolt(const JPH_AABox *value)
{
    return {ToJolt(value->min), ToJolt(value->max)};
}

#ifdef JPH_DOUBLE_PRECISION
static inline JPH::RVec3 ToJolt(const JPH_RVec3 &vec)
{
    return JPH::RVec3(vec.x, vec.y, vec.z);
}

static inline JPH::RVec3 ToJolt(const JPH_RVec3 *vec)
{
    return JPH::RVec3(vec->x, vec->y, vec->z);
}

static inline JPH::RMat44 ToJolt(const JPH_RMatrix4x4 *matrix)
{
    JPH::RMat44 result{};
    result.SetColumn4(0, JPH::Vec4(matrix->m11, matrix->m12, matrix->m13, matrix->m14));
    result.SetColumn4(1, JPH::Vec4(matrix->m21, matrix->m22, matrix->m23, matrix->m24));
    result.SetColumn4(2, JPH::Vec4(matrix->m31, matrix->m32, matrix->m33, matrix->m34));
    result.SetTranslation(JPH::RVec3(matrix->m41, matrix->m42, matrix->m43));
    return result;
}
#endif // JPH_DOUBLE_PRECISION

static inline JPH::MassProperties ToJolt(const JPH_MassProperties *properties)
{
    JPH::MassProperties result{};
    if (properties == nullptr)
    {
        return result;
    }
    result.mMass = properties->mass;
    result.mInertia = ToJolt(&properties->inertia);
    return result;
}

static inline JPH::RayCastSettings ToJolt(const JPH_RayCastSettings *settings)
{
    JPH::RayCastSettings result{};
    if (settings == nullptr)
    {
        return result;
    }
    result.mBackFaceModeTriangles = static_cast<JPH::EBackFaceMode>(settings->backFaceModeTriangles);
    result.mBackFaceModeConvex = static_cast<JPH::EBackFaceMode>(settings->backFaceModeConvex);
    result.mTreatConvexAsSolid = settings->treatConvexAsSolid;
    return result;
}

static inline JPH::SpringSettings ToJolt(const JPH_SpringSettings *settings)
{
    JPH::SpringSettings result{};
    if (settings == nullptr)
    {
        return result;
    }
    result.mMode = static_cast<JPH::ESpringMode>(settings->mode);
    result.mFrequency = settings->frequencyOrStiffness;
    result.mDamping = settings->damping;
    return result;
}

static inline JPH::MotorSettings ToJolt(const JPH_MotorSettings *settings)
{
    JPH::MotorSettings result{};
    if (settings == nullptr)
    {
        return result;
    }
    result.mSpringSettings = ToJolt(&settings->springSettings);
    result.mMinForceLimit = settings->minForceLimit;
    result.mMaxForceLimit = settings->maxForceLimit;
    result.mMinTorqueLimit = settings->minTorqueLimit;
    result.mMaxTorqueLimit = settings->maxTorqueLimit;
    return result;
}

static inline JPH::CollisionGroup ToJolt(const JPH_CollisionGroup *group)
{
    JPH::CollisionGroup result(AsGroupFilter(group->groupFilter), group->groupID, group->subGroupID);
    return result;
}

#pragma endregion ToJoltFunctions

void JPH_MassProperties_DecomposePrincipalMomentsOfInertia(const JPH_MassProperties *properties,
                                                           JPH_Matrix4x4 *rotation,
                                                           JPH_Vec3 *diagonal)
{
    JPH::Mat44 joltRotation;
    JPH::Vec3 joltDiagonal;
    const JPH::MassProperties joltProperties = ToJolt(properties);
    joltProperties.DecomposePrincipalMomentsOfInertia(joltRotation, joltDiagonal);
    FromJolt(joltRotation, rotation);
    FromJolt(joltDiagonal, diagonal);
}

void JPH_MassProperties_ScaleToMass(JPH_MassProperties *properties, const float mass)
{
    JPH::MassProperties joltProperties = ToJolt(properties);
    joltProperties.ScaleToMass(mass);
    properties->mass = joltProperties.mMass;
    FromJolt(joltProperties.mInertia, &properties->inertia);
}

void JPH_MassProperties_GetEquivalentSolidBoxSize(const float mass, const JPH_Vec3 *inertiaDiagonal, JPH_Vec3 *result)
{
    FromJolt(JPH::MassProperties::sGetEquivalentSolidBoxSize(mass, ToJolt(inertiaDiagonal)), result);
}

void JPH_RayCast_GetPointOnRay(const JPH_Vec3 *origin, const JPH_Vec3 *direction, float fraction, JPH_Vec3 *result)
{
    JPH_ASSERT(origin);
    JPH_ASSERT(direction);
    JPH_ASSERT(result);

    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::Vec3 point = ray.GetPointOnRay(fraction);
    FromJolt(point, result);
}

void JPH_RRayCast_GetPointOnRay(const JPH_RVec3 *origin, const JPH_Vec3 *direction, float fraction, JPH_RVec3 *result)
{
    JPH_ASSERT(origin);
    JPH_ASSERT(direction);
    JPH_ASSERT(result);

    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RVec3 point = ray.GetPointOnRay(fraction);
    FromJolt(point, result);
}

static JPH::Triangle ToTriangle(const JPH_Triangle &triangle)
{
    return {ToJoltFloat3(triangle.v1), ToJoltFloat3(triangle.v2), ToJoltFloat3(triangle.v3), triangle.materialIndex};
}

static JPH::IndexedTriangle ToIndexedTriangle(const JPH_IndexedTriangle &triangle)
{
    return {triangle.i1, triangle.i2, triangle.i3, triangle.materialIndex, triangle.userData};
}

// 10 MB was not enough for large simulation, let's use TempAllocatorMalloc
static JPH::TempAllocator *s_TempAllocator = nullptr;

class JobSystemCallback final: public JPH::JobSystemWithBarrier
{
    public:
        explicit JobSystemCallback(const JPH_JobSystemConfig *config)
        {
            Init(config->maxBarriers > 0 ? config->maxBarriers : JPH::cMaxPhysicsBarriers);
            mJobs.Init(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsJobs);
            mConfig = *config;
        }

        JobHandle CreateJob(const char *name,
                            JPH::ColorArg color,
                            const JobFunction &callback,
                            uint32_t dependencies = 0) override
        {
            uint32_t index;

            while (true)
            {
                index = mJobs.ConstructObject(name, color, this, callback, dependencies);
                if (index != JPH::FixedSizeFreeList<Job>::cInvalidObjectIndex)
                {
                    break;
                }
                JPH_ASSERT(false, "No jobs available!");
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }

            Job *job = &mJobs.Get(index);
            JobHandle handle(job);

            if (dependencies == 0)
            {
                QueueJob(job);
            }

            return handle;
        }

        void FreeJob(Job *job) override
        {
            mJobs.DestructObject(job);
        }

        [[nodiscard]] int GetMaxConcurrency() const override
        {
            return mConfig.maxConcurrency;
        }

    protected:
        static void RunJob(void *arg)
        {
            Job *job = static_cast<Job *>(arg);
            job->Execute();
            job->Release();
        }

        void QueueJob(Job *job) override
        {
            job->AddRef();
            mConfig.queueJob(mConfig.context, RunJob, job);
        }

        void QueueJobs(Job **jobs, const uint32_t count) override
        {
            for (uint32_t i = 0; i < count; i++)
            {
                jobs[i]->AddRef();
            }

            mConfig.queueJobs(mConfig.context, RunJob, reinterpret_cast<void **>(jobs), count);
        }

    private:
        JPH::FixedSizeFreeList<Job> mJobs;
        JPH_JobSystemConfig mConfig;
};

JPH_JobSystem *JPH_JobSystemThreadPool_Create(const JobSystemThreadPoolConfig *config)
{
    JobSystemThreadPoolConfig createConfig{};
    if (config != nullptr)
    {
        createConfig = *config;
    }

    const uint32_t maxJobs = createConfig.maxJobs > 0 ? createConfig.maxJobs : JPH::cMaxPhysicsJobs;
    const uint32_t maxBarriers = createConfig.maxBarriers > 0 ? createConfig.maxBarriers : JPH::cMaxPhysicsBarriers;
    const int numThreads = createConfig.numThreads > 0 ? createConfig.numThreads : -1;
    JPH::JobSystem *jobSystem = new JPH::JobSystemThreadPool(maxJobs, maxBarriers, numThreads);
    return reinterpret_cast<JPH_JobSystem *>(jobSystem);
}

JPH_JobSystem *JPH_JobSystemCallback_Create(const JPH_JobSystemConfig *config)
{
    JPH::JobSystem *jobSystem = new JobSystemCallback(config);
    return reinterpret_cast<JPH_JobSystem *>(jobSystem);
}

void JPH_JobSystem_Destroy(JPH_JobSystem *jobSystem)
{
    delete reinterpret_cast<JPH::JobSystem *>(jobSystem);
}

bool JPH_Init()
{
    JPH::RegisterDefaultAllocator();

    // TODO
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    // Create a factory
    JPH::Factory::sInstance = new JPH::Factory();

    // Register all Jolt physics types
    JPH::RegisterTypes();

    // Init temp allocator
    s_TempAllocator = new JPH::TempAllocatorImplWithMallocFallback(8 * 1024 * 1024);

    return true;
}

void JPH_Shutdown()
{
    delete s_TempAllocator;
    s_TempAllocator = nullptr;

    // Unregisters all types with the factory and cleans up the default material
    JPH::UnregisterTypes();

    // Destroy the factory
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

void JPH_SetTraceHandler(const JPH_TraceFunc handler)
{
    s_TraceFunc = handler;
}

void JPH_SetAssertFailureHandler(JPH_AssertFailureFunc handler)
{
#ifdef JPH_ENABLE_ASSERTS
    s_AssertFailureFunc = handler;
#else
    JPH_UNUSED(handler);
#endif
}

/* JPH_CollideShapeResult */
void JPH_CollideShapeResult_FreeMembers(const JPH_CollideShapeResult *result)
{
    if (result->shape1FaceCount != 0)
    {
        free(result->shape1Faces);
    }

    if (result->shape2FaceCount != 0)
    {
        free(result->shape2Faces);
    }
}

void JPH_CollisionEstimationResult_FreeMembers(const JPH_CollisionEstimationResult *result)
{
    if (result->impulseCount != 0)
    {
        free(result->impulses);
    }
}

/* JPH_BroadPhaseLayerInterface */
JPH_BroadPhaseLayerInterface *JPH_BroadPhaseLayerInterfaceMask_Create(const uint32_t numBroadPhaseLayers)
{
    JPH::BroadPhaseLayerInterfaceMask *const system = new JPH::BroadPhaseLayerInterfaceMask(numBroadPhaseLayers);
    return reinterpret_cast<JPH_BroadPhaseLayerInterface *>(system);
}

void JPH_BroadPhaseLayerInterfaceMask_ConfigureLayer(JPH_BroadPhaseLayerInterface *bpInterface,
                                                     const JPH_BroadPhaseLayer broadPhaseLayer,
                                                     const uint32_t groupsToInclude,
                                                     const uint32_t groupsToExclude)
{
    reinterpret_cast<JPH::BroadPhaseLayerInterfaceMask *>(bpInterface)
            ->ConfigureLayer(static_cast<JPH::BroadPhaseLayer>(broadPhaseLayer), groupsToInclude, groupsToExclude);
}

JPH_BroadPhaseLayerInterface *JPH_BroadPhaseLayerInterfaceTable_Create(const uint32_t numObjectLayers,
                                                                       const uint32_t numBroadPhaseLayers)
{
    JPH::BroadPhaseLayerInterfaceTable *const system = new JPH::BroadPhaseLayerInterfaceTable(numObjectLayers,
                                                                                              numBroadPhaseLayers);
    return reinterpret_cast<JPH_BroadPhaseLayerInterface *>(system);
}

void JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(JPH_BroadPhaseLayerInterface *bpInterface,
                                                                  const JPH_ObjectLayer objectLayer,
                                                                  const JPH_BroadPhaseLayer broadPhaseLayer)
{
    reinterpret_cast<JPH::BroadPhaseLayerInterfaceTable *>(bpInterface)
            ->MapObjectToBroadPhaseLayer(objectLayer, static_cast<JPH::BroadPhaseLayer>(broadPhaseLayer));
}

/* JPH_ObjectLayerPairFilter */
JPH_ObjectLayerPairFilter *JPH_ObjectLayerPairFilterMask_Create()
{
    JPH::ObjectLayerPairFilterMask *const filter = new JPH::ObjectLayerPairFilterMask();
    return reinterpret_cast<JPH_ObjectLayerPairFilter *>(filter);
}

JPH_ObjectLayer JPH_ObjectLayerPairFilterMask_GetObjectLayer(const uint32_t group, const uint32_t mask)
{
    return JPH::ObjectLayerPairFilterMask::sGetObjectLayer(group, mask);
}

uint32_t JPH_ObjectLayerPairFilterMask_GetGroup(const JPH_ObjectLayer layer)
{
    return JPH::ObjectLayerPairFilterMask::sGetGroup(layer);
}

uint32_t JPH_ObjectLayerPairFilterMask_GetMask(const JPH_ObjectLayer layer)
{
    return JPH::ObjectLayerPairFilterMask::sGetMask(layer);
}

JPH_ObjectLayerPairFilter *JPH_ObjectLayerPairFilterTable_Create(const uint32_t numObjectLayers)
{
    JPH::ObjectLayerPairFilterTable *const filter = new JPH::ObjectLayerPairFilterTable(numObjectLayers);
    return reinterpret_cast<JPH_ObjectLayerPairFilter *>(filter);
}

void JPH_ObjectLayerPairFilterTable_DisableCollision(JPH_ObjectLayerPairFilter *objectFilter,
                                                     const JPH_ObjectLayer layer1,
                                                     const JPH_ObjectLayer layer2)
{
    reinterpret_cast<JPH::ObjectLayerPairFilterTable *>(objectFilter)->DisableCollision(layer1, layer2);
}

void JPH_ObjectLayerPairFilterTable_EnableCollision(JPH_ObjectLayerPairFilter *objectFilter,
                                                    const JPH_ObjectLayer layer1,
                                                    const JPH_ObjectLayer layer2)
{
    reinterpret_cast<JPH::ObjectLayerPairFilterTable *>(objectFilter)->EnableCollision(layer1, layer2);
}

bool JPH_ObjectLayerPairFilterTable_ShouldCollide(JPH_ObjectLayerPairFilter *objectFilter,
                                                  const JPH_ObjectLayer layer1,
                                                  const JPH_ObjectLayer layer2)
{
    return reinterpret_cast<JPH::ObjectLayerPairFilterTable *>(objectFilter)->ShouldCollide(layer1, layer2);
}

/* JPH_ObjectVsBroadPhaseLayerFilter */
JPH_ObjectVsBroadPhaseLayerFilter *JPH_ObjectVsBroadPhaseLayerFilterMask_Create(const JPH_BroadPhaseLayerInterface
                                                                                        *broadPhaseLayerInterface)
{
    const JPH::BroadPhaseLayerInterfaceMask *const joltBroadPhaseLayerInterface = reinterpret_cast<
            const JPH::BroadPhaseLayerInterfaceMask *>(broadPhaseLayerInterface);
    JPH::ObjectVsBroadPhaseLayerFilterMask
            *const filter = new JPH::ObjectVsBroadPhaseLayerFilterMask(*joltBroadPhaseLayerInterface);
    return reinterpret_cast<JPH_ObjectVsBroadPhaseLayerFilter *>(filter);
}

JPH_ObjectVsBroadPhaseLayerFilter *JPH_ObjectVsBroadPhaseLayerFilterTable_Create(JPH_BroadPhaseLayerInterface
                                                                                         *broadPhaseLayerInterface,
                                                                                 const uint32_t numBroadPhaseLayers,
                                                                                 JPH_ObjectLayerPairFilter
                                                                                         *objectLayerPairFilter,
                                                                                 const uint32_t numObjectLayers)
{
    const JPH::BroadPhaseLayerInterface *const
            joltBroadPhaseLayerInterface = reinterpret_cast<JPH::BroadPhaseLayerInterface *>(broadPhaseLayerInterface);
    const JPH::ObjectLayerPairFilter
            *const joltObjectLayerPairFilter = reinterpret_cast<JPH::ObjectLayerPairFilter *>(objectLayerPairFilter);

    JPH::ObjectVsBroadPhaseLayerFilterTable
            *const filter = new JPH::ObjectVsBroadPhaseLayerFilterTable(*joltBroadPhaseLayerInterface,
                                                                        numBroadPhaseLayers,
                                                                        *joltObjectLayerPairFilter,
                                                                        numObjectLayers);
    return reinterpret_cast<JPH_ObjectVsBroadPhaseLayerFilter *>(filter);
}

void JPH_DrawSettings_InitDefault(JPH_DrawSettings *settings)
{
    JPH_ASSERT(settings);

    settings->drawGetSupportFunction = false;
    settings->drawSupportDirection = false;
    settings->drawGetSupportingFace = false;
    settings->drawShape = true;
    settings->drawShapeWireframe = false;
    settings->drawShapeColor = JPH_BodyManager_ShapeColor_MotionTypeColor;
    settings->drawBoundingBox = false;
    settings->drawCenterOfMassTransform = false;
    settings->drawWorldTransform = false;
    settings->drawVelocity = false;
    settings->drawMassAndInertia = false;
    settings->drawSleepStats = false;
    settings->drawSoftBodyVertices = false;
    settings->drawSoftBodyVertexVelocities = false;
    settings->drawSoftBodyEdgeConstraints = false;
    settings->drawSoftBodyBendConstraints = false;
    settings->drawSoftBodyVolumeConstraints = false;
    settings->drawSoftBodySkinConstraints = false;
    settings->drawSoftBodyLRAConstraints = false;
    settings->drawSoftBodyPredictedBounds = false;
    settings->drawSoftBodyConstraintColor = JPH_SoftBodyConstraintColor_ConstraintType;
}

/* JPH_PhysicsSystem */
struct JPH_PhysicsSystem final
{
        JPH::BroadPhaseLayerInterface *broadPhaseLayerInterface = nullptr;
        JPH::ObjectLayerPairFilter *objectLayerPairFilter = nullptr;
        JPH::ObjectVsBroadPhaseLayerFilter *objectVsBroadPhaseLayerFilter = nullptr;
        JPH::PhysicsSystem *physicsSystem = nullptr;
};
static JPH::UnorderedMap<JPH::PhysicsSystem *, JPH_PhysicsSystem *> s_PhysicsSystems;

JPH_PhysicsSystem *JPH_PhysicsSystem_Create(const JPH_PhysicsSystemSettings *settings)
{
    if (settings == nullptr)
    {
        return nullptr;
    }

    JPH_PhysicsSystem *system = new JPH_PhysicsSystem();
    system->broadPhaseLayerInterface = reinterpret_cast<
            JPH::BroadPhaseLayerInterface *>(settings->broadPhaseLayerInterface);
    system->objectLayerPairFilter = reinterpret_cast<JPH::ObjectLayerPairFilter *>(settings->objectLayerPairFilter);
    system->objectVsBroadPhaseLayerFilter = reinterpret_cast<
            JPH::ObjectVsBroadPhaseLayerFilter *>(settings->objectVsBroadPhaseLayerFilter);

    // Init the physics system
    const uint32_t maxBodies = settings->maxBodies != 0u ? settings->maxBodies : 10240;
    const uint32_t cNumBodyMutexes = settings->numBodyMutexes;
    const uint32_t maxBodyPairs = settings->maxBodyPairs != 0u ? settings->maxBodyPairs : 65536;
    const uint32_t maxContactConstraints = settings->maxContactConstraints != 0u ? settings->maxContactConstraints
                                                                                 : 10240;
    system->physicsSystem = new JPH::PhysicsSystem();
    system->physicsSystem->Init(maxBodies,
                                cNumBodyMutexes,
                                maxBodyPairs,
                                maxContactConstraints,
                                *system->broadPhaseLayerInterface,
                                *system->objectVsBroadPhaseLayerFilter,
                                *system->objectLayerPairFilter);

    s_PhysicsSystems[system->physicsSystem] = system;
    return system;
}

void JPH_PhysicsSystem_Destroy(const JPH_PhysicsSystem *system)
{
    if (system != nullptr)
    {
        s_PhysicsSystems.erase(system->physicsSystem);
        delete system->physicsSystem;
        delete system->broadPhaseLayerInterface;
        delete system->objectVsBroadPhaseLayerFilter;
        delete system->objectLayerPairFilter;

        delete system;
    }
}

void JPH_PhysicsSystem_SetPhysicsSettings(const JPH_PhysicsSystem *system, const JPH_PhysicsSettings *settings)
{
    JPH::PhysicsSettings joltSettings;
    joltSettings.mMaxInFlightBodyPairs = settings->maxInFlightBodyPairs;
    joltSettings.mStepListenersBatchSize = settings->stepListenersBatchSize;
    joltSettings.mStepListenerBatchesPerJob = settings->stepListenerBatchesPerJob;
    joltSettings.mBaumgarte = settings->baumgarte;
    joltSettings.mSpeculativeContactDistance = settings->speculativeContactDistance;
    joltSettings.mPenetrationSlop = settings->penetrationSlop;
    joltSettings.mLinearCastThreshold = settings->linearCastThreshold;
    joltSettings.mLinearCastMaxPenetration = settings->linearCastMaxPenetration;
    joltSettings.mManifoldTolerance = settings->manifoldTolerance;
    joltSettings.mMaxPenetrationDistance = settings->maxPenetrationDistance;
    joltSettings.mBodyPairCacheMaxDeltaPositionSq = settings->bodyPairCacheMaxDeltaPositionSq;
    joltSettings.mBodyPairCacheCosMaxDeltaRotationDiv2 = settings->bodyPairCacheCosMaxDeltaRotationDiv2;
    joltSettings.mContactNormalCosMaxDeltaRotation = settings->contactNormalCosMaxDeltaRotation;
    joltSettings.mContactPointPreserveLambdaMaxDistSq = settings->contactPointPreserveLambdaMaxDistSq;
    joltSettings.mNumVelocitySteps = settings->numVelocitySteps;
    joltSettings.mNumPositionSteps = settings->numPositionSteps;
    joltSettings.mMinVelocityForRestitution = settings->minVelocityForRestitution;
    joltSettings.mTimeBeforeSleep = settings->timeBeforeSleep;
    joltSettings.mPointVelocitySleepThreshold = settings->pointVelocitySleepThreshold;
    joltSettings.mDeterministicSimulation = settings->deterministicSimulation;
    joltSettings.mConstraintWarmStart = settings->constraintWarmStart;
    joltSettings.mUseBodyPairContactCache = settings->useBodyPairContactCache;
    joltSettings.mUseManifoldReduction = settings->useManifoldReduction;
    joltSettings.mUseLargeIslandSplitter = settings->useLargeIslandSplitter;
    joltSettings.mAllowSleeping = settings->allowSleeping;
    joltSettings.mCheckActiveEdges = settings->checkActiveEdges;
    system->physicsSystem->SetPhysicsSettings(joltSettings);
}

void JPH_PhysicsSystem_GetPhysicsSettings(const JPH_PhysicsSystem *system, JPH_PhysicsSettings *result)
{
    const JPH::PhysicsSettings joltSettings = system->physicsSystem->GetPhysicsSettings();
    result->maxInFlightBodyPairs = joltSettings.mMaxInFlightBodyPairs;
    result->stepListenersBatchSize = joltSettings.mStepListenersBatchSize;
    result->stepListenerBatchesPerJob = joltSettings.mStepListenerBatchesPerJob;
    result->baumgarte = joltSettings.mBaumgarte;
    result->speculativeContactDistance = joltSettings.mSpeculativeContactDistance;
    result->penetrationSlop = joltSettings.mPenetrationSlop;
    result->linearCastThreshold = joltSettings.mLinearCastThreshold;
    result->linearCastMaxPenetration = joltSettings.mLinearCastMaxPenetration;
    result->manifoldTolerance = joltSettings.mManifoldTolerance;
    result->maxPenetrationDistance = joltSettings.mMaxPenetrationDistance;
    result->bodyPairCacheMaxDeltaPositionSq = joltSettings.mBodyPairCacheMaxDeltaPositionSq;
    result->bodyPairCacheCosMaxDeltaRotationDiv2 = joltSettings.mBodyPairCacheCosMaxDeltaRotationDiv2;
    result->contactNormalCosMaxDeltaRotation = joltSettings.mContactNormalCosMaxDeltaRotation;
    result->contactPointPreserveLambdaMaxDistSq = joltSettings.mContactPointPreserveLambdaMaxDistSq;
    result->numVelocitySteps = joltSettings.mNumVelocitySteps;
    result->numPositionSteps = joltSettings.mNumPositionSteps;
    result->minVelocityForRestitution = joltSettings.mMinVelocityForRestitution;
    result->timeBeforeSleep = joltSettings.mTimeBeforeSleep;
    result->pointVelocitySleepThreshold = joltSettings.mPointVelocitySleepThreshold;
    result->deterministicSimulation = joltSettings.mDeterministicSimulation;
    result->constraintWarmStart = joltSettings.mConstraintWarmStart;
    result->useBodyPairContactCache = joltSettings.mUseBodyPairContactCache;
    result->useManifoldReduction = joltSettings.mUseManifoldReduction;
    result->useLargeIslandSplitter = joltSettings.mUseLargeIslandSplitter;
    result->allowSleeping = joltSettings.mAllowSleeping;
    result->checkActiveEdges = joltSettings.mCheckActiveEdges;
}

void JPH_PhysicsSystem_OptimizeBroadPhase(JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    system->physicsSystem->OptimizeBroadPhase();
}

JPH_PhysicsUpdateError JPH_PhysicsSystem_Update(const JPH_PhysicsSystem *system,
                                                const float deltaTime,
                                                const int collisionSteps,
                                                JPH_JobSystem *jobSystem)
{
    JPH::JobSystem *joltJobSystem = reinterpret_cast<JPH::JobSystem *>(jobSystem);
    return static_cast<JPH_PhysicsUpdateError>(system->physicsSystem->Update(deltaTime,
                                                                             collisionSteps,
                                                                             s_TempAllocator,
                                                                             joltJobSystem));
}

JPH_BodyInterface *JPH_PhysicsSystem_GetBodyInterface(const JPH_PhysicsSystem *system)
{
    return reinterpret_cast<JPH_BodyInterface *>(&system->physicsSystem->GetBodyInterface());
}

JPH_BodyInterface *JPH_PhysicsSystem_GetBodyInterfaceNoLock(const JPH_PhysicsSystem *system)
{
    return reinterpret_cast<JPH_BodyInterface *>(&system->physicsSystem->GetBodyInterfaceNoLock());
}

const JPH_BodyLockInterface *JPH_PhysicsSystem_GetBodyLockInterface(const JPH_PhysicsSystem *system)
{
    return reinterpret_cast<const JPH_BodyLockInterface *>(&system->physicsSystem->GetBodyLockInterface());
}
const JPH_BodyLockInterface *JPH_PhysicsSystem_GetBodyLockInterfaceNoLock(const JPH_PhysicsSystem *system)
{
    return reinterpret_cast<const JPH_BodyLockInterface *>(&system->physicsSystem->GetBodyLockInterfaceNoLock());
}

/* JPH_BroadPhaseLayerFilter */
static const JPH::BroadPhaseLayerFilter &ToJolt(JPH_BroadPhaseLayerFilter *bpFilter)
{
    static const JPH::BroadPhaseLayerFilter g_defaultBroadPhaseLayerFilter = {};
    return bpFilter != nullptr ? *reinterpret_cast<JPH::BroadPhaseLayerFilter *>(bpFilter)
                               : g_defaultBroadPhaseLayerFilter;
}

class ManagedBroadPhaseLayerFilter final: public JPH::BroadPhaseLayerFilter
{
    public:
        static const JPH_BroadPhaseLayerFilter_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedBroadPhaseLayerFilter(void *userData_): userData(userData_) {}

        [[nodiscard]] bool ShouldCollide(const JPH::BroadPhaseLayer inLayer) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollide != nullptr)
            {
                return s_Procs->ShouldCollide(userData, static_cast<JPH_BroadPhaseLayer>(inLayer));
            }

            return true;
        }
};

const JPH_BroadPhaseLayerFilter_Procs *ManagedBroadPhaseLayerFilter::s_Procs = nullptr;

void JPH_BroadPhaseLayerFilter_SetProcs(const JPH_BroadPhaseLayerFilter_Procs *procs)
{
    ManagedBroadPhaseLayerFilter::s_Procs = procs;
}

JPH_BroadPhaseLayerFilter *JPH_BroadPhaseLayerFilter_Create(void *userData)
{
    ManagedBroadPhaseLayerFilter *const filter = new ManagedBroadPhaseLayerFilter(userData);
    return reinterpret_cast<JPH_BroadPhaseLayerFilter *>(filter);
}

void JPH_BroadPhaseLayerFilter_Destroy(JPH_BroadPhaseLayerFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedBroadPhaseLayerFilter *>(filter);
    }
}

/* JPH_ObjectLayerFilter */
static const JPH::ObjectLayerFilter &ToJolt(JPH_ObjectLayerFilter *opFilter)
{
    static const JPH::ObjectLayerFilter g_defaultObjectLayerFilter = {};
    return opFilter != nullptr ? *reinterpret_cast<JPH::ObjectLayerFilter *>(opFilter) : g_defaultObjectLayerFilter;
}

class ManagedObjectLayerFilter final: public JPH::ObjectLayerFilter
{
    public:
        static const JPH_ObjectLayerFilter_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedObjectLayerFilter(void *userData_): userData(userData_) {}

        [[nodiscard]] bool ShouldCollide(const JPH::ObjectLayer inLayer) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollide != nullptr)
            {
                return s_Procs->ShouldCollide(userData, inLayer);
            }

            return true;
        }
};

const JPH_ObjectLayerFilter_Procs *ManagedObjectLayerFilter::s_Procs = nullptr;

void JPH_ObjectLayerFilter_SetProcs(const JPH_ObjectLayerFilter_Procs *procs)
{
    ManagedObjectLayerFilter::s_Procs = procs;
}

JPH_ObjectLayerFilter *JPH_ObjectLayerFilter_Create(void *userData)
{
    ManagedObjectLayerFilter *const filter = new ManagedObjectLayerFilter(userData);
    return reinterpret_cast<JPH_ObjectLayerFilter *>(filter);
}

void JPH_ObjectLayerFilter_Destroy(JPH_ObjectLayerFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedObjectLayerFilter *>(filter);
    }
}

/* JPH_BodyFilter */
static const JPH::BodyFilter &ToJolt(const JPH_BodyFilter *bodyFilter)
{
    static const JPH::BodyFilter g_defaultBodyFilter = {};
    return bodyFilter != nullptr ? *reinterpret_cast<const JPH::BodyFilter *>(bodyFilter) : g_defaultBodyFilter;
}

class ManagedBodyFilter final: public JPH::BodyFilter
{
    public:
        static const JPH_BodyFilter_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedBodyFilter(void *userData_): userData(userData_) {}

        [[nodiscard]] bool ShouldCollide(const JPH::BodyID &bodyID) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollide != nullptr)
            {
                return s_Procs->ShouldCollide(userData, bodyID.GetIndexAndSequenceNumber());
            }

            return true;
        }

        [[nodiscard]] bool ShouldCollideLocked(const JPH::Body &body) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollideLocked != nullptr)
            {
                return s_Procs->ShouldCollideLocked(userData, reinterpret_cast<const JPH_Body *>(&body));
            }

            return true;
        }
};

const JPH_BodyFilter_Procs *ManagedBodyFilter::s_Procs = nullptr;

void JPH_BodyFilter_SetProcs(const JPH_BodyFilter_Procs *procs)
{
    ManagedBodyFilter::s_Procs = procs;
}

JPH_BodyFilter *JPH_BodyFilter_Create(void *userData)
{
    ManagedBodyFilter *const filter = new ManagedBodyFilter(userData);
    return reinterpret_cast<JPH_BodyFilter *>(filter);
}

void JPH_BodyFilter_Destroy(JPH_BodyFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedBodyFilter *>(filter);
    }
}

/* JPH_ShapeFilter */
static const JPH::ShapeFilter &ToJolt(const JPH_ShapeFilter *filter)
{
    static const JPH::ShapeFilter g_defaultBodyFilter = {};
    return filter != nullptr ? *reinterpret_cast<const JPH::ShapeFilter *>(filter) : g_defaultBodyFilter;
}

class ManagedShapeFilter final: public JPH::ShapeFilter
{
    public:
        static const JPH_ShapeFilter_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedShapeFilter(void *userData_): userData(userData_) {}

        bool ShouldCollide([[maybe_unused]] const JPH::Shape *inShape2,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape2) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollide != nullptr)
            {
                const JPH::uint32 subShapeIDOfShape2 = inSubShapeIDOfShape2.GetValue();
                return s_Procs->ShouldCollide(userData, ToShape(inShape2), &subShapeIDOfShape2);
            }

            return true;
        }

        bool ShouldCollide([[maybe_unused]] const JPH::Shape *inShape1,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape1,
                           [[maybe_unused]] const JPH::Shape *inShape2,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape2) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollide2 != nullptr)
            {
                const JPH::uint32 subShapeIDOfShape1 = inSubShapeIDOfShape1.GetValue();
                const JPH::uint32 subShapeIDOfShape2 = inSubShapeIDOfShape2.GetValue();

                return s_Procs->ShouldCollide2(userData,
                                               ToShape(inShape1),
                                               &subShapeIDOfShape1,
                                               ToShape(inShape2),
                                               &subShapeIDOfShape2);
            }

            return true;
        }
};

const JPH_ShapeFilter_Procs *ManagedShapeFilter::s_Procs = nullptr;

void JPH_ShapeFilter_SetProcs(const JPH_ShapeFilter_Procs *procs)
{
    ManagedShapeFilter::s_Procs = procs;
}

JPH_ShapeFilter *JPH_ShapeFilter_Create(void *userData)
{
    ManagedShapeFilter *const filter = new ManagedShapeFilter(userData);
    return reinterpret_cast<JPH_ShapeFilter *>(filter);
}

void JPH_ShapeFilter_Destroy(JPH_ShapeFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedShapeFilter *>(filter);
    }
}

JPH_BodyID JPH_ShapeFilter_GetBodyID2(JPH_ShapeFilter *filter)
{
    return reinterpret_cast<ManagedShapeFilter *>(filter)->mBodyID2.GetIndexAndSequenceNumber();
}

void JPH_ShapeFilter_SetBodyID2(JPH_ShapeFilter *filter, const JPH_BodyID id)
{
    reinterpret_cast<ManagedShapeFilter *>(filter)->mBodyID2 = JPH::BodyID(id);
}

/* JPH_SimShapeFilter */
static const JPH::SimShapeFilter &ToJolt(const JPH_SimShapeFilter *filter)
{
    static const JPH::SimShapeFilter g_defaultSimShapeFilter = {};
    return filter != nullptr ? *reinterpret_cast<const JPH::SimShapeFilter *>(filter) : g_defaultSimShapeFilter;
}

class ManagedSimShapeFilter final: public JPH::SimShapeFilter
{
    public:
        static const JPH_SimShapeFilter_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedSimShapeFilter(void *userData_): userData(userData_) {}

        bool ShouldCollide([[maybe_unused]] const JPH::Body &inBody1,
                           [[maybe_unused]] const JPH::Shape *inShape1,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape1,
                           [[maybe_unused]] const JPH::Body &inBody2,
                           [[maybe_unused]] const JPH::Shape *inShape2,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape2) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldCollide != nullptr)
            {
                const JPH::uint32 subShapeIDOfShape1 = inSubShapeIDOfShape1.GetValue();
                const JPH::uint32 subShapeIDOfShape2 = inSubShapeIDOfShape2.GetValue();
                return s_Procs->ShouldCollide(userData,
                                              reinterpret_cast<const JPH_Body *>(&inBody1),
                                              ToShape(inShape1),
                                              &subShapeIDOfShape1,
                                              reinterpret_cast<const JPH_Body *>(&inBody2),
                                              ToShape(inShape2),
                                              &subShapeIDOfShape2);
            }

            return true;
        }
};

const JPH_SimShapeFilter_Procs *ManagedSimShapeFilter::s_Procs = nullptr;

void JPH_SimShapeFilter_SetProcs(const JPH_SimShapeFilter_Procs *procs)
{
    ManagedSimShapeFilter::s_Procs = procs;
}

JPH_SimShapeFilter *JPH_SimShapeFilter_Create(void *userData)
{
    ManagedSimShapeFilter *const filter = new ManagedSimShapeFilter(userData);
    return reinterpret_cast<JPH_SimShapeFilter *>(filter);
}

void JPH_SimShapeFilter_Destroy(JPH_SimShapeFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedSimShapeFilter *>(filter);
    }
}

/* Math */
void JPH_Quaternion_FromTo(const JPH_Vec3 *from, const JPH_Vec3 *to, JPH_Quat *quat)
{
    FromJolt(JPH::Quat::sFromTo(ToJolt(from), ToJolt(to)), quat);
}

void JPH_Quat_GetAxisAngle(const JPH_Quat *quat, JPH_Vec3 *outAxis, float *outAngle)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(outAxis);
    JPH_ASSERT(outAngle);

    JPH::Vec3 joltAxis;
    float angle;
    const JPH::Quat joltQuat = ToJolt(quat);
    joltQuat.GetAxisAngle(joltAxis, angle);
    FromJolt(joltAxis, outAxis);
    *outAngle = angle;
}

void JPH_Quat_GetEulerAngles(const JPH_Quat *quat, JPH_Vec3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).GetEulerAngles(), result);
}

void JPH_Quat_RotateAxisX(const JPH_Quat *quat, JPH_Vec3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).RotateAxisX(), result);
}

void JPH_Quat_RotateAxisY(const JPH_Quat *quat, JPH_Vec3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).RotateAxisY(), result);
}

void JPH_Quat_RotateAxisZ(const JPH_Quat *quat, JPH_Vec3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).RotateAxisZ(), result);
}

void JPH_Quat_Inversed(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).Inversed(), result);
}

void JPH_Quat_GetPerpendicular(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).GetPerpendicular(), result);
}

float JPH_Quat_GetRotationAngle(const JPH_Quat *quat, const JPH_Vec3 *axis)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(axis);

    return ToJolt(quat).GetRotationAngle(ToJolt(axis));
}

void JPH_Quat_Multiply(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result)
{
    JPH_ASSERT(q1 && q2 && result);
    FromJolt(ToJolt(q1) * ToJolt(q2), result);
}

void JPH_Quat_MultiplyScalar(const JPH_Quat *q, float scalar, JPH_Quat *result)
{
    JPH_ASSERT(q && result);
    FromJolt(ToJolt(q) * scalar, result);
}

void JPH_Quat_Add(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result)
{
    JPH_ASSERT(q1 && q2 && result);
    FromJolt(ToJolt(q1) + ToJolt(q2), result);
}

void JPH_Quat_Subtract(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result)
{
    JPH_ASSERT(q1 && q2 && result);
    FromJolt(ToJolt(q1) - ToJolt(q2), result);
}

void JPH_Quat_DivideScalar(const JPH_Quat *q, float scalar, JPH_Quat *result)
{
    JPH_ASSERT(q && result);
    JPH_ASSERT(scalar != 0.0f);
    FromJolt(ToJolt(q) / scalar, result);
}

void JPH_Quat_Dot(const JPH_Quat *q1, const JPH_Quat *q2, float *result)
{
    JPH_ASSERT(q1 && q2 && result);
    *result = ToJolt(q1).Dot(ToJolt(q2));
}

void JPH_Quat_Conjugated(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat && result);
    FromJolt(ToJolt(quat).Conjugated(), result);
}

void JPH_Quat_GetTwist(const JPH_Quat *quat, const JPH_Vec3 *axis, JPH_Quat *result)
{
    JPH_ASSERT(quat && axis && result);
    FromJolt(ToJolt(quat).GetTwist(ToJolt(axis)), result);
}

void JPH_Quat_GetSwingTwist(const JPH_Quat *quat, JPH_Quat *outSwing, JPH_Quat *outTwist)
{
    JPH_ASSERT(quat && outSwing && outTwist);
    JPH::Quat swing;
    JPH::Quat twist;
    ToJolt(quat).GetSwingTwist(swing, twist);
    FromJolt(swing, outSwing);
    FromJolt(twist, outTwist);
}

void JPH_Quat_Lerp(const JPH_Quat *from, const JPH_Quat *to, float fraction, JPH_Quat *result)
{
    JPH_ASSERT(from && to && result);
    FromJolt(ToJolt(from).LERP(ToJolt(to), fraction), result);
}

void JPH_Quat_Slerp(const JPH_Quat *from, const JPH_Quat *to, float fraction, JPH_Quat *result)
{
    JPH_ASSERT(from && to && result);
    FromJolt(ToJolt(from).SLERP(ToJolt(to), fraction), result);
}

void JPH_Quat_Rotate(const JPH_Quat *quat, const JPH_Vec3 *vec, JPH_Vec3 *result)
{
    JPH_ASSERT(quat && vec && result);
    FromJolt(ToJolt(quat) * ToJolt(vec), result);
}

void JPH_Quat_InverseRotate(const JPH_Quat *quat, const JPH_Vec3 *vec, JPH_Vec3 *result)
{
    JPH_ASSERT(quat && vec && result);
    FromJolt(ToJolt(quat).InverseRotate(ToJolt(vec)), result);
}

void JPH_Quat_FromEulerAngles(const JPH_Vec3 *angles, JPH_Quat *result)
{
    JPH_ASSERT(angles && result);
    FromJolt(JPH::Quat::sEulerAngles(ToJolt(angles)), result);
}

bool JPH_Vec3_IsClose(const JPH_Vec3 *v1, const JPH_Vec3 *v2, float maxDistSq)
{
    JPH_ASSERT(v1 != nullptr);
    JPH_ASSERT(v2 != nullptr);

    return ToJolt(v1).IsClose(ToJolt(v2), maxDistSq);
}

bool JPH_Vec3_IsNearZero(const JPH_Vec3 *v, float maxDistSq)
{
    JPH_ASSERT(v != nullptr);

    return ToJolt(v).IsNearZero(maxDistSq);
}

bool JPH_Vec3_IsNormalized(const JPH_Vec3 *v, float tolerance)
{
    JPH_ASSERT(v != nullptr);

    return ToJolt(v).IsNormalized(tolerance);
}

bool JPH_Vec3_IsNaN(const JPH_Vec3 *v)
{
    JPH_ASSERT(v != nullptr);

    return ToJolt(v).IsNaN();
}

void JPH_Vec3_Negate(const JPH_Vec3 *v, JPH_Vec3 *result)
{
    JPH_ASSERT(v != nullptr);
    JPH_ASSERT(result != nullptr);

    FromJolt(-ToJolt(v), result);
}

void JPH_Vec3_Normalized(const JPH_Vec3 *v, JPH_Vec3 *result)
{
    JPH_ASSERT(v != nullptr);
    JPH_ASSERT(result != nullptr);

    FromJolt(ToJolt(v).Normalized(), result);
}

void JPH_Vec3_Cross(const JPH_Vec3 *v1, const JPH_Vec3 *v2, JPH_Vec3 *result)
{
    JPH_ASSERT(v1 != nullptr);
    JPH_ASSERT(v2 != nullptr);
    JPH_ASSERT(result != nullptr);

    FromJolt(ToJolt(v1).Cross(ToJolt(v2)), result);
}

void JPH_Vec3_Abs(const JPH_Vec3 *v, JPH_Vec3 *result)
{
    JPH_ASSERT(v != nullptr);
    JPH_ASSERT(result != nullptr);

    FromJolt(ToJolt(v).Abs(), result);
}

float JPH_Vec3_Length(const JPH_Vec3 *v)
{
    JPH_ASSERT(v);

    return ToJolt(v).Length();
}

float JPH_Vec3_LengthSquared(const JPH_Vec3 *v)
{
    JPH_ASSERT(v);

    return ToJolt(v).LengthSq();
}

void JPH_Vec3_Multiply(const JPH_Vec3 *v1, const JPH_Vec3 *v2, JPH_Vec3 *result)
{
    JPH_ASSERT(v1 && v2 && result);

    FromJolt(ToJolt(v1) * ToJolt(v2), result);
}

void JPH_Vec3_MultiplyScalar(const JPH_Vec3 *v, float scalar, JPH_Vec3 *result)
{
    JPH_ASSERT(v && result);

    FromJolt(ToJolt(v) * scalar, result);
}

void JPH_Vec3_Divide(const JPH_Vec3 *v1, const JPH_Vec3 *v2, JPH_Vec3 *result)
{
    JPH_ASSERT(v1 && v2 && result);

    FromJolt(ToJolt(v1) / ToJolt(v2), result);
}

void JPH_Vec3_DivideScalar(const JPH_Vec3 *v, float scalar, JPH_Vec3 *result)
{
    JPH_ASSERT(v && result);
    JPH_ASSERT(scalar != 0.0f);

    FromJolt(ToJolt(v) / scalar, result);
}

void JPH_Vec3_DotProduct(const JPH_Vec3 *v1, const JPH_Vec3 *v2, float *result)
{
    JPH_ASSERT(v1 && v2 && result);

    *result = ToJolt(v1).Dot(ToJolt(v2));
}

void JPH_Vec3_Normalize(const JPH_Vec3 *v, JPH_Vec3 *result)
{
    JPH_ASSERT(v && result);

    FromJolt(ToJolt(v).Normalized(), result);
}

void JPH_Vec3_Add(const JPH_Vec3 *v1, const JPH_Vec3 *v2, JPH_Vec3 *result)
{
    JPH_ASSERT(v1 && v2 && result);

    FromJolt(ToJolt(v1) + ToJolt(v2), result);
}

void JPH_Vec3_Subtract(const JPH_Vec3 *v1, const JPH_Vec3 *v2, JPH_Vec3 *result)
{
    JPH_ASSERT(v1 && v2 && result);

    FromJolt(ToJolt(v1) - ToJolt(v2), result);
}

void JPH_Matrix4x4_Add(const JPH_Matrix4x4 *m1, const JPH_Matrix4x4 *m2, JPH_Matrix4x4 *result)
{
    JPH_ASSERT(m1 && m2 && result);
    FromJolt(ToJolt(m1) + ToJolt(m2), result);
}

void JPH_Matrix4x4_Subtract(const JPH_Matrix4x4 *m1, const JPH_Matrix4x4 *m2, JPH_Matrix4x4 *result)
{
    JPH_ASSERT(m1 && m2 && result);
    FromJolt(ToJolt(m1) - ToJolt(m2), result);
}

void JPH_Matrix4x4_Multiply(const JPH_Matrix4x4 *m1, const JPH_Matrix4x4 *m2, JPH_Matrix4x4 *result)
{
    JPH_ASSERT(m1 && m2 && result);
    FromJolt(ToJolt(m1) * ToJolt(m2), result);
}

void JPH_Matrix4x4_MultiplyScalar(const JPH_Matrix4x4 *m, float scalar, JPH_Matrix4x4 *result)
{
    JPH_ASSERT(m && result);
    FromJolt(ToJolt(m) * scalar, result);
}

void JPH_Matrix4x4_Zero(JPH_Matrix4x4 *result)
{
    FromJolt(JPH::Mat44::sZero(), result);
}

void JPH_Matrix4x4_Identity(JPH_Matrix4x4 *result)
{
    FromJolt(JPH::Mat44::sIdentity(), result);
}

void JPH_Matrix4x4_Rotation(JPH_Matrix4x4 *result, const JPH_Quat *rotation)
{
    FromJolt(JPH::Mat44::sRotation(ToJolt(rotation)), result);
}

void JPH_Matrix4x4_Translation(JPH_Matrix4x4 *result, const JPH_Vec3 *translation)
{
    FromJolt(JPH::Mat44::sTranslation(ToJolt(translation)), result);
}

void JPH_Matrix4x4_RotationTranslation(JPH_Matrix4x4 *result, const JPH_Quat *rotation, const JPH_Vec3 *translation)
{
    FromJolt(JPH::Mat44::sRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_Matrix4x4_InverseRotationTranslation(JPH_Matrix4x4 *result,
                                              const JPH_Quat *rotation,
                                              const JPH_Vec3 *translation)
{
    FromJolt(JPH::Mat44::sInverseRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_Matrix4x4_Scale(JPH_Matrix4x4 *result, const JPH_Vec3 *scale)
{
    FromJolt(JPH::Mat44::sScale(ToJolt(scale)), result);
}

void JPH_Matrix4x4_Transposed(const JPH_Matrix4x4 *m, JPH_Matrix4x4 *result)
{
    JPH_ASSERT(m && result);
    FromJolt(ToJolt(m).Transposed(), result);
}

void JPH_Matrix4x4_Inversed(const JPH_Matrix4x4 *m, JPH_Matrix4x4 *result)
{
    JPH_ASSERT(m && result);
    FromJolt(ToJolt(m).Inversed(), result);
}

void JPH_RMatrix4x4_Zero(JPH_RMatrix4x4 *result)
{
    FromJolt(JPH::RMat44::sZero(), result);
}

void JPH_RMatrix4x4_Identity(JPH_RMatrix4x4 *result)
{
    FromJolt(JPH::RMat44::sIdentity(), result);
}

void JPH_RMatrix4x4_Rotation(JPH_RMatrix4x4 *result, const JPH_Quat *rotation)
{
    FromJolt(JPH::RMat44::sRotation(ToJolt(rotation)), result);
}

void JPH_RMatrix4x4_Translation(JPH_RMatrix4x4 *result, const JPH_RVec3 *translation)
{
    FromJolt(JPH::RMat44::sTranslation(ToJolt(translation)), result);
}

void JPH_RMatrix4x4_RotationTranslation(JPH_RMatrix4x4 *result, const JPH_Quat *rotation, const JPH_RVec3 *translation)
{
    FromJolt(JPH::RMat44::sRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_RMatrix4x4_InverseRotationTranslation(JPH_RMatrix4x4 *result,
                                               const JPH_Quat *rotation,
                                               const JPH_RVec3 *translation)
{
    FromJolt(JPH::RMat44::sInverseRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_RMatrix4x4_Scale(JPH_RMatrix4x4 *result, const JPH_Vec3 *scale)
{
    FromJolt(JPH::RMat44::sScale(ToJolt(scale)), result);
}

void JPH_RMatrix4x4_Inversed(const JPH_RMatrix4x4 *matrix, JPH_RMatrix4x4 *result)
{
    JPH_ASSERT(matrix && result);
    FromJolt(ToJolt(matrix).Inversed(), result);
}

void JPH_Matrix4x4_GetAxisX(const JPH_Matrix4x4 *matrix, JPH_Vec3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);
    FromJolt(ToJolt(matrix).GetAxisX(), result);
}

void JPH_Matrix4x4_GetAxisY(const JPH_Matrix4x4 *matrix, JPH_Vec3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);
    FromJolt(ToJolt(matrix).GetAxisY(), result);
}

void JPH_Matrix4x4_GetAxisZ(const JPH_Matrix4x4 *matrix, JPH_Vec3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);
    FromJolt(ToJolt(matrix).GetAxisZ(), result);
}

void JPH_Matrix4x4_GetTranslation(const JPH_Matrix4x4 *matrix, JPH_Vec3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);
    FromJolt(ToJolt(matrix).GetTranslation(), result);
}

void JPH_Matrix4x4_GetQuaternion(const JPH_Matrix4x4 *matrix, JPH_Quat *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);
    FromJolt(ToJolt(matrix).GetQuaternion(), result);
}

/* Material */
JPH_PhysicsMaterial *JPH_PhysicsMaterial_Create(const char *name, const uint32_t color)
{
    JPH::PhysicsMaterialSimple *const material = new JPH::PhysicsMaterialSimple(name, JPH::Color(color));
    material->AddRef();

    return ToPhysicsMaterial(material);
}

void JPH_PhysicsMaterial_Destroy(JPH_PhysicsMaterial *material)
{
    if (material != nullptr)
    {
        AsPhysicsMaterial(material)->Release();
    }
}

const char *JPH_PhysicsMaterial_GetDebugName(const JPH_PhysicsMaterial *material)
{
    return AsPhysicsMaterial(material)->GetDebugName();
}

uint32_t JPH_PhysicsMaterial_GetDebugColor(const JPH_PhysicsMaterial *material)
{
    return AsPhysicsMaterial(material)->GetDebugColor().GetUInt32();
}

/* GroupFilter/GroupFilterTable */
void JPH_GroupFilter_Destroy(JPH_GroupFilter *groupFilter)
{
    if (groupFilter != nullptr)
    {
        AsGroupFilter(groupFilter)->Release();
    }
}

bool JPH_GroupFilter_CanCollide(JPH_GroupFilter *groupFilter,
                                const JPH_CollisionGroup *group1,
                                const JPH_CollisionGroup *group2)
{
    return AsGroupFilter(groupFilter)->CanCollide(ToJolt(group1), ToJolt(group2));
}

JPH_GroupFilterTable *JPH_GroupFilterTable_Create(const uint32_t numSubGroups)
{
    JPH::GroupFilterTable *const material = new JPH::GroupFilterTable(numSubGroups);
    material->AddRef();

    return ToGroupFilterTable(material);
}

void JPH_GroupFilterTable_DisableCollision(JPH_GroupFilterTable *table,
                                           const JPH_CollisionSubGroupID subGroup1,
                                           const JPH_CollisionSubGroupID subGroup2)
{
    AsGroupFilterTable(table)->DisableCollision(subGroup1, subGroup2);
}

void JPH_GroupFilterTable_EnableCollision(JPH_GroupFilterTable *table,
                                          const JPH_CollisionSubGroupID subGroup1,
                                          const JPH_CollisionSubGroupID subGroup2)
{
    AsGroupFilterTable(table)->EnableCollision(subGroup1, subGroup2);
}

bool JPH_GroupFilterTable_IsCollisionEnabled(JPH_GroupFilterTable *table,
                                             const JPH_CollisionSubGroupID subGroup1,
                                             const JPH_CollisionSubGroupID subGroup2)
{
    return AsGroupFilterTable(table)->IsCollisionEnabled(subGroup1, subGroup2);
}

/* ShapeSettings */
void JPH_ShapeSettings_Destroy(JPH_ShapeSettings *settings)
{
    if (settings != nullptr)
    {
        const JPH::ShapeSettings *const joltSettings = reinterpret_cast<JPH::ShapeSettings *>(settings);
        joltSettings->Release();
    }
}

uint64_t JPH_ShapeSettings_GetUserData(const JPH_ShapeSettings *settings)
{
    return reinterpret_cast<const JPH::ShapeSettings *>(settings)->mUserData;
}

void JPH_ShapeSettings_SetUserData(JPH_ShapeSettings *settings, const uint64_t userData)
{
    reinterpret_cast<JPH::ShapeSettings *>(settings)->mUserData = userData;
}


/* Shape */
void JPH_Shape_Destroy(JPH_Shape *shape)
{
    AsShape(shape)->Release();
}

JPH_ShapeType JPH_Shape_GetType(const JPH_Shape *shape)
{
    return static_cast<JPH_ShapeType>(reinterpret_cast<const JPH::Shape *>(shape)->GetType());
}

JPH_ShapeSubType JPH_Shape_GetSubType(const JPH_Shape *shape)
{
    return static_cast<JPH_ShapeSubType>(reinterpret_cast<const JPH::Shape *>(shape)->GetSubType());
}

uint64_t JPH_Shape_GetUserData(const JPH_Shape *shape)
{
    return reinterpret_cast<const JPH::Shape *>(shape)->GetUserData();
}

void JPH_Shape_SetUserData(JPH_Shape *shape, const uint64_t userData)
{
    reinterpret_cast<JPH::Shape *>(shape)->SetUserData(userData);
}

bool JPH_Shape_MustBeStatic(const JPH_Shape *shape)
{
    return reinterpret_cast<const JPH::Shape *>(shape)->MustBeStatic();
}

void JPH_Shape_GetCenterOfMass(const JPH_Shape *shape, JPH_Vec3 *result)
{
    FromJolt(AsShape(shape)->GetCenterOfMass(), result);
}

void JPH_Shape_GetLocalBounds(const JPH_Shape *shape, JPH_AABox *result)
{
    FromJolt(AsShape(shape)->GetLocalBounds(), result);
}

uint32_t JPH_Shape_GetSubShapeIDBitsRecursive(const JPH_Shape *shape)
{
    return AsShape(shape)->GetSubShapeIDBitsRecursive();
}

void JPH_Shape_GetWorldSpaceBounds(const JPH_Shape *shape,
                                   const JPH_RMatrix4x4 *centerOfMassTransform,
                                   const JPH_Vec3 *scale,
                                   JPH_AABox *result)
{
    const JPH::AABox bounds = AsShape(shape)->GetWorldSpaceBounds(ToJolt(centerOfMassTransform), ToJolt(scale));
    FromJolt(bounds, result);
}

float JPH_Shape_GetInnerRadius(const JPH_Shape *shape)
{
    return AsShape(shape)->GetInnerRadius();
}

void JPH_Shape_GetMassProperties(const JPH_Shape *shape, JPH_MassProperties *result)
{
    FromJolt(AsShape(shape)->GetMassProperties(), result);
}

const JPH_Shape *JPH_Shape_GetLeafShape(const JPH_Shape *shape,
                                        const JPH_SubShapeID subShapeID,
                                        JPH_SubShapeID *remainder)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    JPH::SubShapeID joltRemainder = JPH::SubShapeID();
    const JPH::Shape *leaf = AsShape(shape)->GetLeafShape(joltSubShapeID, joltRemainder);
    *remainder = joltRemainder.GetValue();
    return reinterpret_cast<const JPH_Shape *>(leaf);
}

const JPH_PhysicsMaterial *JPH_Shape_GetMaterial(const JPH_Shape *shape, const JPH_SubShapeID subShapeID)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    return FromJolt(AsShape(shape)->GetMaterial(joltSubShapeID));
}

void JPH_Shape_GetSurfaceNormal(const JPH_Shape *shape,
                                const JPH_SubShapeID subShapeID,
                                const JPH_Vec3 *localPosition,
                                JPH_Vec3 *normal)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    const JPH::Vec3 joltNormal = AsShape(shape)->GetSurfaceNormal(joltSubShapeID, ToJolt(localPosition));
    FromJolt(joltNormal, normal);
}

void JPH_Shape_GetSupportingFace(const JPH_Shape *shape,
                                 const JPH_SubShapeID subShapeID,
                                 const JPH_Vec3 *direction,
                                 const JPH_Vec3 *scale,
                                 const JPH_Matrix4x4 *centerOfMassTransform,
                                 JPH_SupportingFace *outVertices)
{
    JPH_ASSERT(shape);
    JPH_ASSERT(subShapeID);
    JPH_ASSERT(direction);
    JPH_ASSERT(scale);
    JPH_ASSERT(centerOfMassTransform);
    JPH_ASSERT(outVertices);

    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);

    JPH::Shape::SupportingFace joltFace;
    AsShape(shape)->GetSupportingFace(joltSubShapeID,
                                      ToJolt(direction),
                                      ToJolt(scale),
                                      ToJolt(centerOfMassTransform),
                                      joltFace);

    outVertices->count = joltFace.size();
    JPH_ASSERT(outVertices->count <= 32);

    for (uint32_t i = 0; i < outVertices->count && i < 32; ++i)
    {
        FromJolt(joltFace[i], &outVertices->vertices[i]);
    }
}

float JPH_Shape_GetVolume(const JPH_Shape *shape)
{
    return AsShape(shape)->GetVolume();
}

bool JPH_Shape_IsValidScale(const JPH_Shape *shape, const JPH_Vec3 *scale)
{
    return AsShape(shape)->IsValidScale(ToJolt(scale));
}

void JPH_Shape_MakeScaleValid(const JPH_Shape *shape, const JPH_Vec3 *scale, JPH_Vec3 *result)
{
    FromJolt(AsShape(shape)->MakeScaleValid(ToJolt(scale)), result);
}

JPH_Shape *JPH_Shape_ScaleShape(const JPH_Shape *shape, const JPH_Vec3 *scale)
{
    const JPH::Result<JPH::Ref<JPH::Shape>> shapeResult = AsShape(shape)->ScaleShape(ToJolt(scale));
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    const JPH::Shape *scaleShape = shapeResult.Get().GetPtr();
    scaleShape->AddRef();

    return const_cast<JPH_Shape *>(ToShape(scaleShape));
}

bool JPH_Shape_CastRay(const JPH_Shape *shape,
                       const JPH_Vec3 *origin,
                       const JPH_Vec3 *direction,
                       JPH_RayCastResult *hit)
{
    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    constexpr JPH::SubShapeIDCreator creator;
    JPH::RayCastResult result;

    const bool hadHit = AsShape(shape)->CastRay(ray, creator, result);

    if (hadHit)
    {
        hit->fraction = result.mFraction;
        hit->bodyID = result.mBodyID.GetIndexAndSequenceNumber();
        hit->subShapeID2 = result.mSubShapeID2.GetValue();
    }

    return hadHit;
}

bool JPH_Shape_CastRay2(const JPH_Shape *shape,
                        const JPH_Vec3 *origin,
                        const JPH_Vec3 *direction,
                        const JPH_RayCastSettings *rayCastSettings,
                        const JPH_CollisionCollectorType collectorType,
                        JPH_CastRayResultCallback *callback,
                        void *userData,
                        const JPH_ShapeFilter *shapeFilter)
{
    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RayCastSettings settings = ToJolt(rayCastSettings);

    constexpr JPH::SubShapeIDCreator creator;
    JPH_RayCastResult hitResult{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            AsShape(shape)->CastRay(ray, settings, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::RayCastResult &hit: collector.mHits)
                {
                    hitResult.fraction = hit.mFraction;
                    hitResult.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    hitResult.subShapeID2 = hit.mSubShapeID2.GetValue();
                    callback(userData, &hitResult);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
            AsShape(shape)->CastRay(ray, settings, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CastRayCollector> collector;
            AsShape(shape)->CastRay(ray, settings, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_Shape_CollidePoint(const JPH_Shape *shape, const JPH_Vec3 *point, const JPH_ShapeFilter *shapeFilter)
{
    constexpr JPH::SubShapeIDCreator creator;
    JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> collector;

    AsShape(shape)->CollidePoint(ToJolt(point), creator, collector, ToJolt(shapeFilter));
    return collector.HadHit();
}

bool JPH_Shape_CollidePoint2(const JPH_Shape *shape,
                             const JPH_Vec3 *point,
                             const JPH_CollisionCollectorType collectorType,
                             JPH_CollidePointResultCallback *callback,
                             void *userData,
                             const JPH_ShapeFilter *shapeFilter)
{
    const JPH::Vec3 joltPoint = ToJolt(point);
    constexpr JPH::SubShapeIDCreator creator;
    JPH_CollidePointResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsShape(shape)->CollidePoint(joltPoint, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::CollidePointResult &hit: collector.mHits)
                {
                    result.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    result.subShapeID2 = hit.mSubShapeID2.GetValue();
                    callback(userData, &result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsShape(shape)->CollidePoint(joltPoint, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsShape(shape)->CollidePoint(joltPoint, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

/* ConvexShape */
float JPH_ConvexShapeSettings_GetDensity(const JPH_ConvexShapeSettings *shape)
{
    return reinterpret_cast<const JPH::ConvexShapeSettings *>(shape)->mDensity;
}

void JPH_ConvexShapeSettings_SetDensity(JPH_ConvexShapeSettings *shape, const float value)
{
    reinterpret_cast<JPH::ConvexShapeSettings *>(shape)->SetDensity(value);
}

float JPH_ConvexShape_GetDensity(const JPH_ConvexShape *shape)
{
    return reinterpret_cast<const JPH::ConvexShape *>(shape)->GetDensity();
}

void JPH_ConvexShape_SetDensity(JPH_ConvexShape *shape, const float density)
{
    reinterpret_cast<JPH::ConvexShape *>(shape)->SetDensity(density);
}

/* BoxShape */
JPH_BoxShapeSettings *JPH_BoxShapeSettings_Create(const JPH_Vec3 *halfExtent, const float convexRadius)
{
    JPH::BoxShapeSettings *const settings = new JPH::BoxShapeSettings(ToJolt(halfExtent), convexRadius);
    settings->AddRef();

    return reinterpret_cast<JPH_BoxShapeSettings *>(settings);
}

JPH_BoxShape *JPH_BoxShapeSettings_CreateShape(const JPH_BoxShapeSettings *settings)
{
    const JPH::BoxShapeSettings *joltSettings = reinterpret_cast<const JPH::BoxShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_result = joltSettings->Create();
    if (!shape_result.IsValid())
    {
        return nullptr;
    }

    const JPH::Shape *shape = shape_result.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_BoxShape *>(reinterpret_cast<const JPH_BoxShape *>(shape));
}

JPH_BoxShape *JPH_BoxShape_Create(const JPH_Vec3 *halfExtent, const float convexRadius)
{
    JPH::BoxShape *const shape = new JPH::BoxShape(ToJolt(halfExtent), convexRadius);
    shape->AddRef();

    return reinterpret_cast<JPH_BoxShape *>(shape);
}

void JPH_BoxShape_GetHalfExtent(const JPH_BoxShape *shape, JPH_Vec3 *halfExtent)
{
    const JPH::BoxShape *const joltShape = reinterpret_cast<const JPH::BoxShape *>(shape);
    const JPH::Vec3 joltVector = joltShape->GetHalfExtent();
    FromJolt(joltVector, halfExtent);
}

float JPH_BoxShape_GetConvexRadius(const JPH_BoxShape *shape)
{
    const JPH::BoxShape *const joltShape = reinterpret_cast<const JPH::BoxShape *>(shape);
    return joltShape->GetConvexRadius();
}

/* SphereShapeSettings */
JPH_SphereShapeSettings *JPH_SphereShapeSettings_Create(const float radius)
{
    JPH::SphereShapeSettings *const settings = new JPH::SphereShapeSettings(radius);
    settings->AddRef();

    return reinterpret_cast<JPH_SphereShapeSettings *>(settings);
}

JPH_SphereShape *JPH_SphereShapeSettings_CreateShape(const JPH_SphereShapeSettings *settings)
{
    const JPH::SphereShapeSettings *jolt_settings = reinterpret_cast<const JPH::SphereShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = jolt_settings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_SphereShape *>(reinterpret_cast<const JPH_SphereShape *>(shape));
}

float JPH_SphereShapeSettings_GetRadius(const JPH_SphereShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return reinterpret_cast<const JPH::SphereShapeSettings *>(settings)->mRadius;
}

void JPH_SphereShapeSettings_SetRadius(JPH_SphereShapeSettings *settings, float radius)
{
    JPH_ASSERT(settings);
    reinterpret_cast<JPH::SphereShapeSettings *>(settings)->mRadius = radius;
}

JPH_SphereShape *JPH_SphereShape_Create(const float radius)
{
    JPH::SphereShape *const shape = new JPH::SphereShape(radius);
    shape->AddRef();

    return reinterpret_cast<JPH_SphereShape *>(shape);
}

float JPH_SphereShape_GetRadius(const JPH_SphereShape *shape)
{
    return reinterpret_cast<const JPH::SphereShape *>(shape)->GetRadius();
}

/* PlaneShape */
JPH_PlaneShapeSettings *JPH_PlaneShapeSettings_Create(const JPH_Plane *plane,
                                                      const JPH_PhysicsMaterial *material,
                                                      const float halfExtent)
{
    const JPH::PhysicsMaterial *joltMaterial = material != nullptr
                                                       ? reinterpret_cast<const JPH::PhysicsMaterial *>(material)
                                                       : nullptr;

    JPH::PlaneShapeSettings *const settings = new JPH::PlaneShapeSettings(ToJolt(plane), joltMaterial, halfExtent);
    settings->AddRef();

    return reinterpret_cast<JPH_PlaneShapeSettings *>(settings);
}

JPH_PlaneShape *JPH_PlaneShapeSettings_CreateShape(const JPH_PlaneShapeSettings *settings)
{
    const JPH::PlaneShapeSettings *joltSettings = reinterpret_cast<const JPH::PlaneShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_PlaneShape *>(reinterpret_cast<const JPH_PlaneShape *>(shape));
}

JPH_PlaneShape *JPH_PlaneShape_Create(const JPH_Plane *plane,
                                      const JPH_PhysicsMaterial *material,
                                      const float halfExtent)
{
    const JPH::PhysicsMaterial *joltMaterial = material != nullptr
                                                       ? reinterpret_cast<const JPH::PhysicsMaterial *>(material)
                                                       : nullptr;

    JPH::PlaneShape *const shape = new JPH::PlaneShape(ToJolt(plane), joltMaterial, halfExtent);
    shape->AddRef();

    return reinterpret_cast<JPH_PlaneShape *>(shape);
}

void JPH_PlaneShape_GetPlane(const JPH_PlaneShape *shape, JPH_Plane *result)
{
    FromJolt(reinterpret_cast<const JPH::PlaneShape *>(shape)->GetPlane(), result);
}

float JPH_PlaneShape_GetHalfExtent(const JPH_PlaneShape *shape)
{
    return reinterpret_cast<const JPH::PlaneShape *>(shape)->GetHalfExtent();
}

/* TriangleShape */
JPH_TriangleShapeSettings *JPH_TriangleShapeSettings_Create(const JPH_Vec3 *v1,
                                                            const JPH_Vec3 *v2,
                                                            const JPH_Vec3 *v3,
                                                            const float convexRadius)
{
    JPH::TriangleShapeSettings *const settings = new JPH::TriangleShapeSettings(ToJolt(v1),
                                                                                ToJolt(v2),
                                                                                ToJolt(v3),
                                                                                convexRadius);
    settings->AddRef();

    return reinterpret_cast<JPH_TriangleShapeSettings *>(settings);
}

JPH_TriangleShape *JPH_TriangleShapeSettings_CreateShape(const JPH_TriangleShapeSettings *settings)
{
    const JPH::TriangleShapeSettings *joltSettings = reinterpret_cast<const JPH::TriangleShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_TriangleShape *>(reinterpret_cast<const JPH_TriangleShape *>(shape));
}

JPH_TriangleShape *JPH_TriangleShape_Create(const JPH_Vec3 *v1,
                                            const JPH_Vec3 *v2,
                                            const JPH_Vec3 *v3,
                                            const float convexRadius)
{
    JPH::TriangleShape *const shape = new JPH::TriangleShape(ToJolt(v1), ToJolt(v2), ToJolt(v3), convexRadius);
    shape->AddRef();

    return reinterpret_cast<JPH_TriangleShape *>(shape);
}

float JPH_TriangleShape_GetConvexRadius(const JPH_TriangleShape *shape)
{
    return reinterpret_cast<const JPH::TriangleShape *>(shape)->GetConvexRadius();
}

void JPH_TriangleShape_GetVertex1(const JPH_TriangleShape *shape, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<const JPH::TriangleShape *>(shape)->GetVertex1(), result);
}

void JPH_TriangleShape_GetVertex2(const JPH_TriangleShape *shape, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<const JPH::TriangleShape *>(shape)->GetVertex2(), result);
}

void JPH_TriangleShape_GetVertex3(const JPH_TriangleShape *shape, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<const JPH::TriangleShape *>(shape)->GetVertex3(), result);
}

/* CapsuleShapeSettings */
JPH_CapsuleShapeSettings *JPH_CapsuleShapeSettings_Create(const float halfHeightOfCylinder, const float radius)
{
    JPH::CapsuleShapeSettings *const settings = new JPH::CapsuleShapeSettings(halfHeightOfCylinder, radius);
    settings->AddRef();

    return reinterpret_cast<JPH_CapsuleShapeSettings *>(settings);
}

JPH_CapsuleShape *JPH_CapsuleShapeSettings_CreateShape(const JPH_CapsuleShapeSettings *settings)
{
    const JPH::CapsuleShapeSettings *joltSettings = reinterpret_cast<const JPH::CapsuleShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_CapsuleShape *>(reinterpret_cast<const JPH_CapsuleShape *>(shape));
}

JPH_CapsuleShape *JPH_CapsuleShape_Create(const float halfHeightOfCylinder, const float radius)
{
    JPH::CapsuleShape *const shape = new JPH::CapsuleShape(halfHeightOfCylinder, radius, nullptr);
    shape->AddRef();

    return reinterpret_cast<JPH_CapsuleShape *>(shape);
}

float JPH_CapsuleShape_GetRadius(const JPH_CapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return reinterpret_cast<const JPH::CapsuleShape *>(shape)->GetRadius();
}

float JPH_CapsuleShape_GetHalfHeightOfCylinder(const JPH_CapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return reinterpret_cast<const JPH::CapsuleShape *>(shape)->GetHalfHeightOfCylinder();
}

/* CylinderShapeSettings */
JPH_CylinderShapeSettings *JPH_CylinderShapeSettings_Create(const float halfHeight,
                                                            const float radius,
                                                            const float convexRadius)
{
    JPH::CylinderShapeSettings *const settings = new JPH::CylinderShapeSettings(halfHeight, radius, convexRadius);
    settings->AddRef();

    return reinterpret_cast<JPH_CylinderShapeSettings *>(settings);
}

JPH_CylinderShape *JPH_CylinderShapeSettings_CreateShape(const JPH_CylinderShapeSettings *settings)
{
    const JPH::CylinderShapeSettings *joltSettings = reinterpret_cast<const JPH::CylinderShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_CylinderShape *>(reinterpret_cast<const JPH_CylinderShape *>(shape));
}

JPH_CylinderShape *JPH_CylinderShape_Create(const float halfHeight, const float radius)
{
    JPH::CylinderShape *const shape = new JPH::CylinderShape(halfHeight, radius, 0.f, nullptr);
    shape->AddRef();

    return reinterpret_cast<JPH_CylinderShape *>(shape);
}

float JPH_CylinderShape_GetRadius(const JPH_CylinderShape *shape)
{
    return reinterpret_cast<const JPH::CylinderShape *>(shape)->GetRadius();
}

float JPH_CylinderShape_GetHalfHeight(const JPH_CylinderShape *shape)
{
    return reinterpret_cast<const JPH::CylinderShape *>(shape)->GetHalfHeight();
}

/* TaperedCylinderShape */
JPH_TaperedCylinderShapeSettings *JPH_TaperedCylinderShapeSettings_Create(
        const float halfHeightOfTaperedCylinder,
        const float topRadius,
        const float bottomRadius,
        const float convexRadius /* = cDefaultConvexRadius*/,
        const JPH_PhysicsMaterial *material /* = NULL*/)
{
    const JPH::PhysicsMaterial *joltMaterial = material != nullptr
                                                       ? reinterpret_cast<const JPH::PhysicsMaterial *>(material)
                                                       : nullptr;

    JPH::TaperedCylinderShapeSettings
            *const settings = new JPH::TaperedCylinderShapeSettings(halfHeightOfTaperedCylinder,
                                                                    topRadius,
                                                                    bottomRadius,
                                                                    convexRadius,
                                                                    joltMaterial);
    settings->AddRef();

    return reinterpret_cast<JPH_TaperedCylinderShapeSettings *>(settings);
}

JPH_TaperedCylinderShape *JPH_TaperedCylinderShapeSettings_CreateShape(const JPH_TaperedCylinderShapeSettings *settings)
{
    const JPH::TaperedCylinderShapeSettings
            *joltSettings = reinterpret_cast<const JPH::TaperedCylinderShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_TaperedCylinderShape *>(reinterpret_cast<const JPH_TaperedCylinderShape *>(shape));
}

float JPH_TaperedCylinderShape_GetTopRadius(const JPH_TaperedCylinderShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCylinderShape *>(shape)->GetTopRadius();
}

float JPH_TaperedCylinderShape_GetBottomRadius(const JPH_TaperedCylinderShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCylinderShape *>(shape)->GetBottomRadius();
}

float JPH_TaperedCylinderShape_GetConvexRadius(const JPH_TaperedCylinderShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCylinderShape *>(shape)->GetConvexRadius();
}

float JPH_TaperedCylinderShape_GetHalfHeight(const JPH_TaperedCylinderShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCylinderShape *>(shape)->GetHalfHeight();
}

/* ConvexHullShape */
JPH_ConvexHullShapeSettings *JPH_ConvexHullShapeSettings_Create(const JPH_Vec3 *points,
                                                                const uint32_t pointsCount,
                                                                const float maxConvexRadius)
{
    JPH::Array<JPH::Vec3> joltPoints;
    joltPoints.reserve(pointsCount);

    for (uint32_t i = 0; i < pointsCount; i++)
    {
        joltPoints.push_back(ToJolt(&points[i]));
    }

    JPH::ConvexHullShapeSettings *const settings = new JPH::ConvexHullShapeSettings(joltPoints, maxConvexRadius);
    settings->AddRef();

    return reinterpret_cast<JPH_ConvexHullShapeSettings *>(settings);
}

JPH_ConvexHullShape *JPH_ConvexHullShapeSettings_CreateShape(const JPH_ConvexHullShapeSettings *settings)
{
    const JPH::ConvexHullShapeSettings
            *jolt_settings = reinterpret_cast<const JPH::ConvexHullShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = jolt_settings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_ConvexHullShape *>(reinterpret_cast<const JPH_ConvexHullShape *>(shape));
}

uint32_t JPH_ConvexHullShape_GetNumPoints(const JPH_ConvexHullShape *shape)
{
    return reinterpret_cast<const JPH::ConvexHullShape *>(shape)->GetNumPoints();
}

void JPH_ConvexHullShape_GetPoint(const JPH_ConvexHullShape *shape, const uint32_t index, JPH_Vec3 *result)
{
    const JPH::Vec3 point = reinterpret_cast<const JPH::ConvexHullShape *>(shape)->GetPoint(index);
    FromJolt(point, result);
}

uint32_t JPH_ConvexHullShape_GetNumFaces(const JPH_ConvexHullShape *shape)
{
    return reinterpret_cast<const JPH::ConvexHullShape *>(shape)->GetNumFaces();
}

uint32_t JPH_ConvexHullShape_GetNumVerticesInFace(const JPH_ConvexHullShape *shape, const uint32_t faceIndex)
{
    return reinterpret_cast<const JPH::ConvexHullShape *>(shape)->GetNumVerticesInFace(faceIndex);
}

uint32_t JPH_ConvexHullShape_GetFaceVertices(const JPH_ConvexHullShape *shape,
                                             const uint32_t faceIndex,
                                             const uint32_t maxVertices,
                                             uint32_t *vertices)
{
    return reinterpret_cast<const JPH::ConvexHullShape *>(shape)->GetFaceVertices(faceIndex, maxVertices, vertices);
}

/* MeshShapeSettings */
JPH_MeshShapeSettings *JPH_MeshShapeSettings_Create(const JPH_Triangle *triangles, const uint32_t triangleCount)
{
    JPH::TriangleList jolTriangles;
    jolTriangles.reserve(triangleCount);

    for (uint32_t i = 0; i < triangleCount; ++i)
    {
        jolTriangles.push_back(ToTriangle(triangles[i]));
    }

    JPH::MeshShapeSettings *const settings = new JPH::MeshShapeSettings(jolTriangles);
    settings->AddRef();

    return ToMeshShapeSettings(settings);
}

JPH_MeshShapeSettings *JPH_MeshShapeSettings_Create2(const JPH_Vec3 *vertices,
                                                     const uint32_t verticesCount,
                                                     const JPH_IndexedTriangle *triangles,
                                                     const uint32_t triangleCount)
{
    JPH::VertexList joltVertices;
    JPH::IndexedTriangleList joltTriangles;

    joltVertices.reserve(verticesCount);
    joltTriangles.reserve(triangleCount);

    for (uint32_t i = 0; i < verticesCount; ++i)
    {
        joltVertices.push_back(ToJoltFloat3(vertices[i]));
    }

    for (uint32_t i = 0; i < triangleCount; ++i)
    {
        joltTriangles.push_back(ToIndexedTriangle(triangles[i]));
    }

    JPH::MeshShapeSettings *const settings = new JPH::MeshShapeSettings(joltVertices, joltTriangles);
    settings->AddRef();

    return ToMeshShapeSettings(settings);
}

uint32_t JPH_MeshShapeSettings_GetMaxTrianglesPerLeaf(const JPH_MeshShapeSettings *settings)
{
    return AsMeshShapeSettings(settings)->mMaxTrianglesPerLeaf;
}

void JPH_MeshShapeSettings_SetMaxTrianglesPerLeaf(JPH_MeshShapeSettings *settings, const uint32_t value)
{
    AsMeshShapeSettings(settings)->mMaxTrianglesPerLeaf = value;
}
float JPH_MeshShapeSettings_GetActiveEdgeCosThresholdAngle(const JPH_MeshShapeSettings *settings)
{
    return AsMeshShapeSettings(settings)->mActiveEdgeCosThresholdAngle;
}

void JPH_MeshShapeSettings_SetActiveEdgeCosThresholdAngle(JPH_MeshShapeSettings *settings, const float value)
{
    AsMeshShapeSettings(settings)->mActiveEdgeCosThresholdAngle = value;
}

bool JPH_MeshShapeSettings_GetPerTriangleUserData(const JPH_MeshShapeSettings *settings)
{
    return AsMeshShapeSettings(settings)->mPerTriangleUserData;
}

void JPH_MeshShapeSettings_SetPerTriangleUserData(JPH_MeshShapeSettings *settings, const bool value)
{
    AsMeshShapeSettings(settings)->mPerTriangleUserData = value;
}

JPH_Mesh_Shape_BuildQuality JPH_MeshShapeSettings_GetBuildQuality(const JPH_MeshShapeSettings *settings)
{
    return static_cast<JPH_Mesh_Shape_BuildQuality>(AsMeshShapeSettings(settings)->mBuildQuality);
}

void JPH_MeshShapeSettings_SetBuildQuality(JPH_MeshShapeSettings *settings, JPH_Mesh_Shape_BuildQuality value)
{
    AsMeshShapeSettings(settings)->mBuildQuality = static_cast<JPH::MeshShapeSettings::EBuildQuality>(value);
}

void JPH_MeshShapeSettings_Sanitize(JPH_MeshShapeSettings *settings)
{
    AsMeshShapeSettings(settings)->Sanitize();
}

JPH_MeshShape *JPH_MeshShapeSettings_CreateShape(const JPH_MeshShapeSettings *settings)
{
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = AsMeshShapeSettings(settings)->Create();
    if (!shape_res.IsValid())
    {
        return nullptr;
    }

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_MeshShape *>(reinterpret_cast<const JPH_MeshShape *>(shape));
}

uint32_t JPH_MeshShape_GetTriangleUserData(const JPH_MeshShape *shape, const JPH_SubShapeID id)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(id);
    return AsMeshShape(shape)->GetTriangleUserData(joltSubShapeID);
}

/* HeightFieldShapeSettings */
JPH_HeightFieldShapeSettings *JPH_HeightFieldShapeSettings_Create(const float *samples,
                                                                  const JPH_Vec3 *offset,
                                                                  const JPH_Vec3 *scale,
                                                                  const uint32_t sampleCount)
{
    JPH::HeightFieldShapeSettings *const settings = new JPH::HeightFieldShapeSettings(samples,
                                                                                      ToJolt(offset),
                                                                                      ToJolt(scale),
                                                                                      sampleCount);
    settings->AddRef();

    return ToHeightFieldShapeSettings(settings);
}

JPH_HeightFieldShape *JPH_HeightFieldShapeSettings_CreateShape(JPH_HeightFieldShapeSettings *settings)
{
    const JPH::Result<JPH::Ref<JPH::Shape>> shapeResult = AsHeightFieldShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    const JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_HeightFieldShape *>(reinterpret_cast<const JPH_HeightFieldShape *>(shape));
}

void JPH_HeightFieldShapeSettings_DetermineMinAndMaxSample(const JPH_HeightFieldShapeSettings *settings,
                                                           float *pOutMinValue,
                                                           float *pOutMaxValue,
                                                           float *pOutQuantizationScale)
{
    float outMinValue;
    float outMaxValue;
    float outQuantizationScale;
    AsHeightFieldShapeSettings(settings)->DetermineMinAndMaxSample(outMinValue, outMaxValue, outQuantizationScale);
    if (pOutMinValue != nullptr)
    {
        *pOutMinValue = outMinValue;
    }
    if (pOutMaxValue != nullptr)
    {
        *pOutMaxValue = outMaxValue;
    }
    if (pOutQuantizationScale != nullptr)
    {
        *pOutQuantizationScale = outQuantizationScale;
    }
}

uint32_t JPH_HeightFieldShapeSettings_CalculateBitsPerSampleForError(const JPH_HeightFieldShapeSettings *settings,
                                                                     const float maxError)
{
    return AsHeightFieldShapeSettings(settings)->CalculateBitsPerSampleForError(maxError);
}

uint32_t JPH_HeightFieldShape_GetSampleCount(const JPH_HeightFieldShape *shape)
{
    return AsHeightFieldShape(shape)->GetSampleCount();
}

uint32_t JPH_HeightFieldShape_GetBlockSize(const JPH_HeightFieldShape *shape)
{
    return AsHeightFieldShape(shape)->GetBlockSize();
}

const JPH_PhysicsMaterial *JPH_HeightFieldShape_GetMaterial(const JPH_HeightFieldShape *shape,
                                                            const uint32_t x,
                                                            const uint32_t y)
{
    return FromJolt(AsHeightFieldShape(shape)->GetMaterial(x, y));
}

void JPH_HeightFieldShape_GetPosition(const JPH_HeightFieldShape *shape,
                                      const uint32_t x,
                                      const uint32_t y,
                                      JPH_Vec3 *result)
{
    FromJolt(AsHeightFieldShape(shape)->GetPosition(x, y), result);
}

bool JPH_HeightFieldShape_IsNoCollision(const JPH_HeightFieldShape *shape, const uint32_t x, const uint32_t y)
{
    return AsHeightFieldShape(shape)->IsNoCollision(x, y);
}

bool JPH_HeightFieldShape_ProjectOntoSurface(const JPH_HeightFieldShape *shape,
                                             const JPH_Vec3 *localPosition,
                                             JPH_Vec3 *outSurfacePosition,
                                             JPH_SubShapeID *outSubShapeID)
{
    JPH_ASSERT(outSurfacePosition);
    JPH_ASSERT(outSubShapeID);

    JPH::Vec3 surfacePosition;
    JPH::SubShapeID subShapeID;

    const bool result = AsHeightFieldShape(shape)->ProjectOntoSurface(ToJolt(localPosition),
                                                                      surfacePosition,
                                                                      subShapeID);
    FromJolt(surfacePosition, outSurfacePosition);
    *outSubShapeID = subShapeID.GetValue();
    return result;
}

float JPH_HeightFieldShape_GetMinHeightValue(const JPH_HeightFieldShape *shape)
{
    return AsHeightFieldShape(shape)->GetMinHeightValue();
}

float JPH_HeightFieldShape_GetMaxHeightValue(const JPH_HeightFieldShape *shape)
{
    return AsHeightFieldShape(shape)->GetMaxHeightValue();
}

/* TaperedCapsuleShapeSettings */
JPH_TaperedCapsuleShapeSettings *JPH_TaperedCapsuleShapeSettings_Create(const float halfHeightOfTaperedCylinder,
                                                                        const float topRadius,
                                                                        const float bottomRadius)
{
    JPH::TaperedCapsuleShapeSettings *const settings = new JPH::TaperedCapsuleShapeSettings(halfHeightOfTaperedCylinder,
                                                                                            topRadius,
                                                                                            bottomRadius);
    settings->AddRef();

    return reinterpret_cast<JPH_TaperedCapsuleShapeSettings *>(settings);
}

JPH_TaperedCapsuleShape *JPH_TaperedCapsuleShapeSettings_CreateShape(const JPH_TaperedCapsuleShapeSettings *settings)
{
    const JPH::TaperedCapsuleShapeSettings
            *joltSettings = reinterpret_cast<const JPH::TaperedCapsuleShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_TaperedCapsuleShape *>(reinterpret_cast<const JPH_TaperedCapsuleShape *>(shape));
}

float JPH_TaperedCapsuleShape_GetTopRadius(const JPH_TaperedCapsuleShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCapsuleShape *>(shape)->GetTopRadius();
}

float JPH_TaperedCapsuleShape_GetBottomRadius(const JPH_TaperedCapsuleShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCapsuleShape *>(shape)->GetBottomRadius();
}

float JPH_TaperedCapsuleShape_GetHalfHeight(const JPH_TaperedCapsuleShape *shape)
{
    return reinterpret_cast<const JPH::TaperedCapsuleShape *>(shape)->GetHalfHeight();
}

/* CompoundShape */
void JPH_CompoundShapeSettings_AddShape(JPH_CompoundShapeSettings *settings,
                                        const JPH_Vec3 *position,
                                        const JPH_Quat *rotation,
                                        const JPH_ShapeSettings *shapeSettings,
                                        const uint32_t userData)
{
    AsCompoundShapeSettings(settings)->AddShape(ToJolt(position),
                                                ToJolt(rotation),
                                                AsShapeSettings(shapeSettings),
                                                userData);
}

void JPH_CompoundShapeSettings_AddShape2(JPH_CompoundShapeSettings *settings,
                                         const JPH_Vec3 *position,
                                         const JPH_Quat *rotation,
                                         const JPH_Shape *shape,
                                         const uint32_t userData)
{
    AsCompoundShapeSettings(settings)->AddShape(ToJolt(position), ToJolt(rotation), AsShape(shape), userData);
}

uint32_t JPH_CompoundShape_GetNumSubShapes(const JPH_CompoundShape *shape)
{
    JPH_ASSERT(shape);
    const JPH::CompoundShape *joltShape = reinterpret_cast<const JPH::CompoundShape *>(shape);
    return joltShape->GetNumSubShapes();
}

void JPH_CompoundShape_GetSubShape(const JPH_CompoundShape *shape,
                                   uint32_t index,
                                   const JPH_Shape **subShape,
                                   JPH_Vec3 *positionCOM,
                                   JPH_Quat *rotation,
                                   uint32_t *userData)
{
    JPH_ASSERT(shape);
    const JPH::CompoundShape *joltShape = reinterpret_cast<const JPH::CompoundShape *>(shape);
    const JPH::CompoundShape::SubShape &sub = joltShape->GetSubShape(index);
    if (subShape != nullptr)
    {
        *subShape = reinterpret_cast<const JPH_Shape *>(sub.mShape.GetPtr());
    }
    if (positionCOM != nullptr)
    {
        FromJolt(sub.GetPositionCOM(), positionCOM);
    }
    if (rotation != nullptr)
    {
        FromJolt(sub.GetRotation(), rotation);
    }
    if (userData != nullptr)
    {
        *userData = sub.mUserData;
    }
}

uint32_t JPH_CompoundShape_GetSubShapeIndexFromID(const JPH_CompoundShape *shape,
                                                  JPH_SubShapeID id,
                                                  JPH_SubShapeID *remainder)
{
    JPH_ASSERT(shape);
    const JPH::CompoundShape *joltShape = reinterpret_cast<const JPH::CompoundShape *>(shape);
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(id);
    JPH::SubShapeID joltRemainder = JPH::SubShapeID();
    const uint32_t index = joltShape->GetSubShapeIndexFromID(joltSubShapeID, joltRemainder);
    *remainder = joltRemainder.GetValue();
    return index;
}

/* StaticCompoundShape */
JPH_StaticCompoundShapeSettings *JPH_StaticCompoundShapeSettings_Create()
{
    JPH::StaticCompoundShapeSettings *const settings = new JPH::StaticCompoundShapeSettings();
    settings->AddRef();

    return reinterpret_cast<JPH_StaticCompoundShapeSettings *>(settings);
}

JPH_StaticCompoundShape *JPH_StaticCompoundShape_Create(const JPH_StaticCompoundShapeSettings *settings)
{
    const JPH::StaticCompoundShapeSettings
            *jolt_settings = reinterpret_cast<const JPH::StaticCompoundShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = jolt_settings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_StaticCompoundShape *>(reinterpret_cast<const JPH_StaticCompoundShape *>(shape));
}

/* MutableCompoundShape */
JPH_MutableCompoundShapeSettings *JPH_MutableCompoundShapeSettings_Create()
{
    JPH::MutableCompoundShapeSettings *const settings = new JPH::MutableCompoundShapeSettings();
    settings->AddRef();

    return reinterpret_cast<JPH_MutableCompoundShapeSettings *>(settings);
}

JPH_MutableCompoundShape *JPH_MutableCompoundShape_Create(const JPH_MutableCompoundShapeSettings *settings)
{
    const JPH::MutableCompoundShapeSettings
            *jolt_settings = reinterpret_cast<const JPH::MutableCompoundShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = jolt_settings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_MutableCompoundShape *>(reinterpret_cast<const JPH_MutableCompoundShape *>(shape));
}

uint32_t JPH_MutableCompoundShape_AddShape(JPH_MutableCompoundShape *shape,
                                           const JPH_Vec3 *position,
                                           const JPH_Quat *rotation,
                                           const JPH_Shape *child,
                                           const uint32_t userData,
                                           const uint32_t index)
{
    JPH::MutableCompoundShape *const joltShape = reinterpret_cast<JPH::MutableCompoundShape *>(shape);
    return joltShape->AddShape(ToJolt(position), ToJolt(rotation), AsShape(child), userData, index);
}

void JPH_MutableCompoundShape_RemoveShape(JPH_MutableCompoundShape *shape, const uint32_t index)
{
    reinterpret_cast<JPH::MutableCompoundShape *>(shape)->RemoveShape(index);
}

void JPH_MutableCompoundShape_ModifyShape(JPH_MutableCompoundShape *shape,
                                          const uint32_t index,
                                          const JPH_Vec3 *position,
                                          const JPH_Quat *rotation)
{
    JPH::MutableCompoundShape *const joltShape = reinterpret_cast<JPH::MutableCompoundShape *>(shape);
    joltShape->ModifyShape(index, ToJolt(position), ToJolt(rotation));
}

void JPH_MutableCompoundShape_ModifyShape2(JPH_MutableCompoundShape *shape,
                                           const uint32_t index,
                                           const JPH_Vec3 *position,
                                           const JPH_Quat *rotation,
                                           const JPH_Shape *newShape)
{
    JPH::MutableCompoundShape *const joltShape = reinterpret_cast<JPH::MutableCompoundShape *>(shape);
    const JPH::Shape *const joltNewShape = reinterpret_cast<const JPH::Shape *>(newShape);
    joltShape->ModifyShape(index, ToJolt(position), ToJolt(rotation), joltNewShape);
}

void JPH_MutableCompoundShape_AdjustCenterOfMass(JPH_MutableCompoundShape *shape)
{
    reinterpret_cast<JPH::MutableCompoundShape *>(shape)->AdjustCenterOfMass();
}

/* DecoratedShape */
const JPH_Shape *JPH_DecoratedShape_GetInnerShape(const JPH_DecoratedShape *shape)
{
    const JPH::DecoratedShape *const joltShape = reinterpret_cast<const JPH::DecoratedShape *>(shape);
    return reinterpret_cast<const JPH_Shape *>(joltShape->GetInnerShape());
}

/* RotatedTranslatedShape */
JPH_RotatedTranslatedShapeSettings *JPH_RotatedTranslatedShapeSettings_Create(const JPH_Vec3 *position,
                                                                              const JPH_Quat *rotation,
                                                                              const JPH_ShapeSettings *shapeSettings)
{
    JPH::RotatedTranslatedShapeSettings
            *const settings = new JPH::RotatedTranslatedShapeSettings(ToJolt(position),
                                                                      rotation != nullptr ? ToJolt(rotation)
                                                                                          : JPH::Quat::sIdentity(),
                                                                      AsShapeSettings(shapeSettings));
    settings->AddRef();

    return reinterpret_cast<JPH_RotatedTranslatedShapeSettings *>(settings);
}

JPH_RotatedTranslatedShapeSettings *JPH_RotatedTranslatedShapeSettings_Create2(const JPH_Vec3 *position,
                                                                               const JPH_Quat *rotation,
                                                                               const JPH_Shape *shape)
{
    JPH::RotatedTranslatedShapeSettings
            *const settings = new JPH::RotatedTranslatedShapeSettings(ToJolt(position),
                                                                      rotation != nullptr ? ToJolt(rotation)
                                                                                          : JPH::Quat::sIdentity(),
                                                                      AsShape(shape));
    settings->AddRef();

    return reinterpret_cast<JPH_RotatedTranslatedShapeSettings *>(settings);
}

JPH_RotatedTranslatedShape *JPH_RotatedTranslatedShapeSettings_CreateShape(const JPH_RotatedTranslatedShapeSettings
                                                                                   *settings)
{
    const JPH::RotatedTranslatedShapeSettings
            *jolt_settings = reinterpret_cast<const JPH::RotatedTranslatedShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = jolt_settings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_RotatedTranslatedShape *>(reinterpret_cast<const JPH_RotatedTranslatedShape *>(shape));
}

JPH_RotatedTranslatedShape *JPH_RotatedTranslatedShape_Create(const JPH_Vec3 *position,
                                                              const JPH_Quat *rotation,
                                                              const JPH_Shape *shape)
{
    const JPH::Shape *const jolt_shape = reinterpret_cast<const JPH::Shape *>(shape);

    JPH::RotatedTranslatedShape *const
            rotatedTranslatedShape = new JPH::RotatedTranslatedShape(ToJolt(position),
                                                                     rotation != nullptr ? ToJolt(rotation)
                                                                                         : JPH::Quat::sIdentity(),
                                                                     jolt_shape);
    rotatedTranslatedShape->AddRef();

    return reinterpret_cast<JPH_RotatedTranslatedShape *>(rotatedTranslatedShape);
}

void JPH_RotatedTranslatedShape_GetPosition(const JPH_RotatedTranslatedShape *shape, JPH_Vec3 *position)
{
    JPH_ASSERT(shape);
    const JPH::RotatedTranslatedShape *joltShape = reinterpret_cast<const JPH::RotatedTranslatedShape *>(shape);
    const JPH::Vec3 joltVector = joltShape->GetPosition();
    FromJolt(joltVector, position);
}

void JPH_RotatedTranslatedShape_GetRotation(const JPH_RotatedTranslatedShape *shape, JPH_Quat *rotation)
{
    JPH_ASSERT(shape);
    const JPH::RotatedTranslatedShape *joltShape = reinterpret_cast<const JPH::RotatedTranslatedShape *>(shape);
    const JPH::Quat joltQuat = joltShape->GetRotation();
    FromJolt(joltQuat, rotation);
}

JPH_ScaledShapeSettings *JPH_ScaledShapeSettings_Create(const JPH_ShapeSettings *shapeSettings, const JPH_Vec3 *scale)
{
    JPH::ScaledShapeSettings *const settings = new JPH::ScaledShapeSettings(AsShapeSettings(shapeSettings),
                                                                            ToJolt(scale));
    settings->AddRef();

    return reinterpret_cast<JPH_ScaledShapeSettings *>(settings);
}

JPH_ScaledShapeSettings *JPH_ScaledShapeSettings_Create2(const JPH_Shape *shape, const JPH_Vec3 *scale)
{
    JPH::ScaledShapeSettings *const settings = new JPH::ScaledShapeSettings(AsShape(shape), ToJolt(scale));
    settings->AddRef();

    return reinterpret_cast<JPH_ScaledShapeSettings *>(settings);
}

JPH_ScaledShape *JPH_ScaledShapeSettings_CreateShape(const JPH_ScaledShapeSettings *settings)
{
    const JPH::ScaledShapeSettings *jolt_settings = reinterpret_cast<const JPH::ScaledShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = jolt_settings->Create();
    if (!shape_res.IsValid())
    {
        return nullptr;
    }

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_ScaledShape *>(reinterpret_cast<const JPH_ScaledShape *>(shape));
}

JPH_ScaledShape *JPH_ScaledShape_Create(const JPH_Shape *shape, const JPH_Vec3 *scale)
{
    const JPH::Shape *const jolt_shape = reinterpret_cast<const JPH::Shape *>(shape);

    JPH::ScaledShape *const scaledShape = new JPH::ScaledShape(jolt_shape, ToJolt(scale));
    scaledShape->AddRef();

    return reinterpret_cast<JPH_ScaledShape *>(scaledShape);
}

void JPH_ScaledShape_GetScale(const JPH_ScaledShape *shape, JPH_Vec3 *result)
{
    JPH_ASSERT(shape);
    const JPH::ScaledShape *joltShape = reinterpret_cast<const JPH::ScaledShape *>(shape);
    const JPH::Vec3 joltScale = joltShape->GetScale();
    FromJolt(joltScale, result);
}

/* JPH_OffsetCenterOfMassShape */
JPH_OffsetCenterOfMassShapeSettings *JPH_OffsetCenterOfMassShapeSettings_Create(const JPH_Vec3 *offset,
                                                                                const JPH_ShapeSettings *shapeSettings)
{
    const JPH::ShapeSettings *const joltSettings = reinterpret_cast<const JPH::ShapeSettings *>(shapeSettings);

    JPH::OffsetCenterOfMassShapeSettings *const settings = new JPH::OffsetCenterOfMassShapeSettings(ToJolt(offset),
                                                                                                    joltSettings);
    settings->AddRef();

    return reinterpret_cast<JPH_OffsetCenterOfMassShapeSettings *>(settings);
}

JPH_OffsetCenterOfMassShapeSettings *JPH_OffsetCenterOfMassShapeSettings_Create2(const JPH_Vec3 *offset,
                                                                                 const JPH_Shape *shape)
{
    const JPH::Shape *const joltShape = reinterpret_cast<const JPH::Shape *>(shape);

    JPH::OffsetCenterOfMassShapeSettings
            *const rotatedTranslatedShape = new JPH::OffsetCenterOfMassShapeSettings(ToJolt(offset), joltShape);
    rotatedTranslatedShape->AddRef();

    return reinterpret_cast<JPH_OffsetCenterOfMassShapeSettings *>(rotatedTranslatedShape);
}

JPH_OffsetCenterOfMassShape *JPH_OffsetCenterOfMassShapeSettings_CreateShape(const JPH_OffsetCenterOfMassShapeSettings
                                                                                     *settings)
{
    const JPH::OffsetCenterOfMassShapeSettings
            *joltSettings = reinterpret_cast<const JPH::OffsetCenterOfMassShapeSettings *>(settings);
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = joltSettings->Create();

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return const_cast<JPH_OffsetCenterOfMassShape *>(reinterpret_cast<const JPH_OffsetCenterOfMassShape *>(shape));
}

JPH_OffsetCenterOfMassShape *JPH_OffsetCenterOfMassShape_Create(const JPH_Vec3 *offset, const JPH_Shape *shape)
{
    const JPH::Shape *const joltShape = reinterpret_cast<const JPH::Shape *>(shape);

    JPH::OffsetCenterOfMassShape *const offsetCenterOfMassShape = new JPH::OffsetCenterOfMassShape(joltShape,
                                                                                                   ToJolt(offset));
    offsetCenterOfMassShape->AddRef();

    return reinterpret_cast<JPH_OffsetCenterOfMassShape *>(offsetCenterOfMassShape);
}

void JPH_OffsetCenterOfMassShape_GetOffset(const JPH_OffsetCenterOfMassShape *shape, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<const JPH::OffsetCenterOfMassShape *>(shape)->GetOffset(), result);
}

/* EmptyShape */
JPH_EmptyShapeSettings *JPH_EmptyShapeSettings_Create(const JPH_Vec3 *centerOfMass)
{
    JPH::EmptyShapeSettings *const settings = new JPH::EmptyShapeSettings(ToJolt(centerOfMass));
    settings->AddRef();

    return ToEmptyShapeSettings(settings);
}

JPH_EmptyShape *JPH_EmptyShapeSettings_CreateShape(const JPH_EmptyShapeSettings *settings)
{
    const JPH::Result<JPH::Ref<JPH::Shape>> shape_res = AsEmptyShapeSettings(settings)->Create();
    if (!shape_res.IsValid())
    {
        return nullptr;
    }

    const JPH::Shape *shape = shape_res.Get().GetPtr();
    shape->AddRef();

    return ToEmptyShape(static_cast<JPH::EmptyShape *>(const_cast<JPH::Shape *>(shape)));
}

/* JPH_BodyCreationSettings */
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
    auto *const bodyCreationSettings = new JPH::BodyCreationSettings(AsShapeSettings(shapeSettings),
                                                                     ToJolt(position),
                                                                     rotation != nullptr ? ToJolt(rotation)
                                                                                         : JPH::Quat::sIdentity(),
                                                                     static_cast<JPH::EMotionType>(motionType),
                                                                     objectLayer);
    return ToBodyCreationSettings(bodyCreationSettings);
}

JPH_BodyCreationSettings *JPH_BodyCreationSettings_Create3(const JPH_Shape *shape,
                                                           const JPH_RVec3 *position,
                                                           const JPH_Quat *rotation,
                                                           JPH_MotionType motionType,
                                                           const JPH_ObjectLayer objectLayer)
{
    auto *const bodyCreationSettings = new JPH::BodyCreationSettings(AsShape(shape),
                                                                     ToJolt(position),
                                                                     rotation != nullptr ? ToJolt(rotation)
                                                                                         : JPH::Quat::sIdentity(),
                                                                     static_cast<JPH::EMotionType>(motionType),
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

void JPH_BodyCreationSettings_GetLinearVelocity(JPH_BodyCreationSettings *settings, JPH_Vec3 *velocity)
{
    FromJolt(AsBodyCreationSettings(settings)->mLinearVelocity, velocity);
}

void JPH_BodyCreationSettings_SetLinearVelocity(JPH_BodyCreationSettings *settings, const JPH_Vec3 *velocity)
{
    AsBodyCreationSettings(settings)->mLinearVelocity = ToJolt(velocity);
}

void JPH_BodyCreationSettings_GetAngularVelocity(JPH_BodyCreationSettings *settings, JPH_Vec3 *velocity)
{
    FromJolt(AsBodyCreationSettings(settings)->mAngularVelocity, velocity);
}

void JPH_BodyCreationSettings_SetAngularVelocity(JPH_BodyCreationSettings *settings, const JPH_Vec3 *velocity)
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

void JPH_BodyCreationSettings_GetCollisionGroup(const JPH_BodyCreationSettings *settings, JPH_CollisionGroup *result)
{
    FromJolt(AsBodyCreationSettings(settings)->mCollisionGroup, result);
}

void JPH_BodyCreationSettings_SetCollisionGroup(JPH_BodyCreationSettings *settings, const JPH_CollisionGroup *value)
{
    AsBodyCreationSettings(settings)->mCollisionGroup = ToJolt(value);
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

/* JPH_SoftBodyCreationSettings */
JPH_SoftBodyCreationSettings *JPH_SoftBodyCreationSettings_Create()
{
    JPH::SoftBodyCreationSettings *const bodyCreationSettings = new JPH::SoftBodyCreationSettings();
    return ToSoftBodyCreationSettings(bodyCreationSettings);
}

void JPH_SoftBodyCreationSettings_Destroy(JPH_SoftBodyCreationSettings *settings)
{
    if (settings != nullptr)
    {
        delete AsSoftBodyCreationSettings(settings);
    }
}

/* JPH_ConstraintSettings */
void JPH_ConstraintSettings_Init(const JPH::ConstraintSettings &joltSettings, JPH_ConstraintSettings *settings)
{
    // Copy defaults from jolt
    settings->enabled = joltSettings.mEnabled;
    settings->constraintPriority = joltSettings.mConstraintPriority;
    settings->numVelocityStepsOverride = joltSettings.mNumVelocityStepsOverride;
    settings->numPositionStepsOverride = joltSettings.mNumPositionStepsOverride;
    settings->drawConstraintSize = joltSettings.mDrawConstraintSize;
    settings->userData = joltSettings.mUserData;
}

void JPH_ConstraintSettings_ToJolt(JPH::ConstraintSettings *joltSettings, const JPH_ConstraintSettings *settings)
{
    // Copy settings to jolt
    joltSettings->mEnabled = settings->enabled;
    joltSettings->mConstraintPriority = settings->constraintPriority;
    joltSettings->mNumVelocityStepsOverride = settings->numVelocityStepsOverride;
    joltSettings->mNumPositionStepsOverride = settings->numPositionStepsOverride;
    joltSettings->mDrawConstraintSize = settings->drawConstraintSize;
    joltSettings->mUserData = settings->userData;
}

/* JPH_Constraint */
void JPH_Constraint_Destroy(JPH_Constraint *constraint)
{
    if (constraint != nullptr)
    {
        AsConstraint(constraint)->Release();
    }
}

JPH_ConstraintType JPH_Constraint_GetType(const JPH_Constraint *constraint)
{
    return static_cast<JPH_ConstraintType>(AsConstraint(constraint)->GetType());
}

JPH_ConstraintSubType JPH_Constraint_GetSubType(const JPH_Constraint *constraint)
{
    return static_cast<JPH_ConstraintSubType>(AsConstraint(constraint)->GetSubType());
}

uint32_t JPH_Constraint_GetConstraintPriority(const JPH_Constraint *constraint)
{
    return AsConstraint(constraint)->GetConstraintPriority();
}

void JPH_Constraint_SetConstraintPriority(JPH_Constraint *constraint, const uint32_t priority)
{
    AsConstraint(constraint)->SetConstraintPriority(priority);
}

uint32_t JPH_Constraint_GetNumVelocityStepsOverride(const JPH_Constraint *constraint)
{
    return AsConstraint(constraint)->GetNumVelocityStepsOverride();
}

void JPH_Constraint_SetNumVelocityStepsOverride(JPH_Constraint *constraint, const uint32_t value)
{
    AsConstraint(constraint)->SetNumVelocityStepsOverride(value);
}

uint32_t JPH_Constraint_GetNumPositionStepsOverride(const JPH_Constraint *constraint)
{
    return AsConstraint(constraint)->GetNumPositionStepsOverride();
}

void JPH_Constraint_SetNumPositionStepsOverride(JPH_Constraint *constraint, const uint32_t value)
{
    AsConstraint(constraint)->SetNumPositionStepsOverride(value);
}

bool JPH_Constraint_GetEnabled(const JPH_Constraint *constraint)
{
    return AsConstraint(constraint)->GetEnabled();
}

void JPH_Constraint_SetEnabled(JPH_Constraint *constraint, const bool enabled)
{
    AsConstraint(constraint)->SetEnabled(enabled);
}

uint64_t JPH_Constraint_GetUserData(const JPH_Constraint *constraint)
{
    return AsConstraint(constraint)->GetUserData();
}

void JPH_Constraint_SetUserData(JPH_Constraint *constraint, const uint64_t userData)
{
    AsConstraint(constraint)->SetUserData(userData);
}

void JPH_Constraint_NotifyShapeChanged(JPH_Constraint *constraint, const JPH_BodyID bodyID, const JPH_Vec3 *deltaCOM)
{
    AsConstraint(constraint)->NotifyShapeChanged(JPH::BodyID(bodyID), ToJolt(deltaCOM));
}

void JPH_Constraint_ResetWarmStart(JPH_Constraint *constraint)
{
    AsConstraint(constraint)->ResetWarmStart();
}

bool JPH_Constraint_IsActive(const JPH_Constraint *constraint)
{
    return AsConstraint(constraint)->IsActive();
}

void JPH_Constraint_SetupVelocityConstraint(JPH_Constraint *constraint, const float deltaTime)
{
    AsConstraint(constraint)->SetupVelocityConstraint(deltaTime);
}

void JPH_Constraint_WarmStartVelocityConstraint(JPH_Constraint *constraint, const float warmStartImpulseRatio)
{
    AsConstraint(constraint)->WarmStartVelocityConstraint(warmStartImpulseRatio);
}

bool JPH_Constraint_SolveVelocityConstraint(JPH_Constraint *constraint, const float deltaTime)
{
    return AsConstraint(constraint)->SolveVelocityConstraint(deltaTime);
}

bool JPH_Constraint_SolvePositionConstraint(JPH_Constraint *constraint, const float deltaTime, const float baumgarte)
{
    return AsConstraint(constraint)->SolvePositionConstraint(deltaTime, baumgarte);
}

/* JPH_TwoBodyConstraint */
JPH_Body *JPH_TwoBodyConstraint_GetBody1(const JPH_TwoBodyConstraint *constraint)
{
    const JPH::Body *joltBody = AsTwoBodyConstraint(constraint)->GetBody1();
    return const_cast<JPH_Body *>(reinterpret_cast<const JPH_Body *>(joltBody));
}

JPH_Body *JPH_TwoBodyConstraint_GetBody2(const JPH_TwoBodyConstraint *constraint)
{
    const JPH::Body *joltBody = AsTwoBodyConstraint(constraint)->GetBody2();
    return const_cast<JPH_Body *>(reinterpret_cast<const JPH_Body *>(joltBody));
}

void JPH_TwoBodyConstraint_GetConstraintToBody1Matrix(const JPH_TwoBodyConstraint *constraint, JPH_Matrix4x4 *result)
{
    const JPH::Mat44 joltMatrix = AsTwoBodyConstraint(constraint)->GetConstraintToBody1Matrix();
    FromJolt(joltMatrix, result);
}

void JPH_TwoBodyConstraint_GetConstraintToBody2Matrix(const JPH_TwoBodyConstraint *constraint, JPH_Matrix4x4 *result)
{
    const JPH::Mat44 joltMatrix = AsTwoBodyConstraint(constraint)->GetConstraintToBody2Matrix();
    FromJolt(joltMatrix, result);
}

/* JPH_FixedConstraintSettings */
void JPH_FixedConstraintSettings_FromJolt(JPH_FixedConstraintSettings *settings,
                                          const JPH::FixedConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    settings->autoDetectPoint = joltSettings.mAutoDetectPoint;
    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mAxisX1, &settings->axisX1);
    FromJolt(joltSettings.mAxisY1, &settings->axisY1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
    FromJolt(joltSettings.mAxisX2, &settings->axisX2);
    FromJolt(joltSettings.mAxisY2, &settings->axisY2);
}

void JPH_FixedConstraintSettings_ToJolt(JPH::FixedConstraintSettings *joltSettings,
                                        const JPH_FixedConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mAutoDetectPoint = settings->autoDetectPoint;
    joltSettings->mPoint1 = ToJolt(settings->point1);
    joltSettings->mAxisX1 = ToJolt(settings->axisX1);
    joltSettings->mAxisY1 = ToJolt(settings->axisY1);
    joltSettings->mPoint2 = ToJolt(settings->point2);
    joltSettings->mAxisX2 = ToJolt(settings->axisX2);
    joltSettings->mAxisY2 = ToJolt(settings->axisY2);
}

void JPH_FixedConstraintSettings_Init(JPH_FixedConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::FixedConstraintSettings joltSettings;
    JPH_FixedConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_FixedConstraint *JPH_FixedConstraint_Create(const JPH_FixedConstraintSettings *settings,
                                                JPH_Body *body1,
                                                JPH_Body *body2)
{
    JPH::FixedConstraintSettings joltSettings;
    JPH_FixedConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::FixedConstraint *constraint = static_cast<JPH::FixedConstraint *>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToFixedConstraint(constraint);
}

/* JPH_FixedConstraint */
void JPH_FixedConstraint_GetSettings(const JPH_FixedConstraint *constraint, JPH_FixedConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    const JPH::Ref<JPH::FixedConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::FixedConstraintSettings>(AsFixedConstraint(constraint)->GetConstraintSettings());
    JPH_FixedConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_FixedConstraint_GetTotalLambdaPosition(const JPH_FixedConstraint *constraint, JPH_Vec3 *result)
{
    const JPH::Vec3 lambda = AsFixedConstraint(constraint)->GetTotalLambdaPosition();
    FromJolt(lambda, result);
}

void JPH_FixedConstraint_GetTotalLambdaRotation(const JPH_FixedConstraint *constraint, JPH_Vec3 *result)
{
    const JPH::Vec3 lambda = AsFixedConstraint(constraint)->GetTotalLambdaRotation();
    FromJolt(lambda, result);
}

/* JPH_DistanceConstraintSettings */
void JPH_DistanceConstraintSettings_FromJolt(JPH_DistanceConstraintSettings *settings,
                                             const JPH::DistanceConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
    settings->minDistance = joltSettings.mMinDistance;
    settings->maxDistance = joltSettings.mMaxDistance;
    FromJolt(joltSettings.mLimitsSpringSettings, &settings->limitsSpringSettings);
}

void JPH_DistanceConstraintSettings_ToJolt(JPH::DistanceConstraintSettings *joltSettings,
                                           const JPH_DistanceConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mPoint1 = ToJolt(settings->point1);
    joltSettings->mPoint2 = ToJolt(settings->point2);
    joltSettings->mMinDistance = settings->minDistance;
    joltSettings->mMaxDistance = settings->maxDistance;
    joltSettings->mLimitsSpringSettings = ToJolt(&settings->limitsSpringSettings);
}

void JPH_DistanceConstraintSettings_Init(JPH_DistanceConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::DistanceConstraintSettings joltSettings;
    JPH_DistanceConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_DistanceConstraint *JPH_DistanceConstraint_Create(const JPH_DistanceConstraintSettings *settings,
                                                      JPH_Body *body1,
                                                      JPH_Body *body2)
{
    JPH::DistanceConstraintSettings joltSettings;
    JPH_DistanceConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::DistanceConstraint *constraint = static_cast<JPH::DistanceConstraint *>(joltSettings.Create(*joltBody1,
                                                                                                     *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToDistanceConstraint(constraint);
}

void JPH_DistanceConstraint_GetSettings(const JPH_DistanceConstraint *constraint,
                                        JPH_DistanceConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    const JPH::Ref<JPH::DistanceConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::DistanceConstraintSettings>(AsDistanceConstraint(constraint)->GetConstraintSettings());
    JPH_DistanceConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_DistanceConstraint_SetDistance(JPH_DistanceConstraint *constraint,
                                        const float minDistance,
                                        const float maxDistance)
{
    reinterpret_cast<JPH::DistanceConstraint *>(constraint)->SetDistance(minDistance, maxDistance);
}

float JPH_DistanceConstraint_GetMinDistance(JPH_DistanceConstraint *constraint)
{
    return reinterpret_cast<JPH::DistanceConstraint *>(constraint)->GetMinDistance();
}

float JPH_DistanceConstraint_GetMaxDistance(JPH_DistanceConstraint *constraint)
{
    return reinterpret_cast<JPH::DistanceConstraint *>(constraint)->GetMaxDistance();
}

void JPH_DistanceConstraint_GetLimitsSpringSettings(JPH_DistanceConstraint *constraint, JPH_SpringSettings *result)
{
    JPH::DistanceConstraint *const joltConstraint = reinterpret_cast<JPH::DistanceConstraint *>(constraint);
    FromJolt(joltConstraint->GetLimitsSpringSettings(), result);
}

void JPH_DistanceConstraint_SetLimitsSpringSettings(JPH_DistanceConstraint *constraint,
                                                    const JPH_SpringSettings *settings)
{
    JPH::DistanceConstraint *const joltConstraint = reinterpret_cast<JPH::DistanceConstraint *>(constraint);
    joltConstraint->SetLimitsSpringSettings(ToJolt(settings));
}

float JPH_DistanceConstraint_GetTotalLambdaPosition(const JPH_DistanceConstraint *constraint)
{
    JPH_ASSERT(constraint);
    const JPH::DistanceConstraint *joltConstraint = reinterpret_cast<const JPH::DistanceConstraint *>(constraint);
    return joltConstraint->GetTotalLambdaPosition();
}

/* JPH_PointConstraintSettings */
void JPH_PointConstraintSettings_FromJolt(JPH_PointConstraintSettings *settings,
                                          const JPH::PointConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
}

void JPH_PointConstraintSettings_ToJolt(JPH::PointConstraintSettings *joltSettings,
                                        const JPH_PointConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mPoint1 = ToJolt(settings->point1);
    joltSettings->mPoint2 = ToJolt(settings->point2);
}

void JPH_PointConstraintSettings_Init(JPH_PointConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::PointConstraintSettings joltSettings;
    JPH_PointConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_PointConstraint *JPH_PointConstraint_Create(const JPH_PointConstraintSettings *settings,
                                                JPH_Body *body1,
                                                JPH_Body *body2)
{
    JPH::PointConstraintSettings joltSettings;
    JPH_PointConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::PointConstraint *constraint = static_cast<JPH::PointConstraint *>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToPointConstraint(constraint);
}

void JPH_PointConstraint_GetSettings(const JPH_PointConstraint *constraint, JPH_PointConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    const JPH::Ref<JPH::PointConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::PointConstraintSettings>(AsPointConstraint(constraint)->GetConstraintSettings());
    JPH_PointConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_PointConstraint_SetPoint1(JPH_PointConstraint *constraint, JPH_ConstraintSpace space, const JPH_RVec3 *value)
{
    AsPointConstraint(constraint)->SetPoint1(static_cast<JPH::EConstraintSpace>(space), ToJolt(value));
}

void JPH_PointConstraint_SetPoint2(JPH_PointConstraint *constraint, JPH_ConstraintSpace space, const JPH_RVec3 *value)
{
    AsPointConstraint(constraint)->SetPoint2(static_cast<JPH::EConstraintSpace>(space), ToJolt(value));
}

void JPH_PointConstraint_GetLocalSpacePoint1(const JPH_PointConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsPointConstraint(constraint)->GetLocalSpacePoint1(), result);
}

void JPH_PointConstraint_GetLocalSpacePoint2(const JPH_PointConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsPointConstraint(constraint)->GetLocalSpacePoint2(), result);
}

void JPH_PointConstraint_GetTotalLambdaPosition(const JPH_PointConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsPointConstraint(constraint)->GetTotalLambdaPosition(), result);
}

/* JPH_HingeConstraintSettings */
void JPH_HingeConstraintSettings_FromJolt(JPH_HingeConstraintSettings *settings,
                                          const JPH::HingeConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mHingeAxis1, &settings->hingeAxis1);
    FromJolt(joltSettings.mNormalAxis1, &settings->normalAxis1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
    FromJolt(joltSettings.mHingeAxis2, &settings->hingeAxis2);
    FromJolt(joltSettings.mNormalAxis2, &settings->normalAxis2);
    settings->limitsMin = joltSettings.mLimitsMin;
    settings->limitsMax = joltSettings.mLimitsMax;
    FromJolt(joltSettings.mLimitsSpringSettings, &settings->limitsSpringSettings);
    settings->maxFrictionTorque = joltSettings.mMaxFrictionTorque;
    FromJolt(joltSettings.mMotorSettings, &settings->motorSettings);
}

void JPH_HingeConstraintSettings_ToJolt(JPH::HingeConstraintSettings *joltSettings,
                                        const JPH_HingeConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mPoint1 = ToJolt(settings->point1);
    joltSettings->mHingeAxis1 = ToJolt(settings->hingeAxis1);
    joltSettings->mNormalAxis1 = ToJolt(settings->normalAxis1);
    joltSettings->mPoint2 = ToJolt(settings->point2);
    joltSettings->mHingeAxis2 = ToJolt(settings->hingeAxis2);
    joltSettings->mNormalAxis2 = ToJolt(settings->normalAxis2);
    joltSettings->mLimitsMin = settings->limitsMin;
    joltSettings->mLimitsMax = settings->limitsMax;
    joltSettings->mLimitsSpringSettings = ToJolt(&settings->limitsSpringSettings);
    joltSettings->mMaxFrictionTorque = settings->maxFrictionTorque;
    joltSettings->mMotorSettings = ToJolt(&settings->motorSettings);
}

void JPH_HingeConstraintSettings_Init(JPH_HingeConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::HingeConstraintSettings joltSettings;
    JPH_HingeConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_HingeConstraint *JPH_HingeConstraint_Create(const JPH_HingeConstraintSettings *settings,
                                                JPH_Body *body1,
                                                JPH_Body *body2)
{
    JPH::HingeConstraintSettings joltSettings;
    JPH_HingeConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::HingeConstraint *constraint = static_cast<JPH::HingeConstraint *>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToHingeConstraint(constraint);
}

void JPH_HingeConstraint_GetSettings(JPH_HingeConstraint *constraint, JPH_HingeConstraintSettings *settings)
{
    const JPH::Ref<JPH::HingeConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::HingeConstraintSettings>(AsHingeConstraint(constraint)->GetConstraintSettings());
    JPH_HingeConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_HingeConstraint_GetLocalSpacePoint1(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpacePoint1(), result);
}

void JPH_HingeConstraint_GetLocalSpacePoint2(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpacePoint2(), result);
}

void JPH_HingeConstraint_GetLocalSpaceHingeAxis1(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceHingeAxis1(), result);
}

void JPH_HingeConstraint_GetLocalSpaceHingeAxis2(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceHingeAxis2(), result);
}

void JPH_HingeConstraint_GetLocalSpaceNormalAxis1(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceNormalAxis1(), result);
}

void JPH_HingeConstraint_GetLocalSpaceNormalAxis2(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceNormalAxis2(), result);
}

float JPH_HingeConstraint_GetCurrentAngle(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetCurrentAngle();
}

void JPH_HingeConstraint_SetMaxFrictionTorque(JPH_HingeConstraint *constraint, const float frictionTorque)
{
    AsHingeConstraint(constraint)->SetMaxFrictionTorque(frictionTorque);
}

float JPH_HingeConstraint_GetMaxFrictionTorque(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetMaxFrictionTorque();
}

void JPH_HingeConstraint_SetMotorSettings(JPH_HingeConstraint *constraint, const JPH_MotorSettings *settings)
{
    JPH::MotorSettings &joltSettings = AsHingeConstraint(constraint)->GetMotorSettings();
    joltSettings = ToJolt(settings);
}

void JPH_HingeConstraint_GetMotorSettings(JPH_HingeConstraint *constraint, JPH_MotorSettings *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetMotorSettings(), result);
}

void JPH_HingeConstraint_SetMotorState(JPH_HingeConstraint *constraint, JPH_MotorState state)
{
    AsHingeConstraint(constraint)->SetMotorState(static_cast<JPH::EMotorState>(state));
}

JPH_MotorState JPH_HingeConstraint_GetMotorState(JPH_HingeConstraint *constraint)
{
    return static_cast<JPH_MotorState>(AsHingeConstraint(constraint)->GetMotorState());
}

void JPH_HingeConstraint_SetTargetAngularVelocity(JPH_HingeConstraint *constraint, const float angularVelocity)
{
    AsHingeConstraint(constraint)->SetTargetAngularVelocity(angularVelocity);
}

float JPH_HingeConstraint_GetTargetAngularVelocity(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetTargetAngularVelocity();
}

void JPH_HingeConstraint_SetTargetAngle(JPH_HingeConstraint *constraint, const float angle)
{
    AsHingeConstraint(constraint)->SetTargetAngle(angle);
}

float JPH_HingeConstraint_GetTargetAngle(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetTargetAngle();
}

void JPH_HingeConstraint_SetLimits(JPH_HingeConstraint *constraint, const float inLimitsMin, const float inLimitsMax)
{
    AsHingeConstraint(constraint)->SetLimits(inLimitsMin, inLimitsMax);
}

float JPH_HingeConstraint_GetLimitsMin(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetLimitsMin();
}

float JPH_HingeConstraint_GetLimitsMax(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetLimitsMax();
}

bool JPH_HingeConstraint_HasLimits(JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->HasLimits();
}

void JPH_HingeConstraint_GetLimitsSpringSettings(JPH_HingeConstraint *constraint, JPH_SpringSettings *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLimitsSpringSettings(), result);
}

void JPH_HingeConstraint_SetLimitsSpringSettings(JPH_HingeConstraint *constraint, const JPH_SpringSettings *settings)
{
    AsHingeConstraint(constraint)->SetLimitsSpringSettings(ToJolt(settings));
}

void JPH_HingeConstraint_GetTotalLambdaPosition(const JPH_HingeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetTotalLambdaPosition(), result);
}

void JPH_HingeConstraint_GetTotalLambdaRotation(const JPH_HingeConstraint *constraint, float rotation[2])
{
    JPH::Vector<2> lambda = AsHingeConstraint(constraint)->GetTotalLambdaRotation();
    rotation[0] = lambda[0];
    rotation[1] = lambda[1];
}

float JPH_HingeConstraint_GetTotalLambdaRotationLimits(const JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetTotalLambdaRotationLimits();
}

float JPH_HingeConstraint_GetTotalLambdaMotor(const JPH_HingeConstraint *constraint)
{
    return AsHingeConstraint(constraint)->GetTotalLambdaMotor();
}

/* JPH_SliderConstraintSettings */
void JPH_SliderConstraintSettings_FromJolt(JPH_SliderConstraintSettings *settings,
                                           const JPH::SliderConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    settings->autoDetectPoint = joltSettings.mAutoDetectPoint;

    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mSliderAxis1, &settings->sliderAxis1);
    FromJolt(joltSettings.mNormalAxis1, &settings->normalAxis1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
    FromJolt(joltSettings.mSliderAxis2, &settings->sliderAxis2);
    FromJolt(joltSettings.mNormalAxis2, &settings->normalAxis2);
    settings->limitsMin = joltSettings.mLimitsMin;
    settings->limitsMax = joltSettings.mLimitsMax;
    FromJolt(joltSettings.mLimitsSpringSettings, &settings->limitsSpringSettings);
    settings->maxFrictionForce = joltSettings.mMaxFrictionForce;
    FromJolt(joltSettings.mMotorSettings, &settings->motorSettings);
}

void JPH_SliderConstraintSettings_ToJolt(JPH::SliderConstraintSettings *joltSettings,
                                         const JPH_SliderConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mAutoDetectPoint = settings->autoDetectPoint;
    joltSettings->mPoint1 = ToJolt(settings->point1);
    joltSettings->mSliderAxis1 = ToJolt(settings->sliderAxis1);
    joltSettings->mNormalAxis1 = ToJolt(settings->normalAxis1);
    joltSettings->mPoint2 = ToJolt(settings->point2);
    joltSettings->mSliderAxis2 = ToJolt(settings->sliderAxis2);
    joltSettings->mNormalAxis2 = ToJolt(settings->normalAxis2);
    joltSettings->mLimitsMin = settings->limitsMin;
    joltSettings->mLimitsMax = settings->limitsMax;
    joltSettings->mLimitsSpringSettings = ToJolt(&settings->limitsSpringSettings);
    joltSettings->mMaxFrictionForce = settings->maxFrictionForce;
    joltSettings->mMotorSettings = ToJolt(&settings->motorSettings);
}

void JPH_SliderConstraintSettings_Init(JPH_SliderConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::SliderConstraintSettings joltSettings;
    JPH_SliderConstraintSettings_FromJolt(settings, joltSettings);
}

void JPH_SliderConstraintSettings_SetSliderAxis(JPH_SliderConstraintSettings *settings, const JPH_Vec3 *axis)
{
    JPH_ASSERT(settings);

    const JPH::Vec3 joltAxis = ToJolt(axis);
    FromJolt(joltAxis, &settings->sliderAxis1);
    FromJolt(joltAxis, &settings->sliderAxis2);

    FromJolt(joltAxis.GetNormalizedPerpendicular(), &settings->normalAxis1);
    FromJolt(joltAxis.GetNormalizedPerpendicular(), &settings->normalAxis2);
}

JPH_SliderConstraint *JPH_SliderConstraint_Create(const JPH_SliderConstraintSettings *settings,
                                                  JPH_Body *body1,
                                                  JPH_Body *body2)
{
    JPH::SliderConstraintSettings joltSettings;
    JPH_SliderConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::SliderConstraint *constraint = static_cast<JPH::SliderConstraint *>(joltSettings.Create(*joltBody1,
                                                                                                 *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToSliderConstraint(constraint);
}

void JPH_SliderConstraint_GetSettings(JPH_SliderConstraint *constraint, JPH_SliderConstraintSettings *settings)
{
    const JPH::Ref<JPH::SliderConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::SliderConstraintSettings>(AsSliderConstraint(constraint)->GetConstraintSettings());
    JPH_SliderConstraintSettings_FromJolt(settings, *joltSettings);
}

float JPH_SliderConstraint_GetCurrentPosition(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetCurrentPosition();
}

void JPH_SliderConstraint_SetMaxFrictionForce(JPH_SliderConstraint *constraint, const float frictionForce)
{
    AsSliderConstraint(constraint)->SetMaxFrictionForce(frictionForce);
}

float JPH_SliderConstraint_GetMaxFrictionForce(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetMaxFrictionForce();
}

void JPH_SliderConstraint_SetMotorSettings(JPH_SliderConstraint *constraint, const JPH_MotorSettings *settings)
{
    JPH::MotorSettings &joltSettings = AsSliderConstraint(constraint)->GetMotorSettings();
    joltSettings = ToJolt(settings);
}

void JPH_SliderConstraint_GetMotorSettings(const JPH_SliderConstraint *constraint, JPH_MotorSettings *result)
{
    FromJolt(AsSliderConstraint(constraint)->GetMotorSettings(), result);
}

void JPH_SliderConstraint_SetMotorState(JPH_SliderConstraint *constraint, JPH_MotorState state)
{
    AsSliderConstraint(constraint)->SetMotorState(static_cast<JPH::EMotorState>(state));
}

JPH_MotorState JPH_SliderConstraint_GetMotorState(JPH_SliderConstraint *constraint)
{
    return static_cast<JPH_MotorState>(AsSliderConstraint(constraint)->GetMotorState());
}

void JPH_SliderConstraint_SetTargetVelocity(JPH_SliderConstraint *constraint, const float velocity)
{
    AsSliderConstraint(constraint)->SetTargetVelocity(velocity);
}

float JPH_SliderConstraint_GetTargetVelocity(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetTargetVelocity();
}

void JPH_SliderConstraint_SetTargetPosition(JPH_SliderConstraint *constraint, const float position)
{
    AsSliderConstraint(constraint)->SetTargetPosition(position);
}

float JPH_SliderConstraint_GetTargetPosition(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetTargetPosition();
}

void JPH_SliderConstraint_SetLimits(JPH_SliderConstraint *constraint, const float inLimitsMin, const float inLimitsMax)
{
    AsSliderConstraint(constraint)->SetLimits(inLimitsMin, inLimitsMax);
}

float JPH_SliderConstraint_GetLimitsMin(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetLimitsMin();
}

float JPH_SliderConstraint_GetLimitsMax(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetLimitsMax();
}

bool JPH_SliderConstraint_HasLimits(JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->HasLimits();
}

void JPH_SliderConstraint_GetLimitsSpringSettings(JPH_SliderConstraint *constraint, JPH_SpringSettings *result)
{
    FromJolt(AsSliderConstraint(constraint)->GetLimitsSpringSettings(), result);
}

void JPH_SliderConstraint_SetLimitsSpringSettings(JPH_SliderConstraint *constraint, const JPH_SpringSettings *settings)
{
    AsSliderConstraint(constraint)->SetLimitsSpringSettings(ToJolt(settings));
}

void JPH_SliderConstraint_GetTotalLambdaPosition(const JPH_SliderConstraint *constraint, float position[2])
{
    JPH::Vector<2> lambda = AsSliderConstraint(constraint)->GetTotalLambdaPosition();
    position[0] = lambda[0];
    position[1] = lambda[1];
}

float JPH_SliderConstraint_GetTotalLambdaPositionLimits(const JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetTotalLambdaPositionLimits();
}

void JPH_SliderConstraint_GetTotalLambdaRotation(const JPH_SliderConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSliderConstraint(constraint)->GetTotalLambdaRotation(), result);
}

float JPH_SliderConstraint_GetTotalLambdaMotor(const JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetTotalLambdaMotor();
}

/* JPH_ConeConstraintSettings */
void JPH_ConeConstraintSettings_FromJolt(JPH_ConeConstraintSettings *settings,
                                         const JPH::ConeConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mTwistAxis1, &settings->twistAxis1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
    FromJolt(joltSettings.mTwistAxis2, &settings->twistAxis2);
    settings->halfConeAngle = joltSettings.mHalfConeAngle;
}

void JPH_ConeConstraintSettings_ToJolt(JPH::ConeConstraintSettings *joltSettings,
                                       const JPH_ConeConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mPoint1 = ToJolt(settings->point1);
    joltSettings->mTwistAxis1 = ToJolt(settings->twistAxis1);
    joltSettings->mPoint2 = ToJolt(settings->point2);
    joltSettings->mTwistAxis2 = ToJolt(settings->twistAxis2);
    joltSettings->mHalfConeAngle = settings->halfConeAngle;
}

void JPH_ConeConstraintSettings_Init(JPH_ConeConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::ConeConstraintSettings joltSettings;
    JPH_ConeConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_ConeConstraint *JPH_ConeConstraint_Create(const JPH_ConeConstraintSettings *settings,
                                              JPH_Body *body1,
                                              JPH_Body *body2)
{
    JPH::ConeConstraintSettings joltSettings;
    JPH_ConeConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::ConeConstraint *constraint = static_cast<JPH::ConeConstraint *>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToConeConstraint(constraint);
}

void JPH_ConeConstraint_GetSettings(JPH_ConeConstraint *constraint, JPH_ConeConstraintSettings *settings)
{
    const JPH::Ref<JPH::ConeConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::ConeConstraintSettings>(AsConeConstraint(constraint)->GetConstraintSettings());
    JPH_ConeConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_ConeConstraint_SetHalfConeAngle(JPH_ConeConstraint *constraint, const float halfConeAngle)
{
    AsConeConstraint(constraint)->SetHalfConeAngle(halfConeAngle);
}

float JPH_ConeConstraint_GetCosHalfConeAngle(const JPH_ConeConstraint *constraint)
{
    return AsConeConstraint(constraint)->GetCosHalfConeAngle();
}

void JPH_ConeConstraint_GetTotalLambdaPosition(const JPH_ConeConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsConeConstraint(constraint)->GetTotalLambdaPosition(), result);
}

float JPH_ConeConstraint_GetTotalLambdaRotation(const JPH_ConeConstraint *constraint)
{
    return AsConeConstraint(constraint)->GetTotalLambdaRotation();
}

/* JPH_SwingTwistConstraintSettings */
void JPH_SwingTwistConstraintSettings_FromJolt(JPH_SwingTwistConstraintSettings *settings,
                                               const JPH::SwingTwistConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPosition1, &settings->position1);
    FromJolt(joltSettings.mTwistAxis1, &settings->twistAxis1);
    FromJolt(joltSettings.mPlaneAxis1, &settings->planeAxis1);
    FromJolt(joltSettings.mPosition2, &settings->position2);
    FromJolt(joltSettings.mTwistAxis2, &settings->twistAxis2);
    FromJolt(joltSettings.mPlaneAxis2, &settings->planeAxis2);
    settings->swingType = static_cast<JPH_SwingType>(joltSettings.mSwingType);
    settings->normalHalfConeAngle = joltSettings.mNormalHalfConeAngle;
    settings->planeHalfConeAngle = joltSettings.mPlaneHalfConeAngle;

    settings->twistMinAngle = joltSettings.mTwistMinAngle;
    settings->twistMaxAngle = joltSettings.mTwistMaxAngle;
    settings->maxFrictionTorque = joltSettings.mMaxFrictionTorque;

    FromJolt(joltSettings.mSwingMotorSettings, &settings->swingMotorSettings);
    FromJolt(joltSettings.mTwistMotorSettings, &settings->twistMotorSettings);
}

void JPH_SwingTwistConstraintSettings_ToJolt(JPH::SwingTwistConstraintSettings *joltSettings,
                                             const JPH_SwingTwistConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mPosition1 = ToJolt(settings->position1);
    joltSettings->mTwistAxis1 = ToJolt(settings->twistAxis1);
    joltSettings->mPlaneAxis1 = ToJolt(settings->planeAxis1);
    joltSettings->mPosition2 = ToJolt(settings->position2);
    joltSettings->mTwistAxis2 = ToJolt(settings->twistAxis2);
    joltSettings->mPlaneAxis2 = ToJolt(settings->planeAxis2);
    joltSettings->mSwingType = static_cast<JPH::ESwingType>(settings->swingType);
    joltSettings->mNormalHalfConeAngle = settings->normalHalfConeAngle;
    joltSettings->mPlaneHalfConeAngle = settings->planeHalfConeAngle;
    joltSettings->mTwistMinAngle = settings->twistMinAngle;
    joltSettings->mTwistMaxAngle = settings->twistMaxAngle;
    joltSettings->mMaxFrictionTorque = settings->maxFrictionTorque;
    joltSettings->mSwingMotorSettings = ToJolt(&settings->swingMotorSettings);
    joltSettings->mTwistMotorSettings = ToJolt(&settings->twistMotorSettings);
}

void JPH_SwingTwistConstraintSettings_Init(JPH_SwingTwistConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::SwingTwistConstraintSettings joltSettings;
    JPH_SwingTwistConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_SwingTwistConstraint *JPH_SwingTwistConstraint_Create(const JPH_SwingTwistConstraintSettings *settings,
                                                          JPH_Body *body1,
                                                          JPH_Body *body2)
{
    JPH::SwingTwistConstraintSettings joltSettings;
    JPH_SwingTwistConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::SwingTwistConstraint *constraint = static_cast<JPH::SwingTwistConstraint *>(joltSettings.Create(*joltBody1,
                                                                                                         *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToSwingTwistConstraint(constraint);
}

void JPH_SwingTwistConstraint_GetSettings(JPH_SwingTwistConstraint *constraint,
                                          JPH_SwingTwistConstraintSettings *settings)
{
    const JPH::Ref<JPH::SwingTwistConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::SwingTwistConstraintSettings>(AsSwingTwistConstraint(constraint)->GetConstraintSettings());
    JPH_SwingTwistConstraintSettings_FromJolt(settings, *joltSettings);
}

float JPH_SwingTwistConstraint_GetNormalHalfConeAngle(JPH_SwingTwistConstraint *constraint)
{
    return AsSwingTwistConstraint(constraint)->GetNormalHalfConeAngle();
}

void JPH_SwingTwistConstraint_GetTotalLambdaPosition(const JPH_SwingTwistConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSwingTwistConstraint(constraint)->GetTotalLambdaPosition(), result);
}

float JPH_SwingTwistConstraint_GetTotalLambdaTwist(const JPH_SwingTwistConstraint *constraint)
{
    return AsSwingTwistConstraint(constraint)->GetTotalLambdaTwist();
}

float JPH_SwingTwistConstraint_GetTotalLambdaSwingY(const JPH_SwingTwistConstraint *constraint)
{
    JPH_ASSERT(constraint);
    const JPH::SwingTwistConstraint *joltConstraint = reinterpret_cast<const JPH::SwingTwistConstraint *>(constraint);
    return joltConstraint->GetTotalLambdaSwingY();
}

float JPH_SwingTwistConstraint_GetTotalLambdaSwingZ(const JPH_SwingTwistConstraint *constraint)
{
    JPH_ASSERT(constraint);
    const JPH::SwingTwistConstraint *joltConstraint = reinterpret_cast<const JPH::SwingTwistConstraint *>(constraint);
    return joltConstraint->GetTotalLambdaSwingZ();
}

void JPH_SwingTwistConstraint_GetTotalLambdaMotor(const JPH_SwingTwistConstraint *constraint, JPH_Vec3 *result)
{
    JPH_ASSERT(constraint);
    const JPH::SwingTwistConstraint *joltConstraint = reinterpret_cast<const JPH::SwingTwistConstraint *>(constraint);
    const JPH::Vec3 lambda = joltConstraint->GetTotalLambdaMotor();
    FromJolt(lambda, result);
}

/* JPH_SixDOFConstraintSettings */
void JPH_SixDOFConstraintSettings_FromJolt(JPH_SixDOFConstraintSettings *settings,
                                           const JPH::SixDOFConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPosition1, &settings->position1);
    FromJolt(joltSettings.mAxisX1, &settings->axisX1);
    FromJolt(joltSettings.mAxisY1, &settings->axisY1);
    FromJolt(joltSettings.mPosition2, &settings->position2);
    FromJolt(joltSettings.mAxisX2, &settings->axisX2);
    FromJolt(joltSettings.mAxisY2, &settings->axisY2);

    for (uint32_t i = 0; i < JPH::SixDOFConstraintSettings::EAxis::Num; ++i)
    {
        settings->maxFriction[i] = joltSettings.mMaxFriction[i];
        settings->limitMin[i] = joltSettings.mLimitMin[i];
        settings->limitMax[i] = joltSettings.mLimitMax[i];
        FromJolt(joltSettings.mMotorSettings[i], &settings->motorSettings[i]);
    }

    settings->swingType = static_cast<JPH_SwingType>(joltSettings.mSwingType);
    for (uint32_t i = 0; i < JPH::SixDOFConstraintSettings::EAxis::NumTranslation; ++i)
    {
        FromJolt(joltSettings.mLimitsSpringSettings[i], &settings->limitsSpringSettings[i]);
    }
}

void JPH_SixDOFConstraintSettings_ToJolt(JPH::SixDOFConstraintSettings *joltSettings,
                                         const JPH_SixDOFConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mPosition1 = ToJolt(settings->position1);
    joltSettings->mAxisX1 = ToJolt(settings->axisX1);
    joltSettings->mAxisY1 = ToJolt(settings->axisY1);
    joltSettings->mPosition2 = ToJolt(settings->position2);
    joltSettings->mAxisX2 = ToJolt(settings->axisX2);
    joltSettings->mAxisY2 = ToJolt(settings->axisY2);

    for (uint32_t i = 0; i < JPH::SixDOFConstraintSettings::EAxis::Num; ++i)
    {
        joltSettings->mMaxFriction[i] = settings->maxFriction[i];
        joltSettings->mLimitMin[i] = settings->limitMin[i];
        joltSettings->mLimitMax[i] = settings->limitMax[i];
        joltSettings->mMotorSettings[i] = ToJolt(&settings->motorSettings[i]);
    }

    joltSettings->mSwingType = static_cast<JPH::ESwingType>(settings->swingType);

    for (uint32_t i = 0; i < JPH::SixDOFConstraintSettings::EAxis::NumTranslation; ++i)
    {
        joltSettings->mLimitsSpringSettings[i] = ToJolt(&settings->limitsSpringSettings[i]);
    }
}

void JPH_SixDOFConstraintSettings_Init(JPH_SixDOFConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::SixDOFConstraintSettings joltSettings;
    JPH_SixDOFConstraintSettings_FromJolt(settings, joltSettings);
}

void JPH_SixDOFConstraintSettings_MakeFreeAxis(JPH_SixDOFConstraintSettings *settings,
                                               const JPH_SixDOFConstraintAxis axis)
{
    settings->limitMin[axis] = -FLT_MAX;
    settings->limitMax[axis] = FLT_MAX;
}

bool JPH_SixDOFConstraintSettings_IsFreeAxis(const JPH_SixDOFConstraintSettings *settings,
                                             const JPH_SixDOFConstraintAxis axis)
{
    return settings->limitMin[axis] == -FLT_MAX && settings->limitMax[axis] == FLT_MAX;
}

void JPH_SixDOFConstraintSettings_MakeFixedAxis(JPH_SixDOFConstraintSettings *settings,
                                                const JPH_SixDOFConstraintAxis axis)
{
    settings->limitMin[axis] = FLT_MAX;
    settings->limitMax[axis] = -FLT_MAX;
}

bool JPH_SixDOFConstraintSettings_IsFixedAxis(const JPH_SixDOFConstraintSettings *settings,
                                              const JPH_SixDOFConstraintAxis axis)
{
    return settings->limitMin[axis] >= settings->limitMax[axis];
}

void JPH_SixDOFConstraintSettings_SetLimitedAxis(JPH_SixDOFConstraintSettings *settings,
                                                 const JPH_SixDOFConstraintAxis axis,
                                                 const float min,
                                                 const float max)
{
    settings->limitMin[axis] = min;
    settings->limitMax[axis] = max;
}

JPH_SixDOFConstraint *JPH_SixDOFConstraint_Create(const JPH_SixDOFConstraintSettings *settings,
                                                  JPH_Body *body1,
                                                  JPH_Body *body2)
{
    JPH::SixDOFConstraintSettings joltSettings;
    JPH_SixDOFConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::SixDOFConstraint *constraint = static_cast<JPH::SixDOFConstraint *>(joltSettings.Create(*joltBody1,
                                                                                                 *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToSixDOFConstraint(constraint);
}

void JPH_SixDOFConstraint_GetSettings(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintSettings *settings)
{
    const JPH::Ref<JPH::SixDOFConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::SixDOFConstraintSettings>(AsSixDOFConstraint(constraint)->GetConstraintSettings());
    JPH_SixDOFConstraintSettings_FromJolt(settings, *joltSettings);
}

float JPH_SixDOFConstraint_GetLimitsMin(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis)
{
    return AsSixDOFConstraint(constraint)->GetLimitsMin(static_cast<JPH::SixDOFConstraint::EAxis>(axis));
}

float JPH_SixDOFConstraint_GetLimitsMax(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis)
{
    return AsSixDOFConstraint(constraint)->GetLimitsMax(static_cast<JPH::SixDOFConstraint::EAxis>(axis));
}

void JPH_SixDOFConstraint_GetTotalLambdaPosition(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaPosition(), result);
}

void JPH_SixDOFConstraint_GetTotalLambdaRotation(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaRotation(), result);
}

void JPH_SixDOFConstraint_GetTotalLambdaMotorTranslation(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaMotorTranslation(), result);
}

void JPH_SixDOFConstraint_GetTotalLambdaMotorRotation(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaMotorRotation(), result);
}

void JPH_SixDOFConstraint_GetTranslationLimitsMin(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTranslationLimitsMin(), result);
}

void JPH_SixDOFConstraint_GetTranslationLimitsMax(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTranslationLimitsMax(), result);
}

void JPH_SixDOFConstraint_GetRotationLimitsMin(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetRotationLimitsMin(), result);
}

void JPH_SixDOFConstraint_GetRotationLimitsMax(const JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetRotationLimitsMax(), result);
}

bool JPH_SixDOFConstraint_IsFixedAxis(const JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis)
{
    return AsSixDOFConstraint(constraint)->IsFixedAxis(static_cast<JPH::SixDOFConstraint::EAxis>(axis));
}

bool JPH_SixDOFConstraint_IsFreeAxis(const JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis)
{
    return AsSixDOFConstraint(constraint)->IsFreeAxis(static_cast<JPH::SixDOFConstraint::EAxis>(axis));
}

void JPH_SixDOFConstraint_GetLimitsSpringSettings(JPH_SixDOFConstraint *constraint,
                                                  JPH_SpringSettings *result,
                                                  JPH_SixDOFConstraintAxis axis)
{
    const JPH::SixDOFConstraint *const joltConstraint = reinterpret_cast<JPH::SixDOFConstraint *>(constraint);
    FromJolt(joltConstraint->GetLimitsSpringSettings(static_cast<JPH::SixDOFConstraint::EAxis>(axis)), result);
}

void JPH_SixDOFConstraint_SetLimitsSpringSettings(JPH_SixDOFConstraint *constraint,
                                                  const JPH_SpringSettings *settings,
                                                  JPH_SixDOFConstraintAxis axis)
{
    JPH::SixDOFConstraint *const joltConstraint = reinterpret_cast<JPH::SixDOFConstraint *>(constraint);
    joltConstraint->SetLimitsSpringSettings(static_cast<JPH::SixDOFConstraint::EAxis>(axis), ToJolt(settings));
}

void JPH_SixDOFConstraint_SetMaxFriction(JPH_SixDOFConstraint *constraint,
                                         JPH_SixDOFConstraintAxis axis,
                                         const float inFriction)
{
    AsSixDOFConstraint(constraint)->SetMaxFriction(static_cast<JPH::SixDOFConstraint::EAxis>(axis), inFriction);
}

float JPH_SixDOFConstraint_GetMaxFriction(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis)
{
    return AsSixDOFConstraint(constraint)->GetMaxFriction(static_cast<JPH::SixDOFConstraint::EAxis>(axis));
}

void JPH_SixDOFConstraint_GetRotationInConstraintSpace(JPH_SixDOFConstraint *constraint, JPH_Quat *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetRotationInConstraintSpace(), result);
}

void JPH_SixDOFConstraint_GetMotorSettings(JPH_SixDOFConstraint *constraint,
                                           JPH_SixDOFConstraintAxis axis,
                                           JPH_MotorSettings *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetMotorSettings(static_cast<JPH::SixDOFConstraint::EAxis>(axis)), result);
}

void JPH_SixDOFConstraint_SetMotorState(JPH_SixDOFConstraint *constraint,
                                        JPH_SixDOFConstraintAxis axis,
                                        JPH_MotorState state)
{
    AsSixDOFConstraint(constraint)
            ->SetMotorState(static_cast<JPH::SixDOFConstraint::EAxis>(axis), static_cast<JPH::EMotorState>(state));
}

JPH_MotorState JPH_SixDOFConstraint_GetMotorState(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis)
{
    return static_cast<JPH_MotorState>(AsSixDOFConstraint(constraint)
                                               ->GetMotorState(static_cast<JPH::SixDOFConstraint::EAxis>(axis)));
}

void JPH_SixDOFConstraint_SetTargetVelocityCS(JPH_SixDOFConstraint *constraint, const JPH_Vec3 *inVelocity)
{
    AsSixDOFConstraint(constraint)->SetTargetVelocityCS(ToJolt(inVelocity));
}

void JPH_SixDOFConstraint_GetTargetVelocityCS(JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTargetVelocityCS(), result);
}

void JPH_SixDOFConstraint_SetTargetAngularVelocityCS(JPH_SixDOFConstraint *constraint,
                                                     const JPH_Vec3 *inAngularVelocity)
{
    AsSixDOFConstraint(constraint)->SetTargetAngularVelocityCS(ToJolt(inAngularVelocity));
}

void JPH_SixDOFConstraint_GetTargetAngularVelocityCS(JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTargetAngularVelocityCS(), result);
}

void JPH_SixDOFConstraint_SetTargetPositionCS(JPH_SixDOFConstraint *constraint, const JPH_Vec3 *inPosition)
{
    AsSixDOFConstraint(constraint)->SetTargetPositionCS(ToJolt(inPosition));
}

void JPH_SixDOFConstraint_GetTargetPositionCS(JPH_SixDOFConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTargetPositionCS(), result);
}

void JPH_SixDOFConstraint_SetTargetOrientationCS(JPH_SixDOFConstraint *constraint, const JPH_Quat *inOrientation)
{
    AsSixDOFConstraint(constraint)->SetTargetOrientationCS(ToJolt(inOrientation));
}

void JPH_SixDOFConstraint_GetTargetOrientationCS(JPH_SixDOFConstraint *constraint, JPH_Quat *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTargetOrientationCS(), result);
}

void JPH_SixDOFConstraint_SetTargetOrientationBS(JPH_SixDOFConstraint *constraint, const JPH_Quat *inOrientation)
{
    AsSixDOFConstraint(constraint)->SetTargetOrientationBS(ToJolt(inOrientation));
}

/* JPH_GearConstraint */
void JPH_GearConstraintSettings_FromJolt(JPH_GearConstraintSettings *settings,
                                         const JPH::GearConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mHingeAxis1, &settings->hingeAxis1);
    FromJolt(joltSettings.mHingeAxis2, &settings->hingeAxis2);
    settings->ratio = joltSettings.mRatio;
}

void JPH_GearConstraintSettings_ToJolt(JPH::GearConstraintSettings *joltSettings,
                                       const JPH_GearConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
    joltSettings->mHingeAxis1 = ToJolt(settings->hingeAxis1);
    joltSettings->mHingeAxis2 = ToJolt(settings->hingeAxis2);
    joltSettings->mRatio = settings->ratio;
}

void JPH_GearConstraintSettings_Init(JPH_GearConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::GearConstraintSettings joltSettings;
    JPH_GearConstraintSettings_FromJolt(settings, joltSettings);
}

JPH_GearConstraint *JPH_GearConstraint_Create(const JPH_GearConstraintSettings *settings,
                                              JPH_Body *body1,
                                              JPH_Body *body2)
{
    JPH::GearConstraintSettings joltSettings;
    JPH_GearConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::Body *joltBody1 = reinterpret_cast<JPH::Body *>(body1);
    JPH::Body *joltBody2 = reinterpret_cast<JPH::Body *>(body2);

    JPH::GearConstraint *constraint = static_cast<JPH::GearConstraint *>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToGearConstraint(constraint);
}

void JPH_GearConstraint_GetSettings(JPH_GearConstraint *constraint, JPH_GearConstraintSettings *settings)
{
    const JPH::Ref<JPH::GearConstraintSettings> joltSettings = JPH::StaticCast<
            JPH::GearConstraintSettings>(AsGearConstraint(constraint)->GetConstraintSettings());
    JPH_GearConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_GearConstraint_SetConstraints(JPH_GearConstraint *constraint,
                                       const JPH_Constraint *gear1,
                                       const JPH_Constraint *gear2)
{
    AsGearConstraint(constraint)->SetConstraints(AsConstraint(gear1), AsConstraint(gear2));
}

float JPH_GearConstraint_GetTotalLambda(const JPH_GearConstraint *constraint)
{
    return AsGearConstraint(constraint)->GetTotalLambda();
}

/* JPH_MotionProperties */
JPH_AllowedDOFs JPH_MotionProperties_GetAllowedDOFs(const JPH_MotionProperties *properties)
{
    return static_cast<JPH_AllowedDOFs>(reinterpret_cast<const JPH::MotionProperties *>(properties)->GetAllowedDOFs());
}

void JPH_MotionProperties_SetLinearDamping(JPH_MotionProperties *properties, const float damping)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetLinearDamping(damping);
}

float JPH_MotionProperties_GetLinearDamping(const JPH_MotionProperties *properties)
{
    return reinterpret_cast<const JPH::MotionProperties *>(properties)->GetLinearDamping();
}

void JPH_MotionProperties_SetAngularDamping(JPH_MotionProperties *properties, const float damping)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetAngularDamping(damping);
}

float JPH_MotionProperties_GetAngularDamping(const JPH_MotionProperties *properties)
{
    return reinterpret_cast<const JPH::MotionProperties *>(properties)->GetAngularDamping();
}

void JPH_MotionProperties_SetMassProperties(JPH_MotionProperties *properties,
                                            JPH_AllowedDOFs allowedDOFs,
                                            const JPH_MassProperties *massProperties)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)
            ->SetMassProperties(static_cast<JPH::EAllowedDOFs>(allowedDOFs), ToJolt(massProperties));
}

float JPH_MotionProperties_GetInverseMassUnchecked(JPH_MotionProperties *properties)
{
    return reinterpret_cast<JPH::MotionProperties *>(properties)->GetInverseMassUnchecked();
}

void JPH_MotionProperties_SetInverseMass(JPH_MotionProperties *properties, const float inverseMass)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetInverseMass(inverseMass);
}

void JPH_MotionProperties_GetInverseInertiaDiagonal(JPH_MotionProperties *properties, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<JPH::MotionProperties *>(properties)->GetInverseInertiaDiagonal(), result);
}

void JPH_MotionProperties_GetInertiaRotation(JPH_MotionProperties *properties, JPH_Quat *result)
{
    FromJolt(reinterpret_cast<JPH::MotionProperties *>(properties)->GetInertiaRotation(), result);
}

void JPH_MotionProperties_SetInverseInertia(JPH_MotionProperties *properties,
                                            const JPH_Vec3 *diagonal,
                                            const JPH_Quat *rot)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetInverseInertia(ToJolt(diagonal), ToJolt(rot));
}

void JPH_MotionProperties_ScaleToMass(JPH_MotionProperties *properties, const float mass)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->ScaleToMass(mass);
}

const JPH_BroadPhaseQuery *JPH_PhysicsSystem_GetBroadPhaseQuery(const JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    return reinterpret_cast<const JPH_BroadPhaseQuery *>(&system->physicsSystem->GetBroadPhaseQuery());
}

const JPH_NarrowPhaseQuery *JPH_PhysicsSystem_GetNarrowPhaseQuery(const JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    return reinterpret_cast<const JPH_NarrowPhaseQuery *>(&system->physicsSystem->GetNarrowPhaseQuery());
}
const JPH_NarrowPhaseQuery *JPH_PhysicsSystem_GetNarrowPhaseQueryNoLock(const JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    return reinterpret_cast<const JPH_NarrowPhaseQuery *>(&system->physicsSystem->GetNarrowPhaseQueryNoLock());
}

void JPH_PhysicsSystem_SetContactListener(JPH_PhysicsSystem *system, JPH_ContactListener *listener)
{
    JPH_ASSERT(system);

    JPH::ContactListener *joltListener = reinterpret_cast<JPH::ContactListener *>(listener);
    system->physicsSystem->SetContactListener(joltListener);
}

void JPH_PhysicsSystem_SetBodyActivationListener(JPH_PhysicsSystem *system, JPH_BodyActivationListener *listener)
{
    JPH_ASSERT(system);

    JPH::BodyActivationListener *joltListener = reinterpret_cast<JPH::BodyActivationListener *>(listener);
    system->physicsSystem->SetBodyActivationListener(joltListener);
}

void JPH_PhysicsSystem_SetSimShapeFilter(JPH_PhysicsSystem *system, JPH_SimShapeFilter *filter)
{
    JPH_ASSERT(system);

    JPH::SimShapeFilter *joltFilter = reinterpret_cast<JPH::SimShapeFilter *>(filter);
    system->physicsSystem->SetSimShapeFilter(joltFilter);
}

bool JPH_PhysicsSystem_WereBodiesInContact(const JPH_PhysicsSystem *system, JPH_BodyID body1, JPH_BodyID body2)
{
    JPH_ASSERT(system);

    return system->physicsSystem->WereBodiesInContact(JPH::BodyID(body1), JPH::BodyID(body2));
}

uint32_t JPH_PhysicsSystem_GetNumBodies(const JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    return system->physicsSystem->GetNumBodies();
}

uint32_t JPH_PhysicsSystem_GetNumActiveBodies(const JPH_PhysicsSystem *system, JPH_BodyType type)
{
    JPH_ASSERT(system);

    return system->physicsSystem->GetNumActiveBodies(static_cast<JPH::EBodyType>(type));
}

uint32_t JPH_PhysicsSystem_GetMaxBodies(const JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    return system->physicsSystem->GetMaxBodies();
}

uint32_t JPH_PhysicsSystem_GetNumConstraints(const JPH_PhysicsSystem *system)
{
    JPH_ASSERT(system);

    return static_cast<uint32_t>(system->physicsSystem->GetConstraints().size());
}

void JPH_PhysicsSystem_SetGravity(JPH_PhysicsSystem *system, const JPH_Vec3 *value)
{
    JPH_ASSERT(system);

    system->physicsSystem->SetGravity(ToJolt(value));
}

void JPH_PhysicsSystem_GetGravity(JPH_PhysicsSystem *system, JPH_Vec3 *result)
{
    JPH_ASSERT(system);

    const JPH::Vec3 joltVector = system->physicsSystem->GetGravity();
    FromJolt(joltVector, result);
}

void JPH_PhysicsSystem_AddConstraint(JPH_PhysicsSystem *system, JPH_Constraint *constraint)
{
    JPH_ASSERT(system);
    JPH_ASSERT(constraint);

    JPH::Constraint *joltConstraint = reinterpret_cast<JPH::Constraint *>(constraint);
    system->physicsSystem->AddConstraint(joltConstraint);
}

void JPH_PhysicsSystem_RemoveConstraint(JPH_PhysicsSystem *system, JPH_Constraint *constraint)
{
    JPH_ASSERT(system);
    JPH_ASSERT(constraint);

    JPH::Constraint *joltConstraint = reinterpret_cast<JPH::Constraint *>(constraint);
    system->physicsSystem->RemoveConstraint(joltConstraint);
}

void JPH_PhysicsSystem_AddConstraints(JPH_PhysicsSystem *system, JPH_Constraint **constraints, int count)
{
    JPH_ASSERT(system);
    JPH_ASSERT(constraints);
    JPH_ASSERT(count > 0);

    JPH::Array<JPH::Constraint *> joltConstraints;
    joltConstraints.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        JPH::Constraint *joltConstraint = reinterpret_cast<JPH::Constraint *>(constraints[i]);
        joltConstraints.push_back(joltConstraint);
    }

    system->physicsSystem->AddConstraints(joltConstraints.data(), count);
}

void JPH_PhysicsSystem_RemoveConstraints(JPH_PhysicsSystem *system, JPH_Constraint **constraints, int count)
{
    JPH_ASSERT(system);
    JPH_ASSERT(constraints);
    JPH_ASSERT(count > 0);

    JPH::Array<JPH::Constraint *> joltConstraints;
    joltConstraints.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        JPH::Constraint *joltConstraint = reinterpret_cast<JPH::Constraint *>(constraints[i]);
        joltConstraints.push_back(joltConstraint);
    }

    system->physicsSystem->RemoveConstraints(joltConstraints.data(), count);
}

void JPH_PhysicsSystem_AddStepListener(JPH_PhysicsSystem *system, JPH_PhysicsStepListener *listener)
{
    JPH_ASSERT(system);
    JPH_ASSERT(listener);

    system->physicsSystem->AddStepListener(AsPhysicsStepListener(listener));
}

void JPH_PhysicsSystem_RemoveStepListener(JPH_PhysicsSystem *system, JPH_PhysicsStepListener *listener)
{
    JPH_ASSERT(system);
    JPH_ASSERT(listener);

    system->physicsSystem->RemoveStepListener(AsPhysicsStepListener(listener));
}

void JPH_PhysicsSystem_GetBodies(const JPH_PhysicsSystem *system, JPH_BodyID *ids, uint32_t count)
{
    JPH_ASSERT(system);
    JPH_ASSERT(ids);
    JPH_ASSERT(count <= JPH_PhysicsSystem_GetNumBodies(system));

    JPH::BodyIDVector bodies;
    system->physicsSystem->GetBodies(bodies);

    for (uint32_t i = 0; i < count; i++)
    {
        ids[i] = bodies[i].GetIndexAndSequenceNumber();
    }
}

void JPH_PhysicsSystem_GetConstraints(const JPH_PhysicsSystem *system,
                                      const JPH_Constraint **constraints,
                                      uint32_t count)
{
    JPH_ASSERT(constraints);

    JPH::Constraints list = system->physicsSystem->GetConstraints();

    for (uint32_t i = 0; i < count && i < list.size(); i++)
    {
        constraints[i] = ToConstraint(list[i].GetPtr());
    }
}

#ifdef JPH_DEBUG_RENDERER
static inline JPH::BodyManager::DrawSettings ToJolt(const JPH_DrawSettings *settings)
{
    JPH::BodyManager::DrawSettings result{};
    if (settings == nullptr)
    {
        return result;
    }
    result.mDrawGetSupportFunction = settings->drawGetSupportFunction;
    result.mDrawSupportDirection = settings->drawSupportDirection;
    result.mDrawGetSupportingFace = settings->drawGetSupportingFace;
    result.mDrawShape = settings->drawShape;
    result.mDrawShapeWireframe = settings->drawShapeWireframe;
    result.mDrawShapeColor = static_cast<JPH::BodyManager::EShapeColor>(settings->drawShapeColor);
    result.mDrawBoundingBox = settings->drawBoundingBox;
    result.mDrawCenterOfMassTransform = settings->drawCenterOfMassTransform;
    result.mDrawWorldTransform = settings->drawWorldTransform;
    result.mDrawVelocity = settings->drawVelocity;
    result.mDrawMassAndInertia = settings->drawMassAndInertia;
    result.mDrawSleepStats = settings->drawSleepStats;
    result.mDrawSoftBodyVertices = settings->drawSoftBodyVertices;
    result.mDrawSoftBodyVertexVelocities = settings->drawSoftBodyVertexVelocities;
    result.mDrawSoftBodyEdgeConstraints = settings->drawSoftBodyEdgeConstraints;
    result.mDrawSoftBodyBendConstraints = settings->drawSoftBodyBendConstraints;
    result.mDrawSoftBodyVolumeConstraints = settings->drawSoftBodyVolumeConstraints;
    result.mDrawSoftBodySkinConstraints = settings->drawSoftBodySkinConstraints;
    result.mDrawSoftBodyLRAConstraints = settings->drawSoftBodyLRAConstraints;
    result.mDrawSoftBodyPredictedBounds = settings->drawSoftBodyPredictedBounds;
    result.mDrawSoftBodyConstraintColor = static_cast<
            JPH::ESoftBodyConstraintColor>(settings->drawSoftBodyConstraintColor);
    return result;
}

void JPH_PhysicsSystem_DrawBodies(const JPH_PhysicsSystem *system,
                                  const JPH_DrawSettings *settings,
                                  JPH_DebugRenderer *renderer,
                                  const JPH_BodyDrawFilter *bodyFilter)
{
    const JPH::BodyManager::DrawSettings joltSettings = ToJolt(settings);
    const JPH::BodyDrawFilter *bodyDrawFilter = bodyFilter != nullptr
                                                        ? reinterpret_cast<const JPH::BodyDrawFilter *>(bodyFilter)
                                                        : nullptr;
    system->physicsSystem->DrawBodies(joltSettings, reinterpret_cast<JPH::DebugRenderer *>(renderer), bodyDrawFilter);
}

void JPH_PhysicsSystem_DrawConstraints(const JPH_PhysicsSystem *system, JPH_DebugRenderer *renderer)
{
    system->physicsSystem->DrawConstraints(reinterpret_cast<JPH::DebugRenderer *>(renderer));
}

void JPH_PhysicsSystem_DrawConstraintLimits(const JPH_PhysicsSystem *system, JPH_DebugRenderer *renderer)
{
    system->physicsSystem->DrawConstraintLimits(reinterpret_cast<JPH::DebugRenderer *>(renderer));
}

void JPH_PhysicsSystem_DrawConstraintReferenceFrame(const JPH_PhysicsSystem *system, JPH_DebugRenderer *renderer)
{
    system->physicsSystem->DrawConstraintReferenceFrame(reinterpret_cast<JPH::DebugRenderer *>(renderer));
}
#endif

/* PhysicsStepListener */
class ManagedPhysicsStepListener final: public JPH::PhysicsStepListener
{
    public:
        static const JPH_PhysicsStepListener_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedPhysicsStepListener(void *userData_): userData(userData_) {}

        void OnStep(const JPH::PhysicsStepListenerContext &inContext) override
        {
            if (s_Procs != nullptr && s_Procs->OnStep != nullptr)
            {
                JPH_PhysicsStepListenerContext context{};
                context.deltaTime = inContext.mDeltaTime;
                context.isFirstStep = static_cast<JPH_Bool>(inContext.mIsFirstStep);
                context.isLastStep = static_cast<JPH_Bool>(inContext.mIsLastStep);
                context.physicsSystem = s_PhysicsSystems[inContext.mPhysicsSystem];
                s_Procs->OnStep(userData, &context);
            }
        }
};

const JPH_PhysicsStepListener_Procs *ManagedPhysicsStepListener::s_Procs = nullptr;

void JPH_PhysicsStepListener_SetProcs(const JPH_PhysicsStepListener_Procs *procs)
{
    ManagedPhysicsStepListener::s_Procs = procs;
}

JPH_PhysicsStepListener *JPH_PhysicsStepListener_Create(void *userData)
{
    ManagedPhysicsStepListener *const listener = new ManagedPhysicsStepListener(userData);
    return ToPhysicsStepListener(listener);
}

void JPH_PhysicsStepListener_Destroy(JPH_PhysicsStepListener *listener)
{
    if (listener != nullptr)
    {
        delete reinterpret_cast<ManagedPhysicsStepListener *>(listener);
    }
}

JPH_Body *JPH_BodyInterface_CreateBody(JPH_BodyInterface *interface, const JPH_BodyCreationSettings *settings)
{
    return ToBody(AsBodyInterface(interface)->CreateBody(*AsBodyCreationSettings(settings)));
}

JPH_Body *JPH_BodyInterface_CreateBodyWithID(JPH_BodyInterface *interface,
                                             const JPH_BodyID bodyID,
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

bool JPH_BodyInterface_AssignBodyID2(JPH_BodyInterface *interface, JPH_Body *body, const JPH_BodyID bodyID)
{
    return AsBodyInterface(interface)->AssignBodyID(AsBody(body), JPH::BodyID(bodyID));
}

JPH_Body *JPH_BodyInterface_UnassignBodyID(JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return ToBody(AsBodyInterface(interface)->UnassignBodyID(JPH::BodyID(bodyID)));
}

JPH_BodyID JPH_BodyInterface_CreateAndAddBody(JPH_BodyInterface *interface,
                                              const JPH_BodyCreationSettings *settings,
                                              JPH_Activation activationMode)
{
    const JPH::BodyID
            bodyID = AsBodyInterface(interface)
                             ->CreateAndAddBody(*reinterpret_cast<const JPH::BodyCreationSettings *>(settings),
                                                static_cast<JPH::EActivation>(activationMode));

    return bodyID.GetIndexAndSequenceNumber();
}

void JPH_BodyInterface_DestroyBody(JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    AsBodyInterface(interface)->DestroyBody(JPH::BodyID(bodyID));
}

JPH_Body *JPH_BodyInterface_CreateSoftBody(JPH_BodyInterface *interface, const JPH_SoftBodyCreationSettings *settings)
{
    JPH::Body *body = AsBodyInterface(interface)->CreateSoftBody(*AsSoftBodyCreationSettings(settings));
    return ToBody(body);
}

JPH_Body *JPH_BodyInterface_CreateSoftBodyWithID(JPH_BodyInterface *interface,
                                                 const JPH_BodyID bodyID,
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

JPH_BodyID JPH_BodyInterface_CreateAndAddSoftBody(JPH_BodyInterface *interface,
                                                  const JPH_SoftBodyCreationSettings *settings,
                                                  JPH_Activation activationMode)
{
    const JPH::BodyID bodyID = AsBodyInterface(interface)
                                       ->CreateAndAddSoftBody(*AsSoftBodyCreationSettings(settings),
                                                              static_cast<JPH::EActivation>(activationMode));
    return bodyID.GetIndexAndSequenceNumber();
}

void JPH_BodyInterface_AddBody(JPH_BodyInterface *interface, JPH_BodyID bodyID, JPH_Activation activationMode)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(interface)->AddBody(joltBodyID, static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_RemoveBody(JPH_BodyInterface *interface, JPH_BodyID bodyID)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(interface)->RemoveBody(joltBodyID);
}

void JPH_BodyInterface_RemoveAndDestroyBody(JPH_BodyInterface *interface, JPH_BodyID bodyID)
{
    const JPH::BodyID joltBodyID(bodyID);
    JPH_ASSERT(!joltBodyID.IsInvalid());

    AsBodyInterface(interface)->RemoveBody(joltBodyID);
    AsBodyInterface(interface)->DestroyBody(joltBodyID);
}

bool JPH_BodyInterface_IsActive(JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(interface)->IsActive(JPH::BodyID(bodyID));
}

bool JPH_BodyInterface_IsAdded(JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(interface)->IsAdded(JPH::BodyID(bodyID));
}

JPH_BodyType JPH_BodyInterface_GetBodyType(JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return static_cast<JPH_BodyType>(AsBodyInterface(interface)->GetBodyType(JPH::BodyID(bodyID)));
}

void JPH_BodyInterface_SetLinearVelocity(JPH_BodyInterface *interface,
                                         const JPH_BodyID bodyID,
                                         const JPH_Vec3 *velocity)
{
    AsBodyInterface(interface)->SetLinearVelocity(JPH::BodyID(bodyID), ToJolt(velocity));
}

void JPH_BodyInterface_GetLinearVelocity(JPH_BodyInterface *interface, const JPH_BodyID bodyID, JPH_Vec3 *velocity)
{
    FromJolt(AsBodyInterface(interface)->GetLinearVelocity(JPH::BodyID(bodyID)), velocity);
}

void JPH_BodyInterface_GetCenterOfMassPosition(JPH_BodyInterface *interface,
                                               const JPH_BodyID bodyID,
                                               JPH_RVec3 *position)
{
    FromJolt(AsBodyInterface(interface)->GetCenterOfMassPosition(JPH::BodyID(bodyID)), position);
}

JPH_MotionType JPH_BodyInterface_GetMotionType(JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return static_cast<JPH_MotionType>(AsBodyInterface(interface)->GetMotionType(JPH::BodyID(bodyID)));
}

void JPH_BodyInterface_SetMotionType(JPH_BodyInterface *interface,
                                     const JPH_BodyID bodyID,
                                     JPH_MotionType motionType,
                                     JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetMotionType(JPH::BodyID(bodyID),
                                              static_cast<JPH::EMotionType>(motionType),
                                              static_cast<JPH::EActivation>(activationMode));
}

float JPH_BodyInterface_GetRestitution(const JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(interface)->GetRestitution(JPH::BodyID(bodyID));
}

void JPH_BodyInterface_SetRestitution(JPH_BodyInterface *interface, const JPH_BodyID bodyID, const float restitution)
{
    AsBodyInterface(interface)->SetRestitution(JPH::BodyID(bodyID), restitution);
}

float JPH_BodyInterface_GetFriction(const JPH_BodyInterface *interface, const JPH_BodyID bodyID)
{
    return AsBodyInterface(interface)->GetFriction(JPH::BodyID(bodyID));
}

void JPH_BodyInterface_SetFriction(JPH_BodyInterface *interface, const JPH_BodyID bodyID, const float friction)
{
    AsBodyInterface(interface)->SetFriction(JPH::BodyID(bodyID), friction);
}

void JPH_BodyInterface_SetPosition(JPH_BodyInterface *interface,
                                   const JPH_BodyID bodyId,
                                   const JPH_RVec3 *position,
                                   JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetPosition(JPH::BodyID(bodyId),
                                            ToJolt(position),
                                            static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetPosition(JPH_BodyInterface *interface, const JPH_BodyID bodyId, JPH_RVec3 *result)
{
    FromJolt(AsBodyInterface(interface)->GetPosition(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetRotation(JPH_BodyInterface *interface,
                                   const JPH_BodyID bodyId,
                                   const JPH_Quat *rotation,
                                   JPH_Activation activationMode)
{
    AsBodyInterface(interface)->SetRotation(JPH::BodyID(bodyId),
                                            ToJolt(rotation),
                                            static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_GetRotation(JPH_BodyInterface *interface, const JPH_BodyID bodyId, JPH_Quat *result)
{
    FromJolt(AsBodyInterface(interface)->GetRotation(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetPositionAndRotation(JPH_BodyInterface *interface,
                                              const JPH_BodyID bodyId,
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
                                                         const JPH_BodyID bodyId,
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
                                              const JPH_BodyID bodyId,
                                              JPH_RVec3 *position,
                                              JPH_Quat *rotation)
{
    JPH::RVec3 joltPosition;
    JPH::Quat joltRotation;
    AsBodyInterface(interface)->GetPositionAndRotation(JPH::BodyID(bodyId), joltPosition, joltRotation);
    FromJolt(joltPosition, position);
    FromJolt(joltRotation, rotation);
}

void JPH_BodyInterface_SetPositionRotationAndVelocity(JPH_BodyInterface *interface,
                                                      const JPH_BodyID bodyId,
                                                      const JPH_RVec3 *position,
                                                      const JPH_Quat *rotation,
                                                      const JPH_Vec3 *linearVelocity,
                                                      const JPH_Vec3 *angularVelocity)
{
    AsBodyInterface(interface)->SetPositionRotationAndVelocity(JPH::BodyID(bodyId),
                                                               ToJolt(position),
                                                               ToJolt(rotation),
                                                               ToJolt(linearVelocity),
                                                               ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetCollisionGroup(JPH_BodyInterface *interface,
                                         const JPH_BodyID bodyId,
                                         JPH_CollisionGroup *result)
{
    FromJolt(AsBodyInterface(interface)->GetCollisionGroup(JPH::BodyID(bodyId)), result);
}

void JPH_BodyInterface_SetCollisionGroup(JPH_BodyInterface *interface,
                                         const JPH_BodyID bodyId,
                                         const JPH_CollisionGroup *group)
{
    AsBodyInterface(interface)->SetCollisionGroup(JPH::BodyID(bodyId), ToJolt(group));
}

const JPH_Shape *JPH_BodyInterface_GetShape(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    const JPH::Shape *shape = AsBodyInterface(interface)->GetShape(JPH::BodyID(bodyId)).GetPtr();
    return ToShape(shape);
}

void JPH_BodyInterface_SetShape(JPH_BodyInterface *interface,
                                const JPH_BodyID bodyId,
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
                                          const JPH_BodyID bodyId,
                                          const JPH_Vec3 *previousCenterOfMass,
                                          const bool updateMassProperties,
                                          JPH_Activation activationMode)
{
    AsBodyInterface(interface)->NotifyShapeChanged(JPH::BodyID(bodyId),
                                                   ToJolt(previousCenterOfMass),
                                                   updateMassProperties,
                                                   static_cast<JPH::EActivation>(activationMode));
}

void JPH_BodyInterface_ActivateBody(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    AsBodyInterface(interface)->ActivateBody(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_DeactivateBody(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    AsBodyInterface(interface)->DeactivateBody(JPH::BodyID(bodyId));
}

JPH_ObjectLayer JPH_BodyInterface_GetObjectLayer(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(interface)->GetObjectLayer(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetObjectLayer(JPH_BodyInterface *interface,
                                      const JPH_BodyID bodyId,
                                      const JPH_ObjectLayer layer)
{
    AsBodyInterface(interface)->SetObjectLayer(JPH::BodyID(bodyId), layer);
}

void JPH_BodyInterface_GetWorldTransform(JPH_BodyInterface *interface, const JPH_BodyID bodyId, JPH_RMatrix4x4 *result)
{
    const JPH::RMat44 &mat = AsBodyInterface(interface)->GetWorldTransform(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_GetCenterOfMassTransform(JPH_BodyInterface *interface,
                                                const JPH_BodyID bodyId,
                                                JPH_RMatrix4x4 *result)
{
    const JPH::RMat44 &mat = AsBodyInterface(interface)->GetCenterOfMassTransform(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_MoveKinematic(JPH_BodyInterface *interface,
                                     const JPH_BodyID bodyId,
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
                                            const JPH_BodyID bodyId,
                                            const JPH_RVec3 *surfacePosition,
                                            const JPH_Vec3 *surfaceNormal,
                                            const float buoyancy,
                                            const float linearDrag,
                                            const float angularDrag,
                                            const JPH_Vec3 *fluidVelocity,
                                            const JPH_Vec3 *gravity,
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
                                                   const JPH_BodyID bodyId,
                                                   const JPH_Vec3 *linearVelocity,
                                                   const JPH_Vec3 *angularVelocity)
{
    AsBodyInterface(interface)->SetLinearAndAngularVelocity(JPH::BodyID(bodyId),
                                                            ToJolt(linearVelocity),
                                                            ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetLinearAndAngularVelocity(JPH_BodyInterface *interface,
                                                   const JPH_BodyID bodyId,
                                                   JPH_Vec3 *linearVelocity,
                                                   JPH_Vec3 *angularVelocity)
{
    JPH::Vec3 linear;
    JPH::Vec3 angular;
    AsBodyInterface(interface)->GetLinearAndAngularVelocity(JPH::BodyID(bodyId), linear, angular);
    FromJolt(linear, linearVelocity);
    FromJolt(angular, angularVelocity);
}

void JPH_BodyInterface_AddLinearVelocity(JPH_BodyInterface *interface,
                                         const JPH_BodyID bodyId,
                                         const JPH_Vec3 *linearVelocity)
{
    AsBodyInterface(interface)->AddLinearVelocity(JPH::BodyID(bodyId), ToJolt(linearVelocity));
}

void JPH_BodyInterface_AddLinearAndAngularVelocity(JPH_BodyInterface *interface,
                                                   const JPH_BodyID bodyId,
                                                   const JPH_Vec3 *linearVelocity,
                                                   const JPH_Vec3 *angularVelocity)
{
    AsBodyInterface(interface)->AddLinearAndAngularVelocity(JPH::BodyID(bodyId),
                                                            ToJolt(linearVelocity),
                                                            ToJolt(angularVelocity));
}

void JPH_BodyInterface_SetAngularVelocity(JPH_BodyInterface *interface,
                                          const JPH_BodyID bodyId,
                                          const JPH_Vec3 *angularVelocity)
{
    AsBodyInterface(interface)->SetAngularVelocity(JPH::BodyID(bodyId), ToJolt(angularVelocity));
}

void JPH_BodyInterface_GetAngularVelocity(JPH_BodyInterface *interface,
                                          const JPH_BodyID bodyId,
                                          JPH_Vec3 *angularVelocity)
{
    const JPH::Vec3 result = AsBodyInterface(interface)->GetAngularVelocity(JPH::BodyID(bodyId));
    FromJolt(result, angularVelocity);
}

void JPH_BodyInterface_GetPointVelocity(JPH_BodyInterface *interface,
                                        const JPH_BodyID bodyId,
                                        const JPH_RVec3 *point,
                                        JPH_Vec3 *velocity)
{
    const JPH::Vec3 result = AsBodyInterface(interface)->GetPointVelocity(JPH::BodyID(bodyId), ToJolt(point));
    FromJolt(result, velocity);
}

void JPH_BodyInterface_AddForce(JPH_BodyInterface *interface, const JPH_BodyID bodyId, const JPH_Vec3 *force)
{
    AsBodyInterface(interface)->AddForce(JPH::BodyID(bodyId), ToJolt(force));
}

void JPH_BodyInterface_AddForce2(JPH_BodyInterface *interface,
                                 const JPH_BodyID bodyId,
                                 const JPH_Vec3 *force,
                                 const JPH_RVec3 *point)
{
    AsBodyInterface(interface)->AddForce(JPH::BodyID(bodyId), ToJolt(force), ToJolt(point));
}

void JPH_BodyInterface_AddTorque(JPH_BodyInterface *interface, const JPH_BodyID bodyId, const JPH_Vec3 *torque)
{
    AsBodyInterface(interface)->AddTorque(JPH::BodyID(bodyId), ToJolt(torque));
}

void JPH_BodyInterface_AddForceAndTorque(JPH_BodyInterface *interface,
                                         const JPH_BodyID bodyId,
                                         const JPH_Vec3 *force,
                                         const JPH_Vec3 *torque)
{
    AsBodyInterface(interface)->AddForceAndTorque(JPH::BodyID(bodyId), ToJolt(force), ToJolt(torque));
}

void JPH_BodyInterface_AddImpulse(JPH_BodyInterface *interface, const JPH_BodyID bodyId, const JPH_Vec3 *impulse)
{
    AsBodyInterface(interface)->AddImpulse(JPH::BodyID(bodyId), ToJolt(impulse));
}

void JPH_BodyInterface_AddImpulse2(JPH_BodyInterface *interface,
                                   const JPH_BodyID bodyId,
                                   const JPH_Vec3 *impulse,
                                   const JPH_RVec3 *point)
{
    AsBodyInterface(interface)->AddImpulse(JPH::BodyID(bodyId), ToJolt(impulse), ToJolt(point));
}

void JPH_BodyInterface_AddAngularImpulse(JPH_BodyInterface *interface,
                                         const JPH_BodyID bodyId,
                                         const JPH_Vec3 *angularImpulse)
{
    AsBodyInterface(interface)->AddAngularImpulse(JPH::BodyID(bodyId), ToJolt(angularImpulse));
}

void JPH_BodyInterface_SetMotionQuality(JPH_BodyInterface *interface,
                                        const JPH_BodyID bodyId,
                                        JPH_MotionQuality quality)
{
    AsBodyInterface(interface)->SetMotionQuality(JPH::BodyID(bodyId), static_cast<JPH::EMotionQuality>(quality));
}

JPH_MotionQuality JPH_BodyInterface_GetMotionQuality(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    return static_cast<JPH_MotionQuality>(AsBodyInterface(interface)->GetMotionQuality(JPH::BodyID(bodyId)));
}

void JPH_BodyInterface_GetInverseInertia(JPH_BodyInterface *interface, const JPH_BodyID bodyId, JPH_Matrix4x4 *result)
{
    const JPH::Mat44 &mat = AsBodyInterface(interface)->GetInverseInertia(JPH::BodyID(bodyId));
    FromJolt(mat, result);
}

void JPH_BodyInterface_SetGravityFactor(JPH_BodyInterface *interface, const JPH_BodyID bodyId, const float value)
{
    AsBodyInterface(interface)->SetGravityFactor(JPH::BodyID(bodyId), value);
}

float JPH_BodyInterface_GetGravityFactor(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(interface)->GetGravityFactor(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetUseManifoldReduction(JPH_BodyInterface *interface, const JPH_BodyID bodyId, const bool value)
{
    AsBodyInterface(interface)->SetUseManifoldReduction(JPH::BodyID(bodyId), value);
}

bool JPH_BodyInterface_GetUseManifoldReduction(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(interface)->GetUseManifoldReduction(JPH::BodyID(bodyId));
}

void JPH_BodyInterface_SetUserData(JPH_BodyInterface *interface, const JPH_BodyID bodyId, const uint64_t userData)
{
    AsBodyInterface(interface)->SetUserData(JPH::BodyID(bodyId), userData);
}

uint64_t JPH_BodyInterface_GetUserData(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    return AsBodyInterface(interface)->GetUserData(JPH::BodyID(bodyId));
}

const JPH_PhysicsMaterial *JPH_BodyInterface_GetMaterial(JPH_BodyInterface *interface,
                                                         const JPH_BodyID bodyId,
                                                         const JPH_SubShapeID subShapeID)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    return FromJolt(AsBodyInterface(interface)->GetMaterial(JPH::BodyID(bodyId), joltSubShapeID));
}

void JPH_BodyInterface_InvalidateContactCache(JPH_BodyInterface *interface, const JPH_BodyID bodyId)
{
    AsBodyInterface(interface)->InvalidateContactCache(JPH::BodyID(bodyId));
}

//--------------------------------------------------------------------------------------------------
// JPH_BodyLockInterface
//--------------------------------------------------------------------------------------------------
void JPH_BodyLockInterface_LockRead(const JPH_BodyLockInterface *lockInterface,
                                    JPH_BodyID bodyID,
                                    JPH_BodyLockRead *outLock)
{
    JPH_ASSERT(outLock != nullptr);
    const JPH::BodyLockInterface *joltBodyLockInterface = AsBodyLockInterface(lockInterface);

    ::new (outLock) JPH::BodyLockRead(*joltBodyLockInterface, JPH::BodyID(bodyID));
}

void JPH_BodyLockInterface_UnlockRead(const JPH_BodyLockInterface *lockInterface, JPH_BodyLockRead *ioLock)
{
    JPH_UNUSED(lockInterface);
    JPH_ASSERT(ioLock != nullptr);
    JPH_ASSERT(lockInterface != nullptr && lockInterface == ioLock->lockInterface); // NOLINT(*-simplify-boolean-expr)

    reinterpret_cast<const JPH::BodyLockRead *>(ioLock)->~BodyLockRead();
}

void JPH_BodyLockInterface_LockWrite(const JPH_BodyLockInterface *lockInterface,
                                     JPH_BodyID bodyID,
                                     JPH_BodyLockWrite *outLock)
{
    JPH_ASSERT(outLock != nullptr);
    const JPH::BodyLockInterface
            *joltBodyLockInterface = reinterpret_cast<const JPH::BodyLockInterface *>(lockInterface);

    ::new (outLock) JPH::BodyLockRead(*joltBodyLockInterface, JPH::BodyID(bodyID));
}

void JPH_BodyLockInterface_UnlockWrite(const JPH_BodyLockInterface *lockInterface, JPH_BodyLockWrite *ioLock)
{
    JPH_UNUSED(lockInterface);
    JPH_ASSERT(ioLock != nullptr);
    JPH_ASSERT(lockInterface != nullptr && lockInterface == ioLock->lockInterface); // NOLINT(*-simplify-boolean-expr)

    reinterpret_cast<const JPH::BodyLockWrite *>(ioLock)->~BodyLockWrite();
}

JPH_BodyLockMultiRead *JPH_BodyLockInterface_LockMultiRead(const JPH_BodyLockInterface *lockInterface,
                                                           const JPH_BodyID *bodyIDs,
                                                           const int count)
{
    const JPH::BodyLockInterface *const joltBodyLockInterface = AsBodyLockInterface(lockInterface);
    JPH::Array<JPH::BodyID> joltBodyIDs;

    for (uint32_t i = 0; i < count; ++i)
    {
        joltBodyIDs.push_back(JPH::BodyID(bodyIDs[i]));
    }

    JPH::BodyLockMultiRead *const joltLock = new JPH::BodyLockMultiRead(*joltBodyLockInterface,
                                                                        joltBodyIDs.data(),
                                                                        count);
    return reinterpret_cast<JPH_BodyLockMultiRead *>(joltLock);
}

void JPH_BodyLockMultiRead_Destroy(JPH_BodyLockMultiRead *ioLock)
{
    if (ioLock != nullptr)
    {
        delete reinterpret_cast<JPH::BodyLockMultiRead *>(ioLock);
    }
}

const JPH_Body *JPH_BodyLockMultiRead_GetBody(JPH_BodyLockMultiRead *ioLock, const int bodyIndex)
{
    const JPH::BodyLockMultiRead *const joltLock = reinterpret_cast<JPH::BodyLockMultiRead *>(ioLock);
    const JPH::Body *joltBody = joltLock->GetBody(bodyIndex);
    return reinterpret_cast<const JPH_Body *>(joltBody);
}

JPH_BodyLockMultiWrite *JPH_BodyLockInterface_LockMultiWrite(const JPH_BodyLockInterface *lockInterface,
                                                             const JPH_BodyID *bodyIDs,
                                                             const int count)
{
    const JPH::BodyLockInterface *const joltBodyLockInterface = AsBodyLockInterface(lockInterface);
    JPH::Array<JPH::BodyID> joltBodyIDs;

    for (uint32_t i = 0; i < count; ++i)
    {
        joltBodyIDs.push_back(JPH::BodyID(bodyIDs[i]));
    }

    JPH::BodyLockMultiWrite *const joltLock = new JPH::BodyLockMultiWrite(*joltBodyLockInterface,
                                                                          joltBodyIDs.data(),
                                                                          count);
    return reinterpret_cast<JPH_BodyLockMultiWrite *>(joltLock);
}

void JPH_BodyLockMultiWrite_Destroy(JPH_BodyLockMultiWrite *ioLock)
{
    if (ioLock != nullptr)
    {
        delete reinterpret_cast<JPH::BodyLockMultiWrite *>(ioLock);
    }
}

JPH_Body *JPH_BodyLockMultiWrite_GetBody(JPH_BodyLockMultiWrite *ioLock, const int bodyIndex)
{
    return reinterpret_cast<JPH_Body *>(reinterpret_cast<JPH::BodyLockMultiWrite *>(ioLock)->GetBody(bodyIndex));
}

//--------------------------------------------------------------------------------------------------
// JPH_CollideSettingsBase
//--------------------------------------------------------------------------------------------------
static inline void ToJolt(const JPH_CollideSettingsBase &settings, JPH::CollideSettingsBase *result)
{
    result->mActiveEdgeMode = static_cast<JPH::EActiveEdgeMode>(settings.activeEdgeMode);
    result->mCollectFacesMode = static_cast<JPH::ECollectFacesMode>(settings.collectFacesMode);
    result->mCollisionTolerance = settings.collisionTolerance;
    result->mPenetrationTolerance = settings.penetrationTolerance;
    result->mActiveEdgeMovementDirection = ToJolt(settings.activeEdgeMovementDirection);
}

void JPH_CollideSettingsBase_Init(const JPH::CollideSettingsBase &joltSettings, JPH_CollideSettingsBase *settings)
{
    // Copy defaults from jolt
    settings->activeEdgeMode = static_cast<JPH_ActiveEdgeMode>(joltSettings.mActiveEdgeMode);
    settings->collectFacesMode = static_cast<JPH_CollectFacesMode>(joltSettings.mCollectFacesMode);
    settings->collisionTolerance = joltSettings.mCollisionTolerance;
    settings->penetrationTolerance = joltSettings.mPenetrationTolerance;
    FromJolt(joltSettings.mActiveEdgeMovementDirection, &settings->activeEdgeMovementDirection);
}

//--------------------------------------------------------------------------------------------------
// JPH_CollideShapeSettings
//--------------------------------------------------------------------------------------------------
static inline JPH::CollideShapeSettings ToJolt(const JPH_CollideShapeSettings *settings)
{
    JPH::CollideShapeSettings result{};
    if (settings == nullptr)
    {
        result.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
        return result;
    }

    ToJolt(settings->base, &result);

    result.mMaxSeparationDistance = settings->maxSeparationDistance;
    result.mBackFaceMode = static_cast<JPH::EBackFaceMode>(settings->backFaceMode);
    return result;
}

static inline JPH_CollideShapeSettings FromJolt(const JPH::CollideShapeSettings &joltSettings)
{
    JPH_CollideShapeSettings result{};
    JPH_CollideSettingsBase_Init(joltSettings, &result.base);

    result.maxSeparationDistance = joltSettings.mMaxSeparationDistance;
    result.backFaceMode = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceMode);
    return result;
}

void JPH_CollideShapeSettings_Init(JPH_CollideShapeSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::CollideShapeSettings joltSettings;
    JPH_CollideSettingsBase_Init(joltSettings, &settings->base);

    settings->maxSeparationDistance = joltSettings.mMaxSeparationDistance;
    settings->backFaceMode = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceMode);
}

//--------------------------------------------------------------------------------------------------
// JPH_ShapeCastSettings
//--------------------------------------------------------------------------------------------------
static inline JPH::ShapeCastSettings ToJolt(const JPH_ShapeCastSettings *settings)
{
    JPH::ShapeCastSettings result{};
    if (settings == nullptr)
    {
        result.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
        result.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
        result.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;
        return result;
    }

    ToJolt(settings->base, &result);

    result.mBackFaceModeTriangles = static_cast<JPH::EBackFaceMode>(settings->backFaceModeTriangles);
    result.mBackFaceModeConvex = static_cast<JPH::EBackFaceMode>(settings->backFaceModeConvex);
    result.mUseShrunkenShapeAndConvexRadius = settings->useShrunkenShapeAndConvexRadius;
    result.mReturnDeepestPoint = settings->returnDeepestPoint;
    return result;
}

static inline JPH_ShapeCastSettings FromJolt(const JPH::ShapeCastSettings &joltSettings)
{
    JPH_ShapeCastSettings result{};
    JPH_CollideSettingsBase_Init(joltSettings, &result.base);

    result.backFaceModeTriangles = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeTriangles);
    result.backFaceModeConvex = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeConvex);
    result.useShrunkenShapeAndConvexRadius = joltSettings.mUseShrunkenShapeAndConvexRadius;
    result.returnDeepestPoint = joltSettings.mReturnDeepestPoint;
    return result;
}

void JPH_ShapeCastSettings_Init(JPH_ShapeCastSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::ShapeCastSettings joltSettings;
    JPH_CollideSettingsBase_Init(joltSettings, &settings->base);

    settings->backFaceModeTriangles = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeTriangles);
    settings->backFaceModeConvex = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceModeConvex);
    settings->useShrunkenShapeAndConvexRadius = joltSettings.mUseShrunkenShapeAndConvexRadius;
    settings->returnDeepestPoint = joltSettings.mReturnDeepestPoint;
}

//--------------------------------------------------------------------------------------------------
// JPH_BroadPhaseQuery
//--------------------------------------------------------------------------------------------------
class RayCastBodyCollectorCallback final: public JPH::RayCastBodyCollector
{
    public:
        RayCastBodyCollectorCallback(JPH_RayCastBodyCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::BroadPhaseCastResult &result) override
        {
            JPH_BroadPhaseCastResult hit;
            hit.bodyID = result.mBodyID.GetIndexAndSequenceNumber();
            hit.fraction = result.mFraction;

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_RayCastBodyCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

class CollideShapeBodyCollectorCallback final: public JPH::CollideShapeBodyCollector
{
    public:
        CollideShapeBodyCollectorCallback(JPH_CollideShapeBodyCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::BodyID &result) override
        {
            const float fraction = proc(userData, result.GetIndexAndSequenceNumber());

            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CollideShapeBodyCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

bool JPH_BroadPhaseQuery_CastRay(const JPH_BroadPhaseQuery *query,
                                 const JPH_Vec3 *origin,
                                 const JPH_Vec3 *direction,
                                 JPH_RayCastBodyCollectorCallback *callback,
                                 void *userData,
                                 JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                 JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query && origin && direction && callback);

    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    RayCastBodyCollectorCallback collector(callback, userData);
    AsBroadPhaseQuery(query)->CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));
    return collector.hadHit;
}

bool JPH_BroadPhaseQuery_CastRay2(const JPH_BroadPhaseQuery *query,
                                  const JPH_Vec3 *origin,
                                  const JPH_Vec3 *direction,
                                  const JPH_CollisionCollectorType collectorType,
                                  JPH_RayCastBodyResultCallback *callback,
                                  void *userData,
                                  JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                  JPH_ObjectLayerFilter *objectLayerFilter)
{
    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    JPH_BroadPhaseCastResult hitResult{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::RayCastBodyCollector> collector;
            AsBroadPhaseQuery(query)->CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::BroadPhaseCastResult &hit: collector.mHits)
                {
                    hitResult.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    hitResult.fraction = hit.mFraction;
                    callback(userData, &hitResult);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::RayCastBodyCollector> collector;
            AsBroadPhaseQuery(query)->CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::RayCastBodyCollector> collector;
            AsBroadPhaseQuery(query)->CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));

            if (collector.HadHit())
            {
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.fraction = collector.mHit.mFraction;
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_BroadPhaseQuery_CollideAABox(const JPH_BroadPhaseQuery *query,
                                      const JPH_AABox *box,
                                      JPH_CollideShapeBodyCollectorCallback *callback,
                                      void *userData,
                                      JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                      JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query && box && callback);

    const JPH::AABox joltBox(ToJolt(&box->min), ToJolt(&box->max));
    CollideShapeBodyCollectorCallback collector(callback, userData);
    AsBroadPhaseQuery(query)->CollideAABox(joltBox,
                                           collector,
                                           ToJolt(broadPhaseLayerFilter),
                                           ToJolt(objectLayerFilter));
    return collector.hadHit;
}

bool JPH_BroadPhaseQuery_CollideSphere(const JPH_BroadPhaseQuery *query,
                                       const JPH_Vec3 *center,
                                       float radius,
                                       JPH_CollideShapeBodyCollectorCallback *callback,
                                       void *userData,
                                       JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query && center && callback);

    CollideShapeBodyCollectorCallback collector(callback, userData);
    AsBroadPhaseQuery(query)->CollideSphere(ToJolt(center),
                                            radius,
                                            collector,
                                            ToJolt(broadPhaseLayerFilter),
                                            ToJolt(objectLayerFilter));
    return collector.hadHit;
}

bool JPH_BroadPhaseQuery_CollidePoint(const JPH_BroadPhaseQuery *query,
                                      const JPH_Vec3 *point,
                                      JPH_CollideShapeBodyCollectorCallback *callback,
                                      void *userData,
                                      JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                      JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query && point && callback);

    CollideShapeBodyCollectorCallback collector(callback, userData);
    AsBroadPhaseQuery(query)->CollidePoint(ToJolt(point),
                                           collector,
                                           ToJolt(broadPhaseLayerFilter),
                                           ToJolt(objectLayerFilter));
    return collector.hadHit;
}

//--------------------------------------------------------------------------------------------------
// JPH_NarrowPhaseQuery
//--------------------------------------------------------------------------------------------------
class CastRayCollectorCallback final: public JPH::CastRayCollector
{
    public:
        CastRayCollectorCallback(JPH_CastRayCollectorCallback *proc_, void *userData_): proc(proc_), userData(userData_)
        {}

        void AddHit(const JPH::RayCastResult &result) override
        {
            JPH_RayCastResult hit;
            hit.bodyID = result.mBodyID.GetIndexAndSequenceNumber();
            hit.fraction = result.mFraction;
            hit.subShapeID2 = result.mSubShapeID2.GetValue();

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CastRayCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

class CollidePointCollectorCallback final: public JPH::CollidePointCollector
{
    public:
        CollidePointCollectorCallback(JPH_CollidePointCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::CollidePointResult &result) override
        {
            JPH_CollidePointResult hit;
            hit.bodyID = result.mBodyID.GetIndexAndSequenceNumber();
            hit.subShapeID2 = result.mSubShapeID2.GetValue();

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CollidePointCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

class CollideShapeCollectorCallback final: public JPH::CollideShapeCollector
{
    public:
        CollideShapeCollectorCallback(JPH_CollideShapeCollectorCallback *proc, void *userData):
            proc(proc),
            userData(userData)
        {}

        void AddHit(const JPH::CollideShapeResult &result) override
        {
            const JPH_CollideShapeResult hit = FromJolt(result);

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CollideShapeCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

class CastShapeCollectorCallback final: public JPH::CastShapeCollector
{
    public:
        CastShapeCollectorCallback(JPH_CastShapeCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::ShapeCastResult &result) override
        {
            JPH_ShapeCastResult hit{};
            FromJolt(result.mContactPointOn1, &hit.contactPointOn1);
            FromJolt(result.mContactPointOn2, &hit.contactPointOn2);
            FromJolt(result.mPenetrationAxis, &hit.penetrationAxis);
            hit.penetrationDepth = result.mPenetrationDepth;
            hit.subShapeID1 = result.mSubShapeID1.GetValue();
            hit.subShapeID2 = result.mSubShapeID2.GetValue();
            hit.bodyID2 = result.mBodyID2.GetIndexAndSequenceNumber();
            hit.fraction = result.mFraction;
            hit.isBackFaceHit = result.mIsBackFaceHit;

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CastShapeCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

bool JPH_NarrowPhaseQuery_CastRay(const JPH_NarrowPhaseQuery *query,
                                  const JPH_RVec3 *origin,
                                  const JPH_Vec3 *direction,
                                  JPH_RayCastResult *hit,
                                  JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                  JPH_ObjectLayerFilter *objectLayerFilter,
                                  const JPH_BodyFilter *bodyFilter)
{
    JPH_ASSERT(query && origin && direction && hit);
    const JPH::NarrowPhaseQuery *joltQuery = reinterpret_cast<const JPH::NarrowPhaseQuery *>(query);

    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    JPH::RayCastResult result;

    const bool hadHit = joltQuery->CastRay(ray,
                                           result,
                                           ToJolt(broadPhaseLayerFilter),
                                           ToJolt(objectLayerFilter),
                                           ToJolt(bodyFilter));

    if (hadHit)
    {
        hit->fraction = result.mFraction;
        hit->bodyID = result.mBodyID.GetIndexAndSequenceNumber();
        hit->subShapeID2 = result.mSubShapeID2.GetValue();
    }

    return hadHit;
}

bool JPH_NarrowPhaseQuery_CastRay2(const JPH_NarrowPhaseQuery *query,
                                   const JPH_RVec3 *origin,
                                   const JPH_Vec3 *direction,
                                   const JPH_RayCastSettings *rayCastSettings,
                                   JPH_CastRayCollectorCallback *callback,
                                   void *userData,
                                   JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                   JPH_ObjectLayerFilter *objectLayerFilter,
                                   const JPH_BodyFilter *bodyFilter,
                                   const JPH_ShapeFilter *shapeFilter)
{
    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RayCastSettings raySettings = ToJolt(rayCastSettings);

    CastRayCollectorCallback collector(callback, userData);

    AsNarrowPhaseQuery(query)->CastRay(ray,
                                       raySettings,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CastRay3(const JPH_NarrowPhaseQuery *query,
                                   const JPH_RVec3 *origin,
                                   const JPH_Vec3 *direction,
                                   const JPH_RayCastSettings *rayCastSettings,
                                   const JPH_CollisionCollectorType collectorType,
                                   JPH_CastRayResultCallback *callback,
                                   void *userData,
                                   JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                   JPH_ObjectLayerFilter *objectLayerFilter,
                                   const JPH_BodyFilter *bodyFilter,
                                   const JPH_ShapeFilter *shapeFilter)
{
    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RayCastSettings raySettings = ToJolt(rayCastSettings);
    JPH_RayCastResult hitResult{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            AsNarrowPhaseQuery(query)->CastRay(ray,
                                               raySettings,
                                               collector,
                                               ToJolt(broadPhaseLayerFilter),
                                               ToJolt(objectLayerFilter),
                                               ToJolt(bodyFilter),
                                               ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::RayCastResult &hit: collector.mHits)
                {
                    hitResult.fraction = hit.mFraction;
                    hitResult.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    hitResult.subShapeID2 = hit.mSubShapeID2.GetValue();
                    callback(userData, &hitResult);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
            AsNarrowPhaseQuery(query)->CastRay(ray,
                                               raySettings,
                                               collector,
                                               ToJolt(broadPhaseLayerFilter),
                                               ToJolt(objectLayerFilter),
                                               ToJolt(bodyFilter),
                                               ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CastRayCollector> collector;
            AsNarrowPhaseQuery(query)->CastRay(ray,
                                               raySettings,
                                               collector,
                                               ToJolt(broadPhaseLayerFilter),
                                               ToJolt(objectLayerFilter),
                                               ToJolt(bodyFilter),
                                               ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_NarrowPhaseQuery_CollidePoint(const JPH_NarrowPhaseQuery *query,
                                       const JPH_RVec3 *point,
                                       JPH_CollidePointCollectorCallback *callback,
                                       void *userData,
                                       JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       JPH_ObjectLayerFilter *objectLayerFilter,
                                       const JPH_BodyFilter *bodyFilter,
                                       const JPH_ShapeFilter *shapeFilter)
{
    CollidePointCollectorCallback collector(callback, userData);
    AsNarrowPhaseQuery(query)->CollidePoint(ToJolt(point),
                                            collector,
                                            ToJolt(broadPhaseLayerFilter),
                                            ToJolt(objectLayerFilter),
                                            ToJolt(bodyFilter),
                                            ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CollidePoint2(const JPH_NarrowPhaseQuery *query,
                                        const JPH_RVec3 *point,
                                        const JPH_CollisionCollectorType collectorType,
                                        JPH_CollidePointResultCallback *callback,
                                        void *userData,
                                        JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                        JPH_ObjectLayerFilter *objectLayerFilter,
                                        const JPH_BodyFilter *bodyFilter,
                                        const JPH_ShapeFilter *shapeFilter)
{
    const JPH::Vec3 joltPoint = ToJolt(point);
    JPH_CollidePointResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsNarrowPhaseQuery(query)->CollidePoint(joltPoint,
                                                    collector,
                                                    ToJolt(broadPhaseLayerFilter),
                                                    ToJolt(objectLayerFilter),
                                                    ToJolt(bodyFilter),
                                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::CollidePointResult &hit: collector.mHits)
                {
                    result.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    result.subShapeID2 = hit.mSubShapeID2.GetValue();
                    callback(userData, &result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsNarrowPhaseQuery(query)->CollidePoint(joltPoint,
                                                    collector,
                                                    ToJolt(broadPhaseLayerFilter),
                                                    ToJolt(objectLayerFilter),
                                                    ToJolt(bodyFilter),
                                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsNarrowPhaseQuery(query)->CollidePoint(joltPoint,
                                                    collector,
                                                    ToJolt(broadPhaseLayerFilter),
                                                    ToJolt(objectLayerFilter),
                                                    ToJolt(bodyFilter),
                                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_NarrowPhaseQuery_CollideShape(const JPH_NarrowPhaseQuery *query,
                                       const JPH_Shape *shape,
                                       const JPH_Vec3 *scale,
                                       const JPH_RMatrix4x4 *centerOfMassTransform,
                                       const JPH_CollideShapeSettings *settings,
                                       JPH_RVec3 *baseOffset,
                                       JPH_CollideShapeCollectorCallback *callback,
                                       void *userData,
                                       JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       JPH_ObjectLayerFilter *objectLayerFilter,
                                       const JPH_BodyFilter *bodyFilter,
                                       const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query && shape && scale && centerOfMassTransform && callback);

    CollideShapeCollectorCallback collector(callback, userData);

    AsNarrowPhaseQuery(query)->CollideShape(AsShape(shape),
                                            ToJolt(scale),
                                            ToJolt(centerOfMassTransform),
                                            ToJolt(settings),
                                            ToJolt(baseOffset),
                                            collector,
                                            ToJolt(broadPhaseLayerFilter),
                                            ToJolt(objectLayerFilter),
                                            ToJolt(bodyFilter),
                                            ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CollideShape2(const JPH_NarrowPhaseQuery *query,
                                        const JPH_Shape *shape,
                                        const JPH_Vec3 *scale,
                                        const JPH_RMatrix4x4 *centerOfMassTransform,
                                        const JPH_CollideShapeSettings *settings,
                                        JPH_RVec3 *baseOffset,
                                        JPH_CollisionCollectorType collectorType,
                                        JPH_CollideShapeResultCallback *callback,
                                        void *userData,
                                        JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                        JPH_ObjectLayerFilter *objectLayerFilter,
                                        const JPH_BodyFilter *bodyFilter,
                                        const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query && shape && scale && centerOfMassTransform && callback);

    const JPH::Vec3 joltScale = ToJolt(scale);
    const JPH::Mat44 joltTransform = ToJolt(centerOfMassTransform);

    const JPH::CollideShapeSettings joltSettings = ToJolt(settings);
    const JPH::Vec3 joltBaseOffset = ToJolt(baseOffset);

    JPH_CollideShapeResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
            AsNarrowPhaseQuery(query)->CollideShape(AsShape(shape),
                                                    joltScale,
                                                    joltTransform,
                                                    joltSettings,
                                                    joltBaseOffset,
                                                    collector,
                                                    ToJolt(broadPhaseLayerFilter),
                                                    ToJolt(objectLayerFilter),
                                                    ToJolt(bodyFilter),
                                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::CollideShapeResult &hit: collector.mHits)
                {
                    result = FromJolt(hit);
                    callback(userData, &result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> collector;
            AsNarrowPhaseQuery(query)->CollideShape(AsShape(shape),
                                                    joltScale,
                                                    joltTransform,
                                                    joltSettings,
                                                    joltBaseOffset,
                                                    collector,
                                                    ToJolt(broadPhaseLayerFilter),
                                                    ToJolt(objectLayerFilter),
                                                    ToJolt(bodyFilter),
                                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result = FromJolt(collector.mHit);
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
            AsNarrowPhaseQuery(query)->CollideShape(AsShape(shape),
                                                    joltScale,
                                                    joltTransform,
                                                    joltSettings,
                                                    joltBaseOffset,
                                                    collector,
                                                    ToJolt(broadPhaseLayerFilter),
                                                    ToJolt(objectLayerFilter),
                                                    ToJolt(bodyFilter),
                                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result = FromJolt(collector.mHit);
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_NarrowPhaseQuery_CastShape(const JPH_NarrowPhaseQuery *query,
                                    const JPH_Shape *shape,
                                    const JPH_RMatrix4x4 *worldTransform,
                                    const JPH_Vec3 *direction,
                                    const JPH_ShapeCastSettings *settings,
                                    JPH_RVec3 *baseOffset,
                                    JPH_CastShapeCollectorCallback *callback,
                                    void *userData,
                                    JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                    JPH_ObjectLayerFilter *objectLayerFilter,
                                    const JPH_BodyFilter *bodyFilter,
                                    const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query && shape && worldTransform && direction && callback);

    const JPH::RShapeCast
            shapeCast = JPH::RShapeCast::sFromWorldTransform(AsShape(shape),
                                                             JPH::Vec3(1.f,
                                                                       1.f,
                                                                       1.f), // scale can be embedded in worldTransform
                                                             ToJolt(worldTransform),
                                                             ToJolt(direction));

    const JPH::ShapeCastSettings joltSettings = ToJolt(settings);

    const JPH::Vec3 joltBaseOffset = ToJolt(baseOffset);
    CastShapeCollectorCallback collector(callback, userData);

    AsNarrowPhaseQuery(query)->CastShape(shapeCast,
                                         joltSettings,
                                         joltBaseOffset,
                                         collector,
                                         ToJolt(broadPhaseLayerFilter),
                                         ToJolt(objectLayerFilter),
                                         ToJolt(bodyFilter),
                                         ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CastShape2(const JPH_NarrowPhaseQuery *query,
                                     const JPH_Shape *shape,
                                     const JPH_RMatrix4x4 *worldTransform,
                                     const JPH_Vec3 *direction,
                                     const JPH_ShapeCastSettings *settings,
                                     JPH_RVec3 *baseOffset,
                                     JPH_CollisionCollectorType collectorType,
                                     JPH_CastShapeResultCallback *callback,
                                     void *userData,
                                     JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                     JPH_ObjectLayerFilter *objectLayerFilter,
                                     const JPH_BodyFilter *bodyFilter,
                                     const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query && shape && worldTransform && direction && callback);

    const JPH::RShapeCast
            shapeCast = JPH::RShapeCast::sFromWorldTransform(AsShape(shape),
                                                             JPH::Vec3(1.f,
                                                                       1.f,
                                                                       1.f), // scale can be embedded in worldTransform
                                                             ToJolt(worldTransform),
                                                             ToJolt(direction));

    const JPH::ShapeCastSettings joltSettings = ToJolt(settings);

    const JPH::Vec3 joltBaseOffset = ToJolt(baseOffset);

    JPH_ShapeCastResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
            AsNarrowPhaseQuery(query)->CastShape(shapeCast,
                                                 joltSettings,
                                                 joltBaseOffset,
                                                 collector,
                                                 ToJolt(broadPhaseLayerFilter),
                                                 ToJolt(objectLayerFilter),
                                                 ToJolt(bodyFilter),
                                                 ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::ShapeCastResult &hit: collector.mHits)
                {
                    FromJolt(hit.mContactPointOn1, &result.contactPointOn1);
                    FromJolt(hit.mContactPointOn2, &result.contactPointOn2);
                    FromJolt(hit.mPenetrationAxis, &result.penetrationAxis);
                    result.penetrationDepth = hit.mPenetrationDepth;
                    result.subShapeID1 = hit.mSubShapeID1.GetValue();
                    result.subShapeID2 = hit.mSubShapeID2.GetValue();
                    result.bodyID2 = hit.mBodyID2.GetIndexAndSequenceNumber();
                    result.fraction = hit.mFraction;
                    result.isBackFaceHit = hit.mIsBackFaceHit;
                    callback(userData, &result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
            AsNarrowPhaseQuery(query)->CastShape(shapeCast,
                                                 joltSettings,
                                                 joltBaseOffset,
                                                 collector,
                                                 ToJolt(broadPhaseLayerFilter),
                                                 ToJolt(objectLayerFilter),
                                                 ToJolt(bodyFilter),
                                                 ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                FromJolt(collector.mHit.mContactPointOn1, &result.contactPointOn1);
                FromJolt(collector.mHit.mContactPointOn2, &result.contactPointOn2);
                FromJolt(collector.mHit.mPenetrationAxis, &result.penetrationAxis);
                result.penetrationDepth = collector.mHit.mPenetrationDepth;
                result.subShapeID1 = collector.mHit.mSubShapeID1.GetValue();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                result.bodyID2 = collector.mHit.mBodyID2.GetIndexAndSequenceNumber();
                result.fraction = collector.mHit.mFraction;
                result.isBackFaceHit = collector.mHit.mIsBackFaceHit;
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CastShapeCollector> collector;
            AsNarrowPhaseQuery(query)->CastShape(shapeCast,
                                                 joltSettings,
                                                 joltBaseOffset,
                                                 collector,
                                                 ToJolt(broadPhaseLayerFilter),
                                                 ToJolt(objectLayerFilter),
                                                 ToJolt(bodyFilter),
                                                 ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                FromJolt(collector.mHit.mContactPointOn1, &result.contactPointOn1);
                FromJolt(collector.mHit.mContactPointOn2, &result.contactPointOn2);
                FromJolt(collector.mHit.mPenetrationAxis, &result.penetrationAxis);
                result.penetrationDepth = collector.mHit.mPenetrationDepth;
                result.subShapeID1 = collector.mHit.mSubShapeID1.GetValue();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                result.bodyID2 = collector.mHit.mBodyID2.GetIndexAndSequenceNumber();
                result.fraction = collector.mHit.mFraction;
                result.isBackFaceHit = collector.mHit.mIsBackFaceHit;
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

/* Body */
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

void JPH_Body_GetLinearVelocity(JPH_Body *body, JPH_Vec3 *velocity)
{
    const JPH::Vec3 joltVector = reinterpret_cast<JPH::Body *>(body)->GetLinearVelocity();
    FromJolt(joltVector, velocity);
}

void JPH_Body_SetLinearVelocity(JPH_Body *body, const JPH_Vec3 *velocity)
{
    reinterpret_cast<JPH::Body *>(body)->SetLinearVelocity(ToJolt(velocity));
}

void JPH_Body_SetLinearVelocityClamped(JPH_Body *body, const JPH_Vec3 *velocity)
{
    reinterpret_cast<JPH::Body *>(body)->SetLinearVelocityClamped(ToJolt(velocity));
}

void JPH_Body_GetAngularVelocity(JPH_Body *body, JPH_Vec3 *velocity)
{
    const JPH::Vec3 joltVector = reinterpret_cast<JPH::Body *>(body)->GetAngularVelocity();
    FromJolt(joltVector, velocity);
}

void JPH_Body_SetAngularVelocity(JPH_Body *body, const JPH_Vec3 *velocity)
{
    AsBody(body)->SetAngularVelocity(ToJolt(velocity));
}

void JPH_Body_SetAngularVelocityClamped(JPH_Body *body, const JPH_Vec3 *velocity)
{
    AsBody(body)->SetAngularVelocityClamped(ToJolt(velocity));
}

void JPH_Body_GetPointVelocityCOM(JPH_Body *body, const JPH_Vec3 *pointRelativeToCOM, JPH_Vec3 *velocity)
{
    FromJolt(AsBody(body)->GetPointVelocityCOM(ToJolt(pointRelativeToCOM)), velocity);
}

void JPH_Body_GetPointVelocity(JPH_Body *body, const JPH_RVec3 *point, JPH_Vec3 *velocity)
{
    FromJolt(AsBody(body)->GetPointVelocity(ToJolt(point)), velocity);
}

void JPH_Body_AddForce(JPH_Body *body, const JPH_Vec3 *force)
{
    AsBody(body)->AddForce(ToJolt(force));
}

void JPH_Body_AddForceAtPosition(JPH_Body *body, const JPH_Vec3 *force, const JPH_RVec3 *position)
{
    AsBody(body)->AddForce(ToJolt(force), ToJolt(position));
}

void JPH_Body_AddTorque(JPH_Body *body, const JPH_Vec3 *force)
{
    AsBody(body)->AddTorque(ToJolt(force));
}

void JPH_Body_GetAccumulatedForce(JPH_Body *body, JPH_Vec3 *force)
{
    const JPH::Vec3 joltVector = AsBody(body)->GetAccumulatedForce();
    FromJolt(joltVector, force);
}

void JPH_Body_GetAccumulatedTorque(JPH_Body *body, JPH_Vec3 *force)
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

void JPH_Body_GetInverseInertia(JPH_Body *body, JPH_Matrix4x4 *result)
{
    FromJolt(AsBody(body)->GetInverseInertia(), result);
}

void JPH_Body_AddImpulse(JPH_Body *body, const JPH_Vec3 *impulse)
{
    AsBody(body)->AddImpulse(ToJolt(impulse));
}

void JPH_Body_AddImpulseAtPosition(JPH_Body *body, const JPH_Vec3 *impulse, const JPH_RVec3 *position)
{
    AsBody(body)->AddImpulse(ToJolt(impulse), ToJolt(position));
}

void JPH_Body_AddAngularImpulse(JPH_Body *body, const JPH_Vec3 *angularImpulse)
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
                                   const JPH_Vec3 *surfaceNormal,
                                   const float buoyancy,
                                   const float linearDrag,
                                   const float angularDrag,
                                   const JPH_Vec3 *fluidVelocity,
                                   const JPH_Vec3 *gravity,
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

void JPH_Body_GetWorldTransform(const JPH_Body *body, JPH_RMatrix4x4 *result)
{
    FromJolt(AsBody(body)->GetWorldTransform(), result);
}

void JPH_Body_GetCenterOfMassPosition(const JPH_Body *body, JPH_RVec3 *result)
{
    FromJolt(AsBody(body)->GetCenterOfMassPosition(), result);
}

void JPH_Body_GetCenterOfMassTransform(const JPH_Body *body, JPH_RMatrix4x4 *result)
{
    FromJolt(AsBody(body)->GetCenterOfMassTransform(), result);
}

void JPH_Body_GetInverseCenterOfMassTransform(const JPH_Body *body, JPH_RMatrix4x4 *result)
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
                                         JPH_Vec3 *normal)
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

uint64_t JPH_Body_GetUserData(JPH_Body *body)
{
    return AsBody(body)->GetUserData();
}

JPH_Body *JPH_Body_GetFixedToWorldBody()
{
    return ToBody(&JPH::Body::sFixedToWorld);
}

/* Contact Listener */
class ManagedContactListener final: public JPH::ContactListener
{
    public:
        static const JPH_ContactListener_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedContactListener(void *userData_): userData(userData_) {}

        JPH::ValidateResult OnContactValidate(const JPH::Body &inBody1,
                                              const JPH::Body &inBody2,
                                              JPH::RVec3Arg inBaseOffset,
                                              const JPH::CollideShapeResult &inCollisionResult) override
        {
            JPH_UNUSED(inCollisionResult);
            JPH_RVec3 baseOffset;
            FromJolt(inBaseOffset, &baseOffset);

            if (s_Procs != nullptr && s_Procs->OnContactValidate != nullptr)
            {
                const JPH_CollideShapeResult collideShapeResult = FromJolt(inCollisionResult);

                const JPH_ValidateResult
                        result = s_Procs->OnContactValidate(userData,
                                                            reinterpret_cast<const JPH_Body *>(&inBody1),
                                                            reinterpret_cast<const JPH_Body *>(&inBody2),
                                                            &baseOffset,
                                                            &collideShapeResult);

                return static_cast<JPH::ValidateResult>(result);
            }

            return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
        }

        void OnContactAdded(const JPH::Body &inBody1,
                            const JPH::Body &inBody2,
                            const JPH::ContactManifold &inManifold,
                            JPH::ContactSettings &ioSettings) override
        {
            JPH_UNUSED(inManifold);
            JPH_UNUSED(ioSettings);

            if (s_Procs != nullptr && s_Procs->OnContactAdded != nullptr)
            {
                s_Procs->OnContactAdded(userData,
                                        reinterpret_cast<const JPH_Body *>(&inBody1),
                                        reinterpret_cast<const JPH_Body *>(&inBody2),
                                        ToContactManifold(&inManifold),
                                        reinterpret_cast<JPH_ContactSettings *>(&ioSettings));
            }
        }

        void OnContactPersisted(const JPH::Body &inBody1,
                                const JPH::Body &inBody2,
                                const JPH::ContactManifold &inManifold,
                                JPH::ContactSettings &ioSettings) override
        {
            JPH_UNUSED(inManifold);
            JPH_UNUSED(ioSettings);

            if (s_Procs != nullptr && s_Procs->OnContactPersisted != nullptr)
            {
                s_Procs->OnContactPersisted(userData,
                                            reinterpret_cast<const JPH_Body *>(&inBody1),
                                            reinterpret_cast<const JPH_Body *>(&inBody2),
                                            ToContactManifold(&inManifold),
                                            reinterpret_cast<JPH_ContactSettings *>(&ioSettings));
            }
        }

        void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
        {
            if (s_Procs != nullptr && s_Procs->OnContactRemoved != nullptr)
            {
                s_Procs->OnContactRemoved(userData, reinterpret_cast<const JPH_SubShapeIDPair *>(&inSubShapePair));
            }
        }
};

const JPH_ContactListener_Procs *ManagedContactListener::s_Procs = nullptr;

void JPH_ContactListener_SetProcs(const JPH_ContactListener_Procs *procs)
{
    ManagedContactListener::s_Procs = procs;
}

JPH_ContactListener *JPH_ContactListener_Create(void *userData)
{
    ManagedContactListener *const listener = new ManagedContactListener(userData);
    return reinterpret_cast<JPH_ContactListener *>(listener);
}

void JPH_ContactListener_Destroy(JPH_ContactListener *listener)
{
    if (listener != nullptr)
    {
        delete reinterpret_cast<ManagedContactListener *>(listener);
    }
}

/* BodyActivationListener */
class ManagedBodyActivationListener final: public JPH::BodyActivationListener
{
    public:
        static const JPH_BodyActivationListener_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedBodyActivationListener(void *userData_): userData(userData_) {}

        void OnBodyActivated(const JPH::BodyID &inBodyID, const uint64_t inBodyUserData) override
        {
            if (s_Procs != nullptr && s_Procs->OnBodyDeactivated != nullptr)
            {
                s_Procs->OnBodyActivated(userData, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
            }
        }

        void OnBodyDeactivated(const JPH::BodyID &inBodyID, const uint64_t inBodyUserData) override
        {
            if (s_Procs != nullptr && s_Procs->OnBodyDeactivated != nullptr)
            {
                s_Procs->OnBodyDeactivated(userData, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
            }
        }
};

const JPH_BodyActivationListener_Procs *ManagedBodyActivationListener::s_Procs = nullptr;

void JPH_BodyActivationListener_SetProcs(const JPH_BodyActivationListener_Procs *procs)
{
    ManagedBodyActivationListener::s_Procs = procs;
}

JPH_BodyActivationListener *JPH_BodyActivationListener_Create(void *userData)
{
    ManagedBodyActivationListener *const listener = new ManagedBodyActivationListener(userData);
    return reinterpret_cast<JPH_BodyActivationListener *>(listener);
}

void JPH_BodyActivationListener_Destroy(JPH_BodyActivationListener *listener)
{
    if (listener != nullptr)
    {
        delete reinterpret_cast<ManagedBodyActivationListener *>(listener);
    }
}

/* ContactManifold */
void JPH_ContactManifold_GetWorldSpaceNormal(const JPH_ContactManifold *manifold, JPH_Vec3 *result)
{
    FromJolt(AsContactManifold(manifold)->mWorldSpaceNormal, result);
}

float JPH_ContactManifold_GetPenetrationDepth(const JPH_ContactManifold *manifold)
{
    return AsContactManifold(manifold)->mPenetrationDepth;
}

JPH_SubShapeID JPH_ContactManifold_GetSubShapeID1(const JPH_ContactManifold *manifold)
{
    return AsContactManifold(manifold)->mSubShapeID1.GetValue();
}

JPH_SubShapeID JPH_ContactManifold_GetSubShapeID2(const JPH_ContactManifold *manifold)
{
    return AsContactManifold(manifold)->mSubShapeID2.GetValue();
}

uint32_t JPH_ContactManifold_GetPointCount(const JPH_ContactManifold *manifold)
{
    return AsContactManifold(manifold)->mRelativeContactPointsOn1.size();
}

void JPH_ContactManifold_GetWorldSpaceContactPointOn1(const JPH_ContactManifold *manifold,
                                                      const uint32_t index,
                                                      JPH_RVec3 *result)
{
    FromJolt(AsContactManifold(manifold)->GetWorldSpaceContactPointOn1(index), result);
}

void JPH_ContactManifold_GetWorldSpaceContactPointOn2(const JPH_ContactManifold *manifold,
                                                      const uint32_t index,
                                                      JPH_RVec3 *result)
{
    FromJolt(AsContactManifold(manifold)->GetWorldSpaceContactPointOn2(index), result);
}

/* ContactSettings */
float JPH_ContactSettings_GetFriction(JPH_ContactSettings *settings)
{
    return reinterpret_cast<JPH::ContactSettings *>(settings)->mCombinedFriction;
}

void JPH_ContactSettings_SetFriction(JPH_ContactSettings *settings, const float friction)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mCombinedFriction = friction;
}

float JPH_ContactSettings_GetRestitution(JPH_ContactSettings *settings)
{
    return reinterpret_cast<JPH::ContactSettings *>(settings)->mCombinedRestitution;
}

void JPH_ContactSettings_SetRestitution(JPH_ContactSettings *settings, const float restitution)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mCombinedRestitution = restitution;
}

float JPH_ContactSettings_GetInvMassScale1(JPH_ContactSettings *settings)
{
    return reinterpret_cast<JPH::ContactSettings *>(settings)->mInvMassScale1;
}

void JPH_ContactSettings_SetInvMassScale1(JPH_ContactSettings *settings, const float scale)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mInvMassScale1 = scale;
}

float JPH_ContactSettings_GetInvInertiaScale1(JPH_ContactSettings *settings)
{
    return reinterpret_cast<JPH::ContactSettings *>(settings)->mInvInertiaScale1;
}

void JPH_ContactSettings_SetInvInertiaScale1(JPH_ContactSettings *settings, const float scale)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mInvInertiaScale1 = scale;
}

float JPH_ContactSettings_GetInvMassScale2(JPH_ContactSettings *settings)
{
    return reinterpret_cast<JPH::ContactSettings *>(settings)->mInvMassScale2;
}

void JPH_ContactSettings_SetInvMassScale2(JPH_ContactSettings *settings, const float scale)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mInvMassScale2 = scale;
}

float JPH_ContactSettings_GetInvInertiaScale2(JPH_ContactSettings *settings)
{
    return reinterpret_cast<JPH::ContactSettings *>(settings)->mInvInertiaScale2;
}

void JPH_ContactSettings_SetInvInertiaScale2(JPH_ContactSettings *settings, const float scale)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mInvInertiaScale2 = scale;
}

bool JPH_ContactSettings_GetIsSensor(const JPH_ContactSettings *settings)
{
    return reinterpret_cast<const JPH::ContactSettings *>(settings)->mIsSensor;
}

void JPH_ContactSettings_SetIsSensor(JPH_ContactSettings *settings, const bool sensor)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mIsSensor = sensor;
}

void JPH_ContactSettings_GetRelativeLinearSurfaceVelocity(JPH_ContactSettings *settings, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<JPH::ContactSettings *>(settings)->mRelativeLinearSurfaceVelocity, result);
}

void JPH_ContactSettings_SetRelativeLinearSurfaceVelocity(JPH_ContactSettings *settings, const JPH_Vec3 *velocity)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mRelativeLinearSurfaceVelocity = ToJolt(velocity);
}

void JPH_ContactSettings_GetRelativeAngularSurfaceVelocity(JPH_ContactSettings *settings, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<JPH::ContactSettings *>(settings)->mRelativeAngularSurfaceVelocity, result);
}

void JPH_ContactSettings_SetRelativeAngularSurfaceVelocity(JPH_ContactSettings *settings, const JPH_Vec3 *velocity)
{
    reinterpret_cast<JPH::ContactSettings *>(settings)->mRelativeAngularSurfaceVelocity = ToJolt(velocity);
}

/* CharacterBaseSettings */
void JPH_CharacterBaseSettings_Init(const JPH::CharacterBaseSettings &joltSettings, JPH_CharacterBaseSettings *settings)
{
    // Copy defaults from jolt
    FromJolt(joltSettings.mUp, &settings->up);
    FromJolt(joltSettings.mSupportingVolume, &settings->supportingVolume);
    settings->maxSlopeAngle = joltSettings.mMaxSlopeAngle;
    settings->enhancedInternalEdgeRemoval = joltSettings.mEnhancedInternalEdgeRemoval;
    if (joltSettings.mShape != nullptr)
    {
        settings->shape = ToShape(joltSettings.mShape.GetPtr());
    } else
    {
        constexpr JPH_Vec3 vec = {0.0f, 0.0f, 0.0f};
        const JPH_EmptyShapeSettings *empty_shape_settings = JPH_EmptyShapeSettings_Create(&vec);
        const JPH_EmptyShape *shape = JPH_EmptyShapeSettings_CreateShape(empty_shape_settings);
        settings->shape = reinterpret_cast<const JPH_Shape *>(shape);
    }
}

/* CharacterBase */
void JPH_CharacterBase_Destroy(JPH_CharacterBase *character)
{
    if (character != nullptr)
    {
        const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
        joltCharacter->Release();
    }
}

float JPH_CharacterBase_GetCosMaxSlopeAngle(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return joltCharacter->GetCosMaxSlopeAngle();
}

void JPH_CharacterBase_SetMaxSlopeAngle(JPH_CharacterBase *character, const float maxSlopeAngle)
{
    JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    joltCharacter->SetMaxSlopeAngle(maxSlopeAngle);
}

void JPH_CharacterBase_GetUp(JPH_CharacterBase *character, JPH_Vec3 *result)
{
    FromJolt(reinterpret_cast<JPH::CharacterBase *>(character)->GetUp(), result);
}

void JPH_CharacterBase_SetUp(JPH_CharacterBase *character, const JPH_Vec3 *value)
{
    JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    joltCharacter->SetUp(ToJolt(value));
}

bool JPH_CharacterBase_IsSlopeTooSteep(JPH_CharacterBase *character, const JPH_Vec3 *value)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return joltCharacter->IsSlopeTooSteep(ToJolt(value));
}

const JPH_Shape *JPH_CharacterBase_GetShape(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return ToShape(joltCharacter->GetShape());
}

JPH_GroundState JPH_CharacterBase_GetGroundState(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return static_cast<JPH_GroundState>(joltCharacter->GetGroundState());
}

bool JPH_CharacterBase_IsSupported(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return joltCharacter->IsSupported();
}

void JPH_CharacterBase_GetGroundPosition(JPH_CharacterBase *character, JPH_RVec3 *position)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    const JPH::RVec3 jolt_vector = joltCharacter->GetGroundPosition();
    FromJolt(jolt_vector, position);
}

void JPH_CharacterBase_GetGroundNormal(JPH_CharacterBase *character, JPH_Vec3 *normal)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    const JPH::Vec3 jolt_vector = joltCharacter->GetGroundNormal();
    FromJolt(jolt_vector, normal);
}

void JPH_CharacterBase_GetGroundVelocity(JPH_CharacterBase *character, JPH_Vec3 *velocity)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    const JPH::Vec3 jolt_vector = joltCharacter->GetGroundVelocity();
    FromJolt(jolt_vector, velocity);
}

const JPH_PhysicsMaterial *JPH_CharacterBase_GetGroundMaterial(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return reinterpret_cast<const JPH_PhysicsMaterial *>(joltCharacter->GetGroundMaterial());
}

JPH_BodyID JPH_CharacterBase_GetGroundBodyId(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return joltCharacter->GetGroundBodyID().GetIndexAndSequenceNumber();
}

JPH_SubShapeID JPH_CharacterBase_GetGroundSubShapeId(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return joltCharacter->GetGroundSubShapeID().GetValue();
}

uint64_t JPH_CharacterBase_GetGroundUserData(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return joltCharacter->GetGroundUserData();
}

/* CharacterSettings */
void JPH_CharacterSettings_Init(JPH_CharacterSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::CharacterSettings joltSettings;
    JPH_CharacterBaseSettings_Init(joltSettings, &settings->base);

    settings->layer = joltSettings.mLayer;
    settings->mass = joltSettings.mMass;
    settings->friction = joltSettings.mFriction;
    settings->gravityFactor = joltSettings.mGravityFactor;
    settings->allowedDOFs = static_cast<JPH_AllowedDOFs>(joltSettings.mAllowedDOFs);
}

/* Character */
JPH_Character *JPH_Character_Create(const JPH_CharacterSettings *settings,
                                    const JPH_RVec3 *position,
                                    const JPH_Quat *rotation,
                                    uint64_t userData,
                                    JPH_PhysicsSystem *system)
{
    JPH_ASSERT(settings);

    JPH::CharacterSettings joltSettings;

    // Copy base settings
    joltSettings.mUp = ToJolt(settings->base.up);
    joltSettings.mSupportingVolume = ToJolt(&settings->base.supportingVolume);
    joltSettings.mMaxSlopeAngle = settings->base.maxSlopeAngle;
    joltSettings.mEnhancedInternalEdgeRemoval = settings->base.enhancedInternalEdgeRemoval;
    if (settings->base.shape != nullptr)
    {
        const JPH::Shape *joltShape = reinterpret_cast<const JPH::Shape *>(settings->base.shape);
        joltSettings.mShape = joltShape;
    }

    joltSettings.mLayer = static_cast<JPH::ObjectLayer>(settings->layer);
    joltSettings.mMass = settings->mass;
    joltSettings.mFriction = settings->friction;
    joltSettings.mGravityFactor = settings->gravityFactor;
    joltSettings.mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(settings->allowedDOFs);

    JPH::Character *joltCharacter = new JPH::Character(&joltSettings,
                                                       ToJolt(position),
                                                       rotation != nullptr ? ToJolt(rotation) : JPH::Quat::sIdentity(),
                                                       userData,
                                                       system->physicsSystem);
    joltCharacter->AddRef();

    return reinterpret_cast<JPH_Character *>(joltCharacter);
}

void JPH_Character_AddToPhysicsSystem(JPH_Character *character, JPH_Activation activationMode, const bool lockBodies)
{
    AsCharacter(character)->AddToPhysicsSystem(static_cast<JPH::EActivation>(activationMode), lockBodies);
}

void JPH_Character_RemoveFromPhysicsSystem(JPH_Character *character, const bool lockBodies)
{
    AsCharacter(character)->RemoveFromPhysicsSystem(lockBodies);
}

void JPH_Character_Activate(JPH_Character *character, const bool lockBodies)
{
    AsCharacter(character)->Activate(lockBodies);
}

void JPH_Character_PostSimulation(JPH_Character *character, const float maxSeparationDistance, const bool lockBodies)
{
    AsCharacter(character)->PostSimulation(maxSeparationDistance, lockBodies);
}

void JPH_Character_SetLinearAndAngularVelocity(JPH_Character *character,
                                               const JPH_Vec3 *linearVelocity,
                                               const JPH_Vec3 *angularVelocity,
                                               const bool lockBodies)
{
    AsCharacter(character)->SetLinearAndAngularVelocity(ToJolt(linearVelocity), ToJolt(angularVelocity), lockBodies);
}

void JPH_Character_GetLinearVelocity(JPH_Character *character, JPH_Vec3 *result)
{
    FromJolt(AsCharacter(character)->GetLinearVelocity(), result);
}

void JPH_Character_SetLinearVelocity(JPH_Character *character, const JPH_Vec3 *value, const bool lockBodies)
{
    AsCharacter(character)->SetLinearVelocity(ToJolt(value), lockBodies);
}

void JPH_Character_AddLinearVelocity(JPH_Character *character, const JPH_Vec3 *value, const bool lockBodies)
{
    AsCharacter(character)->AddLinearVelocity(ToJolt(value), lockBodies);
}

void JPH_Character_AddImpulse(JPH_Character *character, const JPH_Vec3 *value, const bool lockBodies)
{
    AsCharacter(character)->AddImpulse(ToJolt(value), lockBodies);
}

JPH_BodyID JPH_Character_GetBodyID(const JPH_Character *character)
{
    return AsCharacter(character)->GetBodyID().GetIndexAndSequenceNumber();
}

void JPH_Character_GetPositionAndRotation(JPH_Character *character,
                                          JPH_RVec3 *position,
                                          JPH_Quat *rotation,
                                          const bool lockBodies)
{
    JPH::RVec3 joltPosition;
    JPH::Quat joltRotation;
    AsCharacter(character)->GetPositionAndRotation(joltPosition, joltRotation, lockBodies);
    FromJolt(joltPosition, position);
    FromJolt(joltRotation, rotation);
}

void JPH_Character_SetPositionAndRotation(JPH_Character *character,
                                          const JPH_RVec3 *position,
                                          const JPH_Quat *rotation,
                                          JPH_Activation activationMode,
                                          const bool lockBodies /* = true */)
{
    AsCharacter(character)->SetPositionAndRotation(ToJolt(position),
                                                   ToJolt(rotation),
                                                   static_cast<JPH::EActivation>(activationMode),
                                                   lockBodies);
}

void JPH_Character_GetPosition(JPH_Character *character, JPH_RVec3 *position, const bool lockBodies)
{
    FromJolt(AsCharacter(character)->GetPosition(lockBodies), position);
}

void JPH_Character_SetPosition(JPH_Character *character,
                               const JPH_RVec3 *position,
                               JPH_Activation activationMode,
                               const bool lockBodies)
{
    AsCharacter(character)->SetPosition(ToJolt(position), static_cast<JPH::EActivation>(activationMode), lockBodies);
}

void JPH_Character_SetShape(JPH_Character *character,
                            const JPH_Shape *shape,
                            const float maxPenetrationDepth,
                            const bool lockBodies)
{
    AsCharacter(character)->SetShape(AsShape(shape), maxPenetrationDepth, lockBodies);
}

void JPH_Character_GetRotation(JPH_Character *character, JPH_Quat *rotation, const bool lockBodies)
{
    FromJolt(AsCharacter(character)->GetRotation(lockBodies), rotation);
}

void JPH_Character_SetRotation(JPH_Character *character,
                               const JPH_Quat *rotation,
                               JPH_Activation activationMode,
                               const bool lockBodies)
{
    AsCharacter(character)->SetRotation(ToJolt(rotation), static_cast<JPH::EActivation>(activationMode), lockBodies);
}

void JPH_Character_GetCenterOfMassPosition(JPH_Character *character, JPH_RVec3 *result, const bool lockBodies)
{
    FromJolt(AsCharacter(character)->GetCenterOfMassPosition(lockBodies), result);
}

void JPH_Character_GetWorldTransform(JPH_Character *character, JPH_RMatrix4x4 *result, const bool lockBodies)
{
    FromJolt(AsCharacter(character)->GetWorldTransform(lockBodies), result);
}

JPH_ObjectLayer JPH_Character_GetLayer(const JPH_Character *character)
{
    return AsCharacter(character)->GetLayer();
}

void JPH_Character_SetLayer(JPH_Character *character, const JPH_ObjectLayer value, const bool lockBodies)
{
    AsCharacter(character)->SetLayer(value, lockBodies);
}

/* CharacterVirtualSettings */
void JPH_CharacterVirtualSettings_Init(JPH_CharacterVirtualSettings *settings)
{
    // Copy defaults from jolt
    const JPH::CharacterVirtualSettings joltSettings;
    JPH_CharacterBaseSettings_Init(joltSettings, &settings->base);

    settings->ID = joltSettings.mID.GetValue();
    settings->mass = joltSettings.mMass;
    settings->maxStrength = joltSettings.mMaxStrength;
    settings->shapeOffset = FromJolt(joltSettings.mShapeOffset);
    settings->backFaceMode = static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceMode);
    settings->predictiveContactDistance = joltSettings.mPredictiveContactDistance;
    settings->maxCollisionIterations = joltSettings.mMaxCollisionIterations;
    settings->maxConstraintIterations = joltSettings.mMaxConstraintIterations;
    settings->minTimeRemaining = joltSettings.mMinTimeRemaining;
    settings->collisionTolerance = joltSettings.mCollisionTolerance;
    settings->characterPadding = joltSettings.mCharacterPadding;
    settings->maxNumHits = joltSettings.mMaxNumHits;
    settings->hitReductionCosMaxAngle = joltSettings.mHitReductionCosMaxAngle;
    settings->penetrationRecoverySpeed = joltSettings.mPenetrationRecoverySpeed;
    if (joltSettings.mInnerBodyShape != nullptr)
    {
        settings->innerBodyShape = reinterpret_cast<const JPH_Shape *>(joltSettings.mInnerBodyShape.GetPtr());
    }
    settings->innerBodyIDOverride = joltSettings.mInnerBodyIDOverride.GetIndexAndSequenceNumber();
    settings->innerBodyLayer = joltSettings.mInnerBodyLayer;
}

/* CharacterVirtual */
JPH_CharacterVirtual *JPH_CharacterVirtual_Create(const JPH_CharacterVirtualSettings *settings,
                                                  const JPH_RVec3 *position,
                                                  const JPH_Quat *rotation,
                                                  uint64_t userData,
                                                  JPH_PhysicsSystem *system)
{
    JPH_ASSERT(settings);

    JPH::CharacterVirtualSettings joltSettings;

    // Copy base settings
    joltSettings.mUp = ToJolt(settings->base.up);
    joltSettings.mSupportingVolume = ToJolt(&settings->base.supportingVolume);
    joltSettings.mMaxSlopeAngle = settings->base.maxSlopeAngle;
    joltSettings.mEnhancedInternalEdgeRemoval = settings->base.enhancedInternalEdgeRemoval;
    if (settings->base.shape != nullptr)
    {
        const JPH::Shape *joltShape = reinterpret_cast<const JPH::Shape *>(settings->base.shape);
        joltSettings.mShape = joltShape;
    }

    if (settings->ID != 0u)
    {
        joltSettings.mID = JPH::CharacterID(settings->ID);
    }

    joltSettings.mMass = settings->mass;
    joltSettings.mMaxStrength = settings->maxStrength;
    joltSettings.mShapeOffset = ToJolt(settings->shapeOffset);
    joltSettings.mBackFaceMode = static_cast<JPH::EBackFaceMode>(settings->backFaceMode);
    joltSettings.mPredictiveContactDistance = settings->predictiveContactDistance;
    joltSettings.mMaxCollisionIterations = settings->maxCollisionIterations;
    joltSettings.mMaxConstraintIterations = settings->maxConstraintIterations;
    joltSettings.mMinTimeRemaining = settings->minTimeRemaining;
    joltSettings.mCollisionTolerance = settings->collisionTolerance;
    joltSettings.mCharacterPadding = settings->characterPadding;
    joltSettings.mMaxNumHits = settings->maxNumHits;
    joltSettings.mHitReductionCosMaxAngle = settings->hitReductionCosMaxAngle;
    joltSettings.mPenetrationRecoverySpeed = settings->penetrationRecoverySpeed;
    if (settings->innerBodyShape != nullptr)
    {
        const JPH::Shape *joltShape = reinterpret_cast<const JPH::Shape *>(settings->innerBodyShape);
        joltSettings.mInnerBodyShape = joltShape;
    }

    if (settings->innerBodyIDOverride != 0u)
    {
        joltSettings.mInnerBodyIDOverride = JPH::BodyID(settings->innerBodyIDOverride);
    }

    joltSettings.mInnerBodyLayer = static_cast<JPH::ObjectLayer>(settings->innerBodyLayer);

    JPH::CharacterVirtual *joltCharacter = new JPH::CharacterVirtual(&joltSettings,
                                                                     ToJolt(position),
                                                                     rotation != nullptr ? ToJolt(rotation)
                                                                                         : JPH::Quat::sIdentity(),
                                                                     userData,
                                                                     system->physicsSystem);
    joltCharacter->AddRef();

    return ToCharacterVirtual(joltCharacter);
}

JPH_CharacterID JPH_CharacterVirtual_GetID(const JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetID().GetValue();
}

void JPH_CharacterVirtual_SetListener(JPH_CharacterVirtual *character, JPH_CharacterContactListener *listener)
{
    if (listener != nullptr)
    {
        JPH::CharacterContactListener *const joltListener = reinterpret_cast<JPH::CharacterContactListener *>(listener);
        AsCharacterVirtual(character)->SetListener(joltListener);
    } else
    {
        AsCharacterVirtual(character)->SetListener(nullptr);
    }
}

void JPH_CharacterVirtual_SetCharacterVsCharacterCollision(JPH_CharacterVirtual *character,
                                                           JPH_CharacterVsCharacterCollision
                                                                   *characterVsCharacterCollision)
{
    if (characterVsCharacterCollision != nullptr)
    {
        JPH::CharacterVsCharacterCollision *const joltCharacterVsCharacterCollision = reinterpret_cast<
                JPH::CharacterVsCharacterCollision *>(characterVsCharacterCollision);
        AsCharacterVirtual(character)->SetCharacterVsCharacterCollision(joltCharacterVsCharacterCollision);
    } else
    {
        AsCharacterVirtual(character)->SetCharacterVsCharacterCollision(nullptr);
    }
}

void JPH_CharacterVirtual_GetLinearVelocity(JPH_CharacterVirtual *character, JPH_Vec3 *velocity)
{
    FromJolt(AsCharacterVirtual(character)->GetLinearVelocity(), velocity);
}

void JPH_CharacterVirtual_SetLinearVelocity(JPH_CharacterVirtual *character, const JPH_Vec3 *velocity)
{
    AsCharacterVirtual(character)->SetLinearVelocity(ToJolt(velocity));
}

void JPH_CharacterVirtual_GetPosition(JPH_CharacterVirtual *character, JPH_RVec3 *position)
{
    FromJolt(AsCharacterVirtual(character)->GetPosition(), position);
}

void JPH_CharacterVirtual_SetPosition(JPH_CharacterVirtual *character, const JPH_RVec3 *position)
{
    AsCharacterVirtual(character)->SetPosition(ToJolt(position));
}

void JPH_CharacterVirtual_GetRotation(JPH_CharacterVirtual *character, JPH_Quat *rotation)
{
    const JPH::Quat jolt_quat = AsCharacterVirtual(character)->GetRotation();
    FromJolt(jolt_quat, rotation);
}

void JPH_CharacterVirtual_SetRotation(JPH_CharacterVirtual *character, const JPH_Quat *rotation)
{
    AsCharacterVirtual(character)->SetRotation(ToJolt(rotation));
}

void JPH_CharacterVirtual_GetWorldTransform(JPH_CharacterVirtual *character, JPH_RMatrix4x4 *result)
{
    const JPH::RMat44 &mat = AsCharacterVirtual(character)->GetWorldTransform();
    FromJolt(mat, result);
}

void JPH_CharacterVirtual_GetCenterOfMassTransform(JPH_CharacterVirtual *character, JPH_RMatrix4x4 *result)
{
    const JPH::RMat44 &mat = AsCharacterVirtual(character)->GetCenterOfMassTransform();
    FromJolt(mat, result);
}

float JPH_CharacterVirtual_GetMass(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetMass();
}

void JPH_CharacterVirtual_SetMass(JPH_CharacterVirtual *character, const float value)
{
    AsCharacterVirtual(character)->SetMass(value);
}

float JPH_CharacterVirtual_GetMaxStrength(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetMaxStrength();
}

void JPH_CharacterVirtual_SetMaxStrength(JPH_CharacterVirtual *character, const float value)
{
    AsCharacterVirtual(character)->SetMaxStrength(value);
}

float JPH_CharacterVirtual_GetPenetrationRecoverySpeed(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetPenetrationRecoverySpeed();
}

void JPH_CharacterVirtual_SetPenetrationRecoverySpeed(JPH_CharacterVirtual *character, const float value)
{
    AsCharacterVirtual(character)->SetPenetrationRecoverySpeed(value);
}

bool JPH_CharacterVirtual_GetEnhancedInternalEdgeRemoval(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetEnhancedInternalEdgeRemoval();
}

void JPH_CharacterVirtual_SetEnhancedInternalEdgeRemoval(JPH_CharacterVirtual *character, const bool value)
{
    AsCharacterVirtual(character)->SetEnhancedInternalEdgeRemoval(value);
}

float JPH_CharacterVirtual_GetCharacterPadding(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetCharacterPadding();
}

uint32_t JPH_CharacterVirtual_GetMaxNumHits(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetMaxNumHits();
}

void JPH_CharacterVirtual_SetMaxNumHits(JPH_CharacterVirtual *character, const uint32_t value)
{
    AsCharacterVirtual(character)->SetMaxNumHits(value);
}

float JPH_CharacterVirtual_GetHitReductionCosMaxAngle(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetHitReductionCosMaxAngle();
}

void JPH_CharacterVirtual_SetHitReductionCosMaxAngle(JPH_CharacterVirtual *character, const float value)
{
    AsCharacterVirtual(character)->SetHitReductionCosMaxAngle(value);
}

bool JPH_CharacterVirtual_GetMaxHitsExceeded(JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetMaxHitsExceeded();
}

void JPH_CharacterVirtual_GetShapeOffset(JPH_CharacterVirtual *character, JPH_Vec3 *result)
{
    const JPH::Vec3 joltVector = AsCharacterVirtual(character)->GetShapeOffset();
    FromJolt(joltVector, result);
}

void JPH_CharacterVirtual_SetShapeOffset(JPH_CharacterVirtual *character, const JPH_Vec3 *value)
{
    AsCharacterVirtual(character)->SetShapeOffset(ToJolt(value));
}

uint64_t JPH_CharacterVirtual_GetUserData(const JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetUserData();
}

void JPH_CharacterVirtual_SetUserData(JPH_CharacterVirtual *character, const uint64_t value)
{
    AsCharacterVirtual(character)->SetUserData(value);
}

JPH_BodyID JPH_CharacterVirtual_GetInnerBodyID(const JPH_CharacterVirtual *character)
{
    return AsCharacterVirtual(character)->GetInnerBodyID().GetIndexAndSequenceNumber();
}

void JPH_CharacterVirtual_CancelVelocityTowardsSteepSlopes(JPH_CharacterVirtual *character,
                                                           const JPH_Vec3 *desiredVelocity,
                                                           JPH_Vec3 *velocity)
{
    FromJolt(AsCharacterVirtual(character)->CancelVelocityTowardsSteepSlopes(ToJolt(desiredVelocity)), velocity);
}

void JPH_CharacterVirtual_StartTrackingContactChanges(JPH_CharacterVirtual *character)
{
    AsCharacterVirtual(character)->StartTrackingContactChanges();
}

void JPH_CharacterVirtual_FinishTrackingContactChanges(JPH_CharacterVirtual *character)
{
    AsCharacterVirtual(character)->FinishTrackingContactChanges();
}

void JPH_CharacterVirtual_Update(JPH_CharacterVirtual *character,
                                 const float deltaTime,
                                 const JPH_ObjectLayer layer,
                                 const JPH_PhysicsSystem *system,
                                 const JPH_BodyFilter *bodyFilter,
                                 const JPH_ShapeFilter *shapeFilter)
{
    const JPH_ObjectLayer joltLayer = layer;

    AsCharacterVirtual(character)->Update(deltaTime,
                                          system->physicsSystem->GetGravity(),
                                          system->physicsSystem->GetDefaultBroadPhaseLayerFilter(joltLayer),
                                          system->physicsSystem->GetDefaultLayerFilter(joltLayer),
                                          ToJolt(bodyFilter),
                                          ToJolt(shapeFilter),
                                          *s_TempAllocator);
}

void JPH_CharacterVirtual_ExtendedUpdate(JPH_CharacterVirtual *character,
                                         float deltaTime,
                                         const JPH_ExtendedUpdateSettings *settings,
                                         JPH_ObjectLayer layer,
                                         JPH_PhysicsSystem *system,
                                         const JPH_BodyFilter *bodyFilter,
                                         const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(settings);

    // Convert to Jolt
    JPH::CharacterVirtual::ExtendedUpdateSettings joltSettings = {};
    joltSettings.mStickToFloorStepDown = ToJolt(&settings->stickToFloorStepDown);
    joltSettings.mWalkStairsStepUp = ToJolt(&settings->walkStairsStepUp);
    joltSettings.mWalkStairsMinStepForward = settings->walkStairsMinStepForward;
    joltSettings.mWalkStairsStepForwardTest = settings->walkStairsStepForwardTest;
    joltSettings.mWalkStairsCosAngleForwardContact = settings->walkStairsCosAngleForwardContact;
    joltSettings.mWalkStairsStepDownExtra = ToJolt(&settings->walkStairsStepDownExtra);

    AsCharacterVirtual(character)->ExtendedUpdate(deltaTime,
                                                  system->physicsSystem->GetGravity(),
                                                  joltSettings,
                                                  system->physicsSystem->GetDefaultBroadPhaseLayerFilter(layer),
                                                  system->physicsSystem->GetDefaultLayerFilter(layer),
                                                  ToJolt(bodyFilter),
                                                  ToJolt(shapeFilter),
                                                  *s_TempAllocator);
}

void JPH_CharacterVirtual_RefreshContacts(JPH_CharacterVirtual *character,
                                          const JPH_ObjectLayer layer,
                                          const JPH_PhysicsSystem *system,
                                          const JPH_BodyFilter *bodyFilter,
                                          const JPH_ShapeFilter *shapeFilter)
{
    AsCharacterVirtual(character)->RefreshContacts(system->physicsSystem->GetDefaultBroadPhaseLayerFilter(layer),
                                                   system->physicsSystem->GetDefaultLayerFilter(layer),
                                                   ToJolt(bodyFilter),
                                                   ToJolt(shapeFilter),
                                                   *s_TempAllocator);
}

bool JPH_CharacterVirtual_CanWalkStairs(JPH_CharacterVirtual *character, const JPH_Vec3 *linearVelocity)
{
    return AsCharacterVirtual(character)->CanWalkStairs(ToJolt(linearVelocity));
}

bool JPH_CharacterVirtual_WalkStairs(JPH_CharacterVirtual *character,
                                     const float deltaTime,
                                     const JPH_Vec3 *stepUp,
                                     const JPH_Vec3 *stepForward,
                                     const JPH_Vec3 *stepForwardTest,
                                     const JPH_Vec3 *stepDownExtra,
                                     const JPH_ObjectLayer layer,
                                     const JPH_PhysicsSystem *system,
                                     const JPH_BodyFilter *bodyFilter,
                                     const JPH_ShapeFilter *shapeFilter)
{
    return AsCharacterVirtual(character)->WalkStairs(deltaTime,
                                                     ToJolt(stepUp),
                                                     ToJolt(stepForward),
                                                     ToJolt(stepForwardTest),
                                                     ToJolt(stepDownExtra),
                                                     system->physicsSystem->GetDefaultBroadPhaseLayerFilter(layer),
                                                     system->physicsSystem->GetDefaultLayerFilter(layer),
                                                     ToJolt(bodyFilter),
                                                     ToJolt(shapeFilter),
                                                     *s_TempAllocator);
}

bool JPH_CharacterVirtual_StickToFloor(JPH_CharacterVirtual *character,
                                       const JPH_Vec3 *stepDown,
                                       const JPH_ObjectLayer layer,
                                       const JPH_PhysicsSystem *system,
                                       const JPH_BodyFilter *bodyFilter,
                                       const JPH_ShapeFilter *shapeFilter)
{
    return AsCharacterVirtual(character)->StickToFloor(ToJolt(stepDown),
                                                       system->physicsSystem->GetDefaultBroadPhaseLayerFilter(layer),
                                                       system->physicsSystem->GetDefaultLayerFilter(layer),
                                                       ToJolt(bodyFilter),
                                                       ToJolt(shapeFilter),
                                                       *s_TempAllocator);
}

void JPH_CharacterVirtual_UpdateGroundVelocity(JPH_CharacterVirtual *character)
{
    AsCharacterVirtual(character)->UpdateGroundVelocity();
}

bool JPH_CharacterVirtual_SetShape(JPH_CharacterVirtual *character,
                                   const JPH_Shape *shape,
                                   const float maxPenetrationDepth,
                                   const JPH_ObjectLayer layer,
                                   const JPH_PhysicsSystem *system,
                                   const JPH_BodyFilter *bodyFilter,
                                   const JPH_ShapeFilter *shapeFilter)
{
    return AsCharacterVirtual(character)->SetShape(AsShape(shape),
                                                   maxPenetrationDepth,
                                                   system->physicsSystem->GetDefaultBroadPhaseLayerFilter(layer),
                                                   system->physicsSystem->GetDefaultLayerFilter(layer),
                                                   ToJolt(bodyFilter),
                                                   ToJolt(shapeFilter),
                                                   *s_TempAllocator);
}

void JPH_CharacterVirtual_SetInnerBodyShape(JPH_CharacterVirtual *character, const JPH_Shape *shape)
{
    AsCharacterVirtual(character)->SetInnerBodyShape(AsShape(shape));
}

uint32_t JPH_CharacterVirtual_GetNumActiveContacts(JPH_CharacterVirtual *character)
{
    return static_cast<uint32_t>(AsCharacterVirtual(character)->GetActiveContacts().size());
}

void JPH_CharacterVirtual_GetActiveContact(JPH_CharacterVirtual *character,
                                           const uint32_t index,
                                           JPH_CharacterVirtualContact *result)
{
    FromJolt(AsCharacterVirtual(character)->GetActiveContacts().at(index), result);
}

bool JPH_CharacterVirtual_HasCollidedWithBody(JPH_CharacterVirtual *character, const JPH_BodyID body)
{
    return AsCharacterVirtual(character)->HasCollidedWith(JPH::BodyID(body));
}

bool JPH_CharacterVirtual_HasCollidedWith(JPH_CharacterVirtual *character, const JPH_CharacterID other)
{
    return AsCharacterVirtual(character)->HasCollidedWith(JPH::CharacterID(other));
}

bool JPH_CharacterVirtual_HasCollidedWithCharacter(JPH_CharacterVirtual *character, const JPH_CharacterVirtual *other)
{
    return AsCharacterVirtual(character)->HasCollidedWith(AsCharacterVirtual(other));
}

/* CharacterContactListener */
class ManagedCharacterContactListener final: public JPH::CharacterContactListener
{
    public:
        static const JPH_CharacterContactListener_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedCharacterContactListener(void *userData_): userData(userData_) {}

        void OnAdjustBodyVelocity(const JPH::CharacterVirtual *inCharacter,
                                  const JPH::Body &inBody2,
                                  JPH::Vec3 &ioLinearVelocity,
                                  JPH::Vec3 &ioAngularVelocity) override
        {
            JPH_Vec3 linearVelocity;
            JPH_Vec3 angularVelocity;
            FromJolt(ioLinearVelocity, &linearVelocity);
            FromJolt(ioAngularVelocity, &angularVelocity);

            if (s_Procs != nullptr && s_Procs->OnAdjustBodyVelocity != nullptr)
            {
                s_Procs->OnAdjustBodyVelocity(userData,
                                              ToCharacterVirtual(inCharacter),
                                              ToBody(&inBody2),
                                              &linearVelocity,
                                              &angularVelocity);

                ioLinearVelocity = ToJolt(linearVelocity);
                ioAngularVelocity = ToJolt(angularVelocity);
            }
        }

        bool OnContactValidate(const JPH::CharacterVirtual *inCharacter,
                               const JPH::BodyID &inBodyID2,
                               const JPH::SubShapeID &inSubShapeID2) override
        {
            if (s_Procs != nullptr && s_Procs->OnContactValidate != nullptr)
            {
                return s_Procs->OnContactValidate(userData,
                                                  ToCharacterVirtual(inCharacter),
                                                  inBodyID2.GetIndexAndSequenceNumber(),
                                                  inSubShapeID2.GetValue());
            }

            return true;
        }

        bool OnCharacterContactValidate(const JPH::CharacterVirtual *inCharacter,
                                        const JPH::CharacterVirtual *inOtherCharacter,
                                        const JPH::SubShapeID &inSubShapeID2) override
        {
            if (s_Procs != nullptr && s_Procs->OnCharacterContactValidate != nullptr)
            {
                return s_Procs->OnCharacterContactValidate(userData,
                                                           ToCharacterVirtual(inCharacter),
                                                           ToCharacterVirtual(inOtherCharacter),
                                                           inSubShapeID2.GetValue());
            }

            return true;
        }

        void OnContactAdded(const JPH::CharacterVirtual *inCharacter,
                            const JPH::BodyID &inBodyID2,
                            const JPH::SubShapeID &inSubShapeID2,
                            JPH::RVec3Arg inContactPosition,
                            JPH::Vec3Arg inContactNormal,
                            JPH::CharacterContactSettings &ioSettings) override
        {
            if (s_Procs != nullptr && s_Procs->OnContactAdded != nullptr)
            {
                JPH_RVec3 contactPosition;
                JPH_Vec3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                s_Procs->OnContactAdded(userData,
                                        ToCharacterVirtual(inCharacter),
                                        inBodyID2.GetIndexAndSequenceNumber(),
                                        inSubShapeID2.GetValue(),
                                        &contactPosition,
                                        &contactNormal,
                                        &settings);

                ioSettings.mCanPushCharacter = settings.canPushCharacter;
                ioSettings.mCanReceiveImpulses = settings.canReceiveImpulses;
            }
        }

        void OnContactPersisted(const JPH::CharacterVirtual *inCharacter,
                                const JPH::BodyID &inBodyID2,
                                const JPH::SubShapeID &inSubShapeID2,
                                JPH::RVec3Arg inContactPosition,
                                JPH::Vec3Arg inContactNormal,
                                JPH::CharacterContactSettings &ioSettings) override
        {
            if (s_Procs != nullptr && s_Procs->OnContactPersisted != nullptr)
            {
                JPH_RVec3 contactPosition;
                JPH_Vec3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                s_Procs->OnContactPersisted(userData,
                                            ToCharacterVirtual(inCharacter),
                                            inBodyID2.GetIndexAndSequenceNumber(),
                                            inSubShapeID2.GetValue(),
                                            &contactPosition,
                                            &contactNormal,
                                            &settings);

                ioSettings.mCanPushCharacter = settings.canPushCharacter;
                ioSettings.mCanReceiveImpulses = settings.canReceiveImpulses;
            }
        }

        void OnContactRemoved(const JPH::CharacterVirtual *inCharacter,
                              const JPH::BodyID &inBodyID2,
                              const JPH::SubShapeID &inSubShapeID2) override
        {
            if (s_Procs != nullptr && s_Procs->OnContactRemoved != nullptr)
            {
                s_Procs->OnContactRemoved(userData,
                                          ToCharacterVirtual(inCharacter),
                                          inBodyID2.GetIndexAndSequenceNumber(),
                                          inSubShapeID2.GetValue());
            }
        }

        void OnCharacterContactAdded(const JPH::CharacterVirtual *inCharacter,
                                     const JPH::CharacterVirtual *inOtherCharacter,
                                     const JPH::SubShapeID &inSubShapeID2,
                                     JPH::RVec3Arg inContactPosition,
                                     JPH::Vec3Arg inContactNormal,
                                     JPH::CharacterContactSettings &ioSettings) override
        {
            if (s_Procs != nullptr && s_Procs->OnCharacterContactAdded != nullptr)
            {
                JPH_RVec3 contactPosition;
                JPH_Vec3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                s_Procs->OnCharacterContactAdded(userData,
                                                 ToCharacterVirtual(inCharacter),
                                                 ToCharacterVirtual(inOtherCharacter),
                                                 inSubShapeID2.GetValue(),
                                                 &contactPosition,
                                                 &contactNormal,
                                                 &settings);

                ioSettings.mCanPushCharacter = settings.canPushCharacter;
                ioSettings.mCanReceiveImpulses = settings.canReceiveImpulses;
            }
        }

        void OnCharacterContactPersisted(const JPH::CharacterVirtual *inCharacter,
                                         const JPH::CharacterVirtual *inOtherCharacter,
                                         const JPH::SubShapeID &inSubShapeID2,
                                         JPH::RVec3Arg inContactPosition,
                                         JPH::Vec3Arg inContactNormal,
                                         JPH::CharacterContactSettings &ioSettings) override
        {
            if (s_Procs != nullptr && s_Procs->OnCharacterContactPersisted != nullptr)
            {
                JPH_RVec3 contactPosition;
                JPH_Vec3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                s_Procs->OnCharacterContactPersisted(userData,
                                                     ToCharacterVirtual(inCharacter),
                                                     ToCharacterVirtual(inOtherCharacter),
                                                     inSubShapeID2.GetValue(),
                                                     &contactPosition,
                                                     &contactNormal,
                                                     &settings);

                ioSettings.mCanPushCharacter = settings.canPushCharacter;
                ioSettings.mCanReceiveImpulses = settings.canReceiveImpulses;
            }
        }

        void OnCharacterContactRemoved(const JPH::CharacterVirtual *inCharacter,
                                       const JPH::CharacterID &inOtherCharacterID,
                                       const JPH::SubShapeID &inSubShapeID2) override
        {
            if (s_Procs != nullptr && s_Procs->OnCharacterContactRemoved != nullptr)
            {
                s_Procs->OnCharacterContactRemoved(userData,
                                                   ToCharacterVirtual(inCharacter),
                                                   inOtherCharacterID.GetValue(),
                                                   inSubShapeID2.GetValue());
            }
        }

        void OnContactSolve(const JPH::CharacterVirtual *inCharacter,
                            const JPH::BodyID &inBodyID2,
                            const JPH::SubShapeID &inSubShapeID2,
                            JPH::RVec3Arg inContactPosition,
                            JPH::Vec3Arg inContactNormal,
                            JPH::Vec3Arg inContactVelocity,
                            const JPH::PhysicsMaterial *inContactMaterial,
                            JPH::Vec3Arg inCharacterVelocity,
                            JPH::Vec3 &ioNewCharacterVelocity) override
        {
            if (s_Procs != nullptr && s_Procs->OnContactSolve != nullptr)
            {
                JPH_RVec3 contactPosition;
                JPH_Vec3 contactNormal;
                JPH_Vec3 contactVelocity;
                JPH_Vec3 characterVelocity;
                JPH_Vec3 newCharacterVelocity;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);
                FromJolt(inContactVelocity, &contactVelocity);
                FromJolt(inCharacterVelocity, &characterVelocity);
                FromJolt(ioNewCharacterVelocity, &newCharacterVelocity);

                s_Procs->OnContactSolve(userData,
                                        ToCharacterVirtual(inCharacter),
                                        inBodyID2.GetIndexAndSequenceNumber(),
                                        inSubShapeID2.GetValue(),
                                        &contactPosition,
                                        &contactNormal,
                                        &contactVelocity,
                                        reinterpret_cast<const JPH_PhysicsMaterial *>(inContactMaterial),
                                        &characterVelocity,
                                        &newCharacterVelocity);

                ioNewCharacterVelocity = ToJolt(newCharacterVelocity);
            }
        }

        void OnCharacterContactSolve(const JPH::CharacterVirtual *inCharacter,
                                     const JPH::CharacterVirtual *inOtherCharacter,
                                     const JPH::SubShapeID &inSubShapeID2,
                                     JPH::RVec3Arg inContactPosition,
                                     JPH::Vec3Arg inContactNormal,
                                     JPH::Vec3Arg inContactVelocity,
                                     const JPH::PhysicsMaterial *inContactMaterial,
                                     JPH::Vec3Arg inCharacterVelocity,
                                     JPH::Vec3 &ioNewCharacterVelocity) override
        {
            if (s_Procs != nullptr && s_Procs->OnCharacterContactSolve != nullptr)
            {
                JPH_RVec3 contactPosition;
                JPH_Vec3 contactNormal;
                JPH_Vec3 contactVelocity;
                JPH_Vec3 characterVelocity;
                JPH_Vec3 newCharacterVelocity;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);
                FromJolt(inContactVelocity, &contactVelocity);
                FromJolt(inCharacterVelocity, &characterVelocity);
                FromJolt(ioNewCharacterVelocity, &newCharacterVelocity);

                s_Procs->OnCharacterContactSolve(userData,
                                                 ToCharacterVirtual(inCharacter),
                                                 ToCharacterVirtual(inOtherCharacter),
                                                 inSubShapeID2.GetValue(),
                                                 &contactPosition,
                                                 &contactNormal,
                                                 &contactVelocity,
                                                 reinterpret_cast<const JPH_PhysicsMaterial *>(inContactMaterial),
                                                 &characterVelocity,
                                                 &newCharacterVelocity);

                ioNewCharacterVelocity = ToJolt(newCharacterVelocity);
            }
        }
};

const JPH_CharacterContactListener_Procs *ManagedCharacterContactListener::s_Procs = nullptr;

void JPH_CharacterContactListener_SetProcs(const JPH_CharacterContactListener_Procs *procs)
{
    ManagedCharacterContactListener::s_Procs = procs;
}

JPH_CharacterContactListener *JPH_CharacterContactListener_Create(void *userData)
{
    ManagedCharacterContactListener *const impl = new ManagedCharacterContactListener(userData);
    return reinterpret_cast<JPH_CharacterContactListener *>(impl);
}

void JPH_CharacterContactListener_Destroy(JPH_CharacterContactListener *listener)
{
    if (listener != nullptr)
    {
        delete reinterpret_cast<ManagedCharacterContactListener *>(listener);
    }
}

/* JPH_CharacterVsCharacterCollision */
class ManagedCharacterVsCharacterCollision final: public JPH::CharacterVsCharacterCollision
{
    public:
        static const JPH_CharacterVsCharacterCollision_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedCharacterVsCharacterCollision(void *userData_): userData(userData_) {}

        void CollideCharacter(const JPH::CharacterVirtual *inCharacter,
                              JPH::RMat44Arg inCenterOfMassTransform,
                              const JPH::CollideShapeSettings &inCollideShapeSettings,
                              JPH::RVec3Arg inBaseOffset,
                              JPH::CollideShapeCollector & /*ioCollector*/) const override
        {
            if (s_Procs != nullptr && s_Procs->CollideCharacter != nullptr)
            {
                JPH_RMatrix4x4 centerOfMassTransform;
                JPH_RVec3 baseOffset;

                const JPH_CollideShapeSettings collideShapeSettings = FromJolt(inCollideShapeSettings);
                FromJolt(inCenterOfMassTransform, &centerOfMassTransform);
                FromJolt(inBaseOffset, &baseOffset);

                s_Procs->CollideCharacter(userData,
                                          ToCharacterVirtual(inCharacter),
                                          &centerOfMassTransform,
                                          &collideShapeSettings,
                                          &baseOffset);
            }
        }

        void CastCharacter(const JPH::CharacterVirtual *inCharacter,
                           JPH::RMat44Arg inCenterOfMassTransform,
                           JPH::Vec3Arg inDirection,
                           const JPH::ShapeCastSettings &inShapeCastSettings,
                           JPH::RVec3Arg inBaseOffset,
                           JPH::CastShapeCollector & /*ioCollector*/) const override
        {
            if (s_Procs != nullptr && s_Procs->CastCharacter != nullptr)
            {
                JPH_RMatrix4x4 centerOfMassTransform;
                JPH_Vec3 direction;
                JPH_RVec3 baseOffset;

                const JPH_ShapeCastSettings shapeCastSettings = FromJolt(inShapeCastSettings);
                FromJolt(inCenterOfMassTransform, &centerOfMassTransform);
                FromJolt(inDirection, &direction);
                FromJolt(inBaseOffset, &baseOffset);

                s_Procs->CastCharacter(userData,
                                       ToCharacterVirtual(inCharacter),
                                       &centerOfMassTransform,
                                       &direction,
                                       &shapeCastSettings,
                                       &baseOffset);
            }
        }
};

const JPH_CharacterVsCharacterCollision_Procs *ManagedCharacterVsCharacterCollision::s_Procs = nullptr;

void JPH_CharacterVsCharacterCollision_SetProcs(const JPH_CharacterVsCharacterCollision_Procs *procs)
{
    ManagedCharacterVsCharacterCollision::s_Procs = procs;
}

JPH_CharacterVsCharacterCollision *JPH_CharacterVsCharacterCollision_Create(void *userData)
{
    ManagedCharacterVsCharacterCollision *const impl = new ManagedCharacterVsCharacterCollision(userData);
    return reinterpret_cast<JPH_CharacterVsCharacterCollision *>(impl);
}

JPH_CharacterVsCharacterCollision *JPH_CharacterVsCharacterCollision_CreateSimple()
{
    JPH::CharacterVsCharacterCollisionSimple *impl = new JPH::CharacterVsCharacterCollisionSimple();
    return reinterpret_cast<JPH_CharacterVsCharacterCollision *>(impl);
}

void JPH_CharacterVsCharacterCollisionSimple_AddCharacter(JPH_CharacterVsCharacterCollision *characterVsCharacter,
                                                          JPH_CharacterVirtual *character)
{
    reinterpret_cast<JPH::CharacterVsCharacterCollisionSimple *>(characterVsCharacter)
            ->Add(AsCharacterVirtual(character));
}

void JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter(JPH_CharacterVsCharacterCollision *characterVsCharacter,
                                                             JPH_CharacterVirtual *character)
{
    reinterpret_cast<JPH::CharacterVsCharacterCollisionSimple *>(characterVsCharacter)
            ->Remove(AsCharacterVirtual(character));
}

void JPH_CharacterVsCharacterCollision_Destroy(JPH_CharacterVsCharacterCollision *listener)
{
    if (listener != nullptr)
    {
        delete reinterpret_cast<JPH::CharacterVsCharacterCollision *>(listener);
    }
}

/* CollisionDispatch */
bool JPH_CollisionDispatch_CollideShapeVsShape(const JPH_Shape *shape1,
                                               const JPH_Shape *shape2,
                                               const JPH_Vec3 *scale1,
                                               const JPH_Vec3 *scale2,
                                               const JPH_Matrix4x4 *centerOfMassTransform1,
                                               const JPH_Matrix4x4 *centerOfMassTransform2,
                                               const JPH_CollideShapeSettings *collideShapeSettings,
                                               JPH_CollideShapeCollectorCallback *callback,
                                               void *userData,
                                               const JPH_ShapeFilter *shapeFilter)
{
    CollideShapeCollectorCallback collector(callback, userData);

    JPH::CollisionDispatch::sCollideShapeVsShape(AsShape(shape1),
                                                 AsShape(shape2),
                                                 ToJolt(scale1),
                                                 ToJolt(scale2),
                                                 ToJolt(centerOfMassTransform1),
                                                 ToJolt(centerOfMassTransform2),
                                                 JPH::SubShapeIDCreator(),
                                                 JPH::SubShapeIDCreator(),
                                                 ToJolt(collideShapeSettings),
                                                 collector,
                                                 ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_CollisionDispatch_CastShapeVsShapeLocalSpace(const JPH_Vec3 *direction,
                                                      const JPH_Shape *shape1,
                                                      const JPH_Shape *shape2,
                                                      const JPH_Vec3 *scale1InShape2LocalSpace,
                                                      const JPH_Vec3 *scale2,
                                                      const JPH_Matrix4x4 *centerOfMassTransform1InShape2LocalSpace,
                                                      const JPH_Matrix4x4 *centerOfMassWorldTransform2,
                                                      const JPH_ShapeCastSettings *shapeCastSettings,
                                                      JPH_CastShapeCollectorCallback *callback,
                                                      void *userData,
                                                      const JPH_ShapeFilter *shapeFilter)
{
    const JPH::ShapeCast shapeCast(AsShape(shape1),
                                   ToJolt(scale1InShape2LocalSpace),
                                   ToJolt(centerOfMassTransform1InShape2LocalSpace),
                                   ToJolt(direction));

    CastShapeCollectorCallback collector(callback, userData);

    JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(shapeCast,
                                                        ToJolt(shapeCastSettings),
                                                        AsShape(shape2),
                                                        ToJolt(scale2),
                                                        ToJolt(shapeFilter),
                                                        ToJolt(centerOfMassWorldTransform2),
                                                        JPH::SubShapeIDCreator(),
                                                        JPH::SubShapeIDCreator(),
                                                        collector);

    return collector.hadHit;
}

bool JPH_CollisionDispatch_CastShapeVsShapeWorldSpace(const JPH_Vec3 *direction,
                                                      const JPH_Shape *shape1,
                                                      const JPH_Shape *shape2,
                                                      const JPH_Vec3 *scale1,
                                                      const JPH_Vec3 *scale2,
                                                      const JPH_Matrix4x4 *centerOfMassWorldTransform1,
                                                      const JPH_Matrix4x4 *centerOfMassWorldTransform2,
                                                      const JPH_ShapeCastSettings *shapeCastSettings,
                                                      JPH_CastShapeCollectorCallback *callback,
                                                      void *userData,
                                                      const JPH_ShapeFilter *shapeFilter)
{
    const JPH::ShapeCast shapeCast = JPH::ShapeCast::sFromWorldTransform(AsShape(shape1),
                                                                         ToJolt(scale1),
                                                                         ToJolt(centerOfMassWorldTransform1),
                                                                         ToJolt(direction));

    CastShapeCollectorCallback collector(callback, userData);

    JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(shapeCast,
                                                        ToJolt(shapeCastSettings),
                                                        AsShape(shape2),
                                                        ToJolt(scale2),
                                                        ToJolt(shapeFilter),
                                                        ToJolt(centerOfMassWorldTransform2),
                                                        JPH::SubShapeIDCreator(),
                                                        JPH::SubShapeIDCreator(),
                                                        collector);

    return collector.hadHit;
}

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRendererSimple.h>

/* JPH_BodyDrawFilter */
class ManagedBodyDrawFilter final: public JPH::BodyDrawFilter
{
    public:
        static const JPH_BodyDrawFilter_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedBodyDrawFilter(void *userData_): userData(userData_) {}

        [[nodiscard]] bool ShouldDraw([[maybe_unused]] const JPH::Body &inBody) const override
        {
            if (s_Procs != nullptr && s_Procs->ShouldDraw != nullptr)
            {
                return s_Procs->ShouldDraw(userData, reinterpret_cast<const JPH_Body *>(&inBody));
            }

            return true;
        }
};
const JPH_BodyDrawFilter_Procs *ManagedBodyDrawFilter::s_Procs = nullptr;

void JPH_BodyDrawFilter_SetProcs(const JPH_BodyDrawFilter_Procs *procs)
{
    ManagedBodyDrawFilter::s_Procs = procs;
}

JPH_BodyDrawFilter *JPH_BodyDrawFilter_Create(void *userData)
{
    ManagedBodyDrawFilter *const filter = new ManagedBodyDrawFilter(userData);
    return reinterpret_cast<JPH_BodyDrawFilter *>(filter);
}

void JPH_BodyDrawFilter_Destroy(JPH_BodyDrawFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedBodyDrawFilter *>(filter);
    }
}

/* DebugRenderer */
class ManagedDebugRendererSimple final: public JPH::DebugRendererSimple
{
    public:
        static const JPH_DebugRenderer_Procs *s_Procs;
        void *userData = nullptr;

        explicit ManagedDebugRendererSimple(void *userData_): userData(userData_) {}

        void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, const JPH::ColorArg inColor) override
        {
            if (s_Procs != nullptr && s_Procs->DrawLine != nullptr)
            {
                JPH_RVec3 from;
                JPH_RVec3 to;

                FromJolt(inFrom, &from);
                FromJolt(inTo, &to);

                s_Procs->DrawLine(userData, &from, &to, inColor.GetUInt32());
            }
        }

        void DrawTriangle(JPH::RVec3Arg inV1,
                          JPH::RVec3Arg inV2,
                          JPH::RVec3Arg inV3,
                          const JPH::ColorArg inColor,
                          ECastShadow inCastShadow = ECastShadow::Off) override
        {
            if (s_Procs != nullptr && s_Procs->DrawTriangle != nullptr)
            {
                JPH_RVec3 v1;
                JPH_RVec3 v2;
                JPH_RVec3 v3;

                FromJolt(inV1, &v1);
                FromJolt(inV2, &v2);
                FromJolt(inV3, &v3);

                s_Procs->DrawTriangle(userData,
                                      &v1,
                                      &v2,
                                      &v3,
                                      inColor.GetUInt32(),
                                      static_cast<JPH_DebugRenderer_CastShadow>(inCastShadow));
            } else
            {
                DebugRendererSimple::DrawTriangle(inV1, inV2, inV3, inColor, inCastShadow);
            }
        }

        void DrawText3D(JPH::RVec3Arg inPosition,
                        const JPH::string_view &inString,
                        const JPH::ColorArg inColor,
                        const float inHeight) override
        {
            if (s_Procs != nullptr && s_Procs->DrawText3D != nullptr)
            {
                JPH_RVec3 position;

                FromJolt(inPosition, &position);

                s_Procs->DrawText3D(userData, &position, inString.data(), inColor.GetUInt32(), inHeight);
            }
        }
};

const JPH_DebugRenderer_Procs *ManagedDebugRendererSimple::s_Procs = nullptr;

void JPH_DebugRenderer_SetProcs(const JPH_DebugRenderer_Procs *procs)
{
    ManagedDebugRendererSimple::s_Procs = procs;
}

JPH_DebugRenderer *JPH_DebugRenderer_Create(void *userData)
{
    ManagedDebugRendererSimple *const impl = new ManagedDebugRendererSimple(userData);
    return reinterpret_cast<JPH_DebugRenderer *>(impl);
}

void JPH_DebugRenderer_Destroy(JPH_DebugRenderer *renderer)
{
    if (renderer != nullptr)
    {
        delete reinterpret_cast<ManagedDebugRendererSimple *>(renderer);
    }
}

void JPH_DebugRenderer_NextFrame(JPH_DebugRenderer *renderer)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->NextFrame();
}

void JPH_DebugRenderer_SetCameraPos(JPH_DebugRenderer *renderer, const JPH_RVec3 *position)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->SetCameraPos(ToJolt(position));
}

void JPH_DebugRenderer_DrawLine(JPH_DebugRenderer *renderer,
                                const JPH_RVec3 *from,
                                const JPH_RVec3 *to,
                                const JPH_Color color)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawLine(ToJolt(from), ToJolt(to), JPH::Color(color));
}

void JPH_DebugRenderer_DrawWireBox(JPH_DebugRenderer *renderer, const JPH_AABox *box, const JPH_Color color)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawWireBox(ToJolt(box), JPH::Color(color));
}

void JPH_DebugRenderer_DrawWireBox2(JPH_DebugRenderer *renderer,
                                    const JPH_RMatrix4x4 *matrix,
                                    const JPH_AABox *box,
                                    const JPH_Color color)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawWireBox(ToJolt(matrix),
                                                                          ToJolt(box),
                                                                          JPH::Color(color));
}

void JPH_DebugRenderer_DrawMarker(JPH_DebugRenderer *renderer,
                                  const JPH_RVec3 *position,
                                  const JPH_Color color,
                                  const float size)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawMarker(ToJolt(position), JPH::Color(color), size);
}

void JPH_DebugRenderer_DrawArrow(JPH_DebugRenderer *renderer,
                                 const JPH_RVec3 *from,
                                 const JPH_RVec3 *to,
                                 const JPH_Color color,
                                 const float size)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawArrow(ToJolt(from),
                                                                        ToJolt(to),
                                                                        JPH::Color(color),
                                                                        size);
}

void JPH_DebugRenderer_DrawCoordinateSystem(JPH_DebugRenderer *renderer, const JPH_RMatrix4x4 *matrix, const float size)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawCoordinateSystem(ToJolt(matrix), size);
}

void JPH_DebugRenderer_DrawPlane(JPH_DebugRenderer *renderer,
                                 const JPH_RVec3 *point,
                                 const JPH_Vec3 *normal,
                                 const JPH_Color color,
                                 const float size)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawPlane(ToJolt(point),
                                                                        ToJolt(normal),
                                                                        JPH::Color(color),
                                                                        size);
}

void JPH_DebugRenderer_DrawWireTriangle(JPH_DebugRenderer *renderer,
                                        const JPH_RVec3 *v1,
                                        const JPH_RVec3 *v2,
                                        const JPH_RVec3 *v3,
                                        const JPH_Color color)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawWireTriangle(ToJolt(v1),
                                                                               ToJolt(v2),
                                                                               ToJolt(v3),
                                                                               JPH::Color(color));
}

void JPH_DebugRenderer_DrawWireSphere(JPH_DebugRenderer *renderer,
                                      const JPH_RVec3 *center,
                                      const float radius,
                                      const JPH_Color color,
                                      const int level)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawWireSphere(ToJolt(center),
                                                                             radius,
                                                                             JPH::Color(color),
                                                                             level);
}

void JPH_DebugRenderer_DrawWireUnitSphere(JPH_DebugRenderer *renderer,
                                          const JPH_RMatrix4x4 *matrix,
                                          const JPH_Color color,
                                          const int level)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawWireUnitSphere(ToJolt(matrix),
                                                                                 JPH::Color(color),
                                                                                 level);
}

void JPH_DebugRenderer_DrawTriangle(JPH_DebugRenderer *renderer,
                                    const JPH_RVec3 *v1,
                                    const JPH_RVec3 *v2,
                                    const JPH_RVec3 *v3,
                                    const JPH_Color color,
                                    JPH_DebugRenderer_CastShadow castShadow)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawTriangle(ToJolt(v1),
                           ToJolt(v2),
                           ToJolt(v3),
                           JPH::Color(color),
                           static_cast<JPH::DebugRenderer::ECastShadow>(castShadow));
}

void JPH_DebugRenderer_DrawBox(JPH_DebugRenderer *renderer,
                               const JPH_AABox *box,
                               const JPH_Color color,
                               JPH_DebugRenderer_CastShadow castShadow,
                               JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawBox(ToJolt(box),
                      JPH::Color(color),
                      static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                      static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawBox2(JPH_DebugRenderer *renderer,
                                const JPH_RMatrix4x4 *matrix,
                                const JPH_AABox *box,
                                const JPH_Color color,
                                JPH_DebugRenderer_CastShadow castShadow,
                                JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawBox(ToJolt(matrix),
                      ToJolt(box),
                      JPH::Color(color),
                      static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                      static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawSphere(JPH_DebugRenderer *renderer,
                                  const JPH_RVec3 *center,
                                  const float radius,
                                  const JPH_Color color,
                                  JPH_DebugRenderer_CastShadow castShadow,
                                  JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawSphere(ToJolt(center),
                         radius,
                         JPH::Color(color),
                         static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                         static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawUnitSphere(JPH_DebugRenderer *renderer,
                                      const JPH_RMatrix4x4 *matrix,
                                      const JPH_Color color,
                                      JPH_DebugRenderer_CastShadow castShadow,
                                      JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawUnitSphere(ToJolt(matrix),
                             JPH::Color(color),
                             static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                             static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawCapsule(JPH_DebugRenderer *renderer,
                                   const JPH_RMatrix4x4 *matrix,
                                   const float halfHeightOfCylinder,
                                   const float radius,
                                   const JPH_Color color,
                                   JPH_DebugRenderer_CastShadow castShadow,
                                   JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawCapsule(ToJolt(matrix),
                          halfHeightOfCylinder,
                          radius,
                          JPH::Color(color),
                          static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                          static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawCylinder(JPH_DebugRenderer *renderer,
                                    const JPH_RMatrix4x4 *matrix,
                                    const float halfHeight,
                                    const float radius,
                                    const JPH_Color color,
                                    JPH_DebugRenderer_CastShadow castShadow,
                                    JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawCylinder(ToJolt(matrix),
                           halfHeight,
                           radius,
                           JPH::Color(color),
                           static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                           static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawOpenCone(JPH_DebugRenderer *renderer,
                                    const JPH_RVec3 *top,
                                    const JPH_Vec3 *axis,
                                    const JPH_Vec3 *perpendicular,
                                    const float halfAngle,
                                    const float length,
                                    const JPH_Color color,
                                    JPH_DebugRenderer_CastShadow castShadow,
                                    JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawOpenCone(ToJolt(top),
                           ToJolt(axis),
                           ToJolt(perpendicular),
                           halfAngle,
                           length,
                           JPH::Color(color),
                           static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                           static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawSwingConeLimits(JPH_DebugRenderer *renderer,
                                           const JPH_RMatrix4x4 *matrix,
                                           const float swingYHalfAngle,
                                           const float swingZHalfAngle,
                                           const float edgeLength,
                                           const JPH_Color color,
                                           JPH_DebugRenderer_CastShadow castShadow,
                                           JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawSwingConeLimits(ToJolt(matrix),
                                  swingYHalfAngle,
                                  swingZHalfAngle,
                                  edgeLength,
                                  JPH::Color(color),
                                  static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                                  static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawSwingPyramidLimits(JPH_DebugRenderer *renderer,
                                              const JPH_RMatrix4x4 *matrix,
                                              const float minSwingYAngle,
                                              const float maxSwingYAngle,
                                              const float minSwingZAngle,
                                              const float maxSwingZAngle,
                                              const float edgeLength,
                                              const JPH_Color color,
                                              JPH_DebugRenderer_CastShadow castShadow,
                                              JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawSwingPyramidLimits(ToJolt(matrix),
                                     minSwingYAngle,
                                     maxSwingYAngle,
                                     minSwingZAngle,
                                     maxSwingZAngle,
                                     edgeLength,
                                     JPH::Color(color),
                                     static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                                     static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}
void JPH_DebugRenderer_DrawPie(JPH_DebugRenderer *renderer,
                               const JPH_RVec3 *center,
                               const float radius,
                               const JPH_Vec3 *normal,
                               const JPH_Vec3 *axis,
                               const float minAngle,
                               const float maxAngle,
                               const JPH_Color color,
                               JPH_DebugRenderer_CastShadow castShadow,
                               JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawPie(ToJolt(center),
                      radius,
                      ToJolt(normal),
                      ToJolt(axis),
                      minAngle,
                      maxAngle,
                      JPH::Color(color),
                      static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                      static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawTaperedCylinder(JPH_DebugRenderer *renderer,
                                           const JPH_RMatrix4x4 *inMatrix,
                                           const float top,
                                           const float bottom,
                                           const float topRadius,
                                           const float bottomRadius,
                                           const JPH_Color color,
                                           JPH_DebugRenderer_CastShadow castShadow,
                                           JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)
            ->DrawTaperedCylinder(ToJolt(inMatrix),
                                  top,
                                  bottom,
                                  topRadius,
                                  bottomRadius,
                                  JPH::Color(color),
                                  static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
                                  static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}
#endif

/* Skeleton */
JPH_Skeleton *JPH_Skeleton_Create()
{
    JPH::Skeleton *const skeleton = new JPH::Skeleton();
    skeleton->AddRef();

    return ToSkeleton(skeleton);
}

void JPH_Skeleton_Destroy(JPH_Skeleton *skeleton)
{
    if (skeleton != nullptr)
    {
        AsSkeleton(skeleton)->Release();
    }
}

uint32_t JPH_Skeleton_AddJoint(JPH_Skeleton *skeleton, const char *name)
{
    return AsSkeleton(skeleton)->AddJoint(name);
}

uint32_t JPH_Skeleton_AddJoint2(JPH_Skeleton *skeleton, const char *name, const int parentIndex)
{
    return AsSkeleton(skeleton)->AddJoint(name, parentIndex);
}

uint32_t JPH_Skeleton_AddJoint3(JPH_Skeleton *skeleton, const char *name, const char *parentName)
{
    return AsSkeleton(skeleton)->AddJoint(name, parentName);
}

int JPH_Skeleton_GetJointCount(const JPH_Skeleton *skeleton)
{
    return AsSkeleton(skeleton)->GetJointCount();
}

void JPH_Skeleton_GetJoint(const JPH_Skeleton *skeleton, const int index, JPH_SkeletonJoint *joint)
{
    FromJolt(AsSkeleton(skeleton)->GetJoint(index), joint);
}

int JPH_Skeleton_GetJointIndex(const JPH_Skeleton *skeleton, const char *name)
{
    return AsSkeleton(skeleton)->GetJointIndex(name);
}

void JPH_Skeleton_CalculateParentJointIndices(JPH_Skeleton *skeleton)
{
    AsSkeleton(skeleton)->CalculateParentJointIndices();
}

bool JPH_Skeleton_AreJointsCorrectlyOrdered(const JPH_Skeleton *skeleton)
{
    return AsSkeleton(skeleton)->AreJointsCorrectlyOrdered();
}

/* Ragdoll */
JPH_RagdollSettings *JPH_RagdollSettings_Create()
{
    JPH::RagdollSettings *const settings = new JPH::RagdollSettings();
    settings->AddRef();
    return ToRagdollSettings(settings);
}

void JPH_RagdollSettings_Destroy(JPH_RagdollSettings *settings)
{
    if (settings != nullptr)
    {
        AsRagdollSettings(settings)->Release();
    }
}

const JPH_Skeleton *JPH_RagdollSettings_GetSkeleton(const JPH_RagdollSettings *character)
{
    return ToSkeleton(AsRagdollSettings(character)->GetSkeleton());
}

void JPH_RagdollSettings_SetSkeleton(JPH_RagdollSettings *character, JPH_Skeleton *skeleton)
{
    AsRagdollSettings(character)->mSkeleton = AsSkeleton(skeleton);
}

bool JPH_RagdollSettings_Stabilize(JPH_RagdollSettings *settings)
{
    return AsRagdollSettings(settings)->Stabilize();
}

void JPH_RagdollSettings_DisableParentChildCollisions(JPH_RagdollSettings *settings,
                                                      const JPH_Matrix4x4 *jointMatrices,
                                                      const float minSeparationDistance)
{
    if (jointMatrices != nullptr)
    {
        const JPH::Mat44 joltJointMatrices = ToJolt(jointMatrices);
        AsRagdollSettings(settings)->DisableParentChildCollisions(&joltJointMatrices, minSeparationDistance);
    } else
    {
        AsRagdollSettings(settings)->DisableParentChildCollisions(nullptr, minSeparationDistance);
    }
}

void JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex(JPH_RagdollSettings *settings)
{
    AsRagdollSettings(settings)->CalculateBodyIndexToConstraintIndex();
}

int JPH_RagdollSettings_GetConstraintIndexForBodyIndex(JPH_RagdollSettings *settings, const int bodyIndex)
{
    return AsRagdollSettings(settings)->GetConstraintIndexForBodyIndex(bodyIndex);
}

void JPH_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(JPH_RagdollSettings *settings)
{
    AsRagdollSettings(settings)->CalculateConstraintIndexToBodyIdxPair();
}

JPH_Ragdoll *JPH_RagdollSettings_CreateRagdoll(JPH_RagdollSettings *settings,
                                               const JPH_PhysicsSystem *system,
                                               const JPH_CollisionGroupID collisionGroup,
                                               const uint64_t userData)
{
    JPH::Ragdoll *ragdoll = AsRagdollSettings(settings)->CreateRagdoll(collisionGroup, userData, system->physicsSystem);
    ragdoll->AddRef();

    return ToRagdoll(ragdoll);
}

void JPH_Ragdoll_Destroy(JPH_Ragdoll *ragdoll)
{
    if (ragdoll != nullptr)
    {
        AsRagdoll(ragdoll)->Release();
    }
}

void JPH_Ragdoll_AddToPhysicsSystem(JPH_Ragdoll *ragdoll,
                                    JPH_Activation activationMode /*= JPH_ActivationActivate */,
                                    const bool lockBodies /* = true */)
{
    AsRagdoll(ragdoll)->AddToPhysicsSystem(static_cast<JPH::EActivation>(activationMode), lockBodies);
}

void JPH_Ragdoll_RemoveFromPhysicsSystem(JPH_Ragdoll *ragdoll, const bool lockBodies /* = true */)
{
    AsRagdoll(ragdoll)->RemoveFromPhysicsSystem(lockBodies);
}

void JPH_Ragdoll_Activate(JPH_Ragdoll *ragdoll, const bool lockBodies /* = true */)
{
    AsRagdoll(ragdoll)->Activate(lockBodies);
}

bool JPH_Ragdoll_IsActive(const JPH_Ragdoll *ragdoll, const bool lockBodies /* = true */)
{
    return AsRagdoll(ragdoll)->IsActive(lockBodies);
}

void JPH_Ragdoll_ResetWarmStart(JPH_Ragdoll *ragdoll)
{
    AsRagdoll(ragdoll)->ResetWarmStart();
}

/* CollisionEstimationResult */
void JPH_EstimateCollisionResponse(const JPH_Body *body1,
                                   const JPH_Body *body2,
                                   const JPH_ContactManifold *manifold,
                                   float combinedFriction,
                                   float combinedRestitution,
                                   float minVelocityForRestitution,
                                   uint32_t numIterations,
                                   JPH_CollisionEstimationResult *result)
{
    JPH_ASSERT(result);

    JPH::CollisionEstimationResult joltResult;

    JPH::EstimateCollisionResponse(*AsBody(body1),
                                   *AsBody(body2),
                                   *AsContactManifold(manifold),
                                   joltResult,
                                   combinedFriction,
                                   combinedRestitution,
                                   minVelocityForRestitution,
                                   numIterations);

    FromJolt(joltResult.mLinearVelocity1, &result->linearVelocity1);
    FromJolt(joltResult.mAngularVelocity1, &result->angularVelocity1);
    FromJolt(joltResult.mLinearVelocity2, &result->linearVelocity2);
    FromJolt(joltResult.mAngularVelocity2, &result->angularVelocity2);
    FromJolt(joltResult.mTangent1, &result->tangent1);
    FromJolt(joltResult.mTangent2, &result->tangent2);

    if (!joltResult.mImpulses.empty())
    {
        result->impulseCount = joltResult.mImpulses.size();
        result->impulses = static_cast<
                JPH_CollisionEstimationResultImpulse *>(malloc(sizeof(JPH_CollisionEstimationResultImpulse) *
                                                               joltResult.mImpulses.size()));
        for (uint32_t i = 0; i < result->impulseCount; i++)
        {
            result->impulses[i].contactImpulse = joltResult.mImpulses[i].mContactImpulse;
            result->impulses[i].frictionImpulse1 = joltResult.mImpulses[i].mFrictionImpulse1;
            result->impulses[i].frictionImpulse2 = joltResult.mImpulses[i].mFrictionImpulse2;
        }
    }
}

/* Wheel */
JPH_WheelSettings *JPH_WheelSettings_Create()
{
    JPH::WheelSettings *const settings = new JPH::WheelSettings();
    settings->AddRef();

    return ToWheelSettings(settings);
}

void JPH_WheelSettings_Destroy(JPH_WheelSettings *settings)
{
    if (settings != nullptr)
    {
        AsWheelSettings(settings)->Release();
    }
}

void JPH_WheelSettings_GetPosition(const JPH_WheelSettings *settings, JPH_Vec3 *result)
{
    FromJolt(AsWheelSettings(settings)->mPosition, result);
}

void JPH_WheelSettings_SetPosition(JPH_WheelSettings *settings, const JPH_Vec3 *value)
{
    AsWheelSettings(settings)->mPosition = ToJolt(value);
}

void JPH_WheelSettings_GetSuspensionForcePoint(const JPH_WheelSettings *settings, JPH_Vec3 *result)
{
    FromJolt(AsWheelSettings(settings)->mSuspensionForcePoint, result);
}

void JPH_WheelSettings_SetSuspensionForcePoint(JPH_WheelSettings *settings, const JPH_Vec3 *value)
{
    AsWheelSettings(settings)->mSuspensionForcePoint = ToJolt(value);
}

void JPH_WheelSettings_GetSuspensionDirection(const JPH_WheelSettings *settings, JPH_Vec3 *result)
{
    FromJolt(AsWheelSettings(settings)->mSuspensionDirection, result);
}

void JPH_WheelSettings_SetSuspensionDirection(JPH_WheelSettings *settings, const JPH_Vec3 *value)
{
    AsWheelSettings(settings)->mSuspensionDirection = ToJolt(value);
}

void JPH_WheelSettings_GetSteeringAxis(const JPH_WheelSettings *settings, JPH_Vec3 *result)
{
    FromJolt(AsWheelSettings(settings)->mSteeringAxis, result);
}

void JPH_WheelSettings_SetSteeringAxis(JPH_WheelSettings *settings, const JPH_Vec3 *value)
{
    AsWheelSettings(settings)->mSteeringAxis = ToJolt(value);
}

void JPH_WheelSettings_GetWheelUp(const JPH_WheelSettings *settings, JPH_Vec3 *result)
{
    FromJolt(AsWheelSettings(settings)->mWheelUp, result);
}

void JPH_WheelSettings_SetWheelUp(JPH_WheelSettings *settings, const JPH_Vec3 *value)
{
    AsWheelSettings(settings)->mWheelUp = ToJolt(value);
}

void JPH_WheelSettings_GetWheelForward(const JPH_WheelSettings *settings, JPH_Vec3 *result)
{
    FromJolt(AsWheelSettings(settings)->mWheelForward, result);
}

void JPH_WheelSettings_SetWheelForward(JPH_WheelSettings *settings, const JPH_Vec3 *value)
{
    AsWheelSettings(settings)->mWheelForward = ToJolt(value);
}

float JPH_WheelSettings_GetSuspensionMinLength(const JPH_WheelSettings *settings)
{
    return AsWheelSettings(settings)->mSuspensionMinLength;
}

void JPH_WheelSettings_SetSuspensionMinLength(JPH_WheelSettings *settings, const float value)
{
    AsWheelSettings(settings)->mSuspensionMinLength = value;
}

float JPH_WheelSettings_GetSuspensionMaxLength(const JPH_WheelSettings *settings)
{
    return AsWheelSettings(settings)->mSuspensionMaxLength;
}

void JPH_WheelSettings_SetSuspensionMaxLength(JPH_WheelSettings *settings, const float value)
{
    AsWheelSettings(settings)->mSuspensionMaxLength = value;
}

float JPH_WheelSettings_GetSuspensionPreloadLength(const JPH_WheelSettings *settings)
{
    return AsWheelSettings(settings)->mSuspensionPreloadLength;
}

void JPH_WheelSettings_SetSuspensionPreloadLength(JPH_WheelSettings *settings, const float value)
{
    AsWheelSettings(settings)->mSuspensionPreloadLength = value;
}

void JPH_WheelSettings_GetSuspensionSpring(const JPH_WheelSettings *settings, JPH_SpringSettings *result)
{
    FromJolt(AsWheelSettings(settings)->mSuspensionSpring, result);
}

void JPH_WheelSettings_SetSuspensionSpring(JPH_WheelSettings *settings, const JPH_SpringSettings *springSettings)
{
    AsWheelSettings(settings)->mSuspensionSpring = ToJolt(springSettings);
}

float JPH_WheelSettings_GetRadius(const JPH_WheelSettings *settings)
{
    return AsWheelSettings(settings)->mRadius;
}

void JPH_WheelSettings_SetRadius(JPH_WheelSettings *settings, const float value)
{
    AsWheelSettings(settings)->mRadius = value;
}

float JPH_WheelSettings_GetWidth(const JPH_WheelSettings *settings)
{
    return AsWheelSettings(settings)->mWidth;
}

void JPH_WheelSettings_SetWidth(JPH_WheelSettings *settings, const float value)
{
    AsWheelSettings(settings)->mWidth = value;
}

bool JPH_WheelSettings_GetEnableSuspensionForcePoint(const JPH_WheelSettings *settings)
{
    return AsWheelSettings(settings)->mEnableSuspensionForcePoint;
}

void JPH_WheelSettings_SetEnableSuspensionForcePoint(JPH_WheelSettings *settings, const bool value)
{
    AsWheelSettings(settings)->mEnableSuspensionForcePoint = value;
}

JPH_Wheel *JPH_Wheel_Create(const JPH_WheelSettings *settings)
{
    JPH_ASSERT(settings);

    JPH::Wheel *wheel = new JPH::Wheel(*AsWheelSettings(settings));
    return ToWheel(wheel);
}

void JPH_Wheel_Destroy(JPH_Wheel *wheel)
{
    if (wheel != nullptr)
    {
        delete AsWheel(wheel);
    }
}

const JPH_WheelSettings *JPH_Wheel_GetSettings(const JPH_Wheel *wheel)
{
    return ToWheelSettings(AsWheel(wheel)->GetSettings());
}

float JPH_Wheel_GetAngularVelocity(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetAngularVelocity();
}

void JPH_Wheel_SetAngularVelocity(JPH_Wheel *wheel, const float value)
{
    AsWheel(wheel)->SetAngularVelocity(value);
}

float JPH_Wheel_GetRotationAngle(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetRotationAngle();
}

void JPH_Wheel_SetRotationAngle(JPH_Wheel *wheel, const float value)
{
    AsWheel(wheel)->SetRotationAngle(value);
}

float JPH_Wheel_GetSteerAngle(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetSteerAngle();
}

void JPH_Wheel_SetSteerAngle(JPH_Wheel *wheel, const float value)
{
    AsWheel(wheel)->SetSteerAngle(value);
}

bool JPH_Wheel_HasContact(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->HasContact();
}

JPH_BodyID JPH_Wheel_GetContactBodyID(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetContactBodyID().GetIndexAndSequenceNumber();
}

JPH_SubShapeID JPH_Wheel_GetContactSubShapeID(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetContactSubShapeID().GetValue();
}

void JPH_Wheel_GetContactPosition(const JPH_Wheel *wheel, JPH_RVec3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactPosition(), result);
}

void JPH_Wheel_GetContactPointVelocity(const JPH_Wheel *wheel, JPH_Vec3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactPointVelocity(), result);
}

void JPH_Wheel_GetContactNormal(const JPH_Wheel *wheel, JPH_Vec3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactNormal(), result);
}

void JPH_Wheel_GetContactLongitudinal(const JPH_Wheel *wheel, JPH_Vec3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactLongitudinal(), result);
}

void JPH_Wheel_GetContactLateral(const JPH_Wheel *wheel, JPH_Vec3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactLateral(), result);
}

float JPH_Wheel_GetSuspensionLength(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetSuspensionLength();
}

float JPH_Wheel_GetSuspensionLambda(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetSuspensionLambda();
}

float JPH_Wheel_GetLongitudinalLambda(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetLongitudinalLambda();
}

float JPH_Wheel_GetLateralLambda(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->GetLateralLambda();
}

bool JPH_Wheel_HasHitHardPoint(const JPH_Wheel *wheel)
{
    return AsWheel(wheel)->HasHitHardPoint();
}


/* VehicleAntiRollBar */
void JPH_VehicleAntiRollBar_Init(JPH_VehicleAntiRollBar *antiRollBar)
{
    JPH_ASSERT(antiRollBar);

    constexpr JPH::VehicleAntiRollBar joltAntiRollBar{};
    antiRollBar->leftWheel = joltAntiRollBar.mLeftWheel;
    antiRollBar->rightWheel = joltAntiRollBar.mRightWheel;
    antiRollBar->stiffness = joltAntiRollBar.mStiffness;
}

/* VehicleEngine */
void JPH_VehicleEngineSettings_Init(JPH_VehicleEngineSettings *settings)
{
    JPH_ASSERT(settings);

    const JPH::VehicleEngineSettings joltSettings{};
    settings->maxTorque = joltSettings.mMaxTorque;
    settings->minRPM = joltSettings.mMinRPM;
    settings->maxRPM = joltSettings.mMaxRPM;
    settings->inertia = joltSettings.mInertia;
    settings->angularDamping = joltSettings.mAngularDamping;
}

static void JPH_VehicleEngineSettings_FromJolt(JPH_VehicleEngineSettings *settings,
                                               const JPH::VehicleEngineSettings &joltSettings)
{
    JPH_ASSERT(settings);

    settings->maxTorque = joltSettings.mMaxTorque;
    settings->minRPM = joltSettings.mMinRPM;
    settings->maxRPM = joltSettings.mMaxRPM;
    settings->inertia = joltSettings.mInertia;
    settings->angularDamping = joltSettings.mAngularDamping;
}

static void JPH_VehicleEngineSettings_ToJolt(JPH::VehicleEngineSettings *joltSettings,
                                             const JPH_VehicleEngineSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    joltSettings->mMaxTorque = settings->maxTorque;
    joltSettings->mMinRPM = settings->minRPM;
    joltSettings->mMaxRPM = settings->maxRPM;
    joltSettings->mInertia = settings->inertia;
    joltSettings->mAngularDamping = settings->angularDamping;
}

/* VehicleDifferentialSettings */
void JPH_VehicleDifferentialSettings_Init(JPH_VehicleDifferentialSettings *settings)
{
    JPH_ASSERT(settings);

    constexpr JPH::VehicleDifferentialSettings joltSettings{};
    settings->leftWheel = joltSettings.mLeftWheel;
    settings->rightWheel = joltSettings.mRightWheel;
    settings->differentialRatio = joltSettings.mDifferentialRatio;
    settings->leftRightSplit = joltSettings.mLeftRightSplit;
    settings->limitedSlipRatio = joltSettings.mLimitedSlipRatio;
    settings->engineTorqueRatio = joltSettings.mEngineTorqueRatio;
}

static void JPH_VehicleDifferentialSettings_FromJolt(JPH_VehicleDifferentialSettings *settings,
                                                     const JPH::VehicleDifferentialSettings &joltSettings)
{
    // Copy settings to jolt
    settings->leftWheel = joltSettings.mLeftWheel;
    settings->rightWheel = joltSettings.mRightWheel;
    settings->differentialRatio = joltSettings.mDifferentialRatio;
    settings->leftRightSplit = joltSettings.mLeftRightSplit;
    settings->limitedSlipRatio = joltSettings.mLimitedSlipRatio;
    settings->engineTorqueRatio = joltSettings.mEngineTorqueRatio;
}

static JPH::VehicleDifferentialSettings JPH_VehicleDifferentialSettings_ToJolt(const JPH_VehicleDifferentialSettings
                                                                                       &settings)
{
    JPH::VehicleDifferentialSettings joltSettings{};
    joltSettings.mLeftWheel = settings.leftWheel;
    joltSettings.mRightWheel = settings.rightWheel;
    joltSettings.mDifferentialRatio = settings.differentialRatio;
    joltSettings.mLeftRightSplit = settings.leftRightSplit;
    joltSettings.mLimitedSlipRatio = settings.limitedSlipRatio;
    joltSettings.mEngineTorqueRatio = settings.engineTorqueRatio;
    return joltSettings;
}

/* VehicleTransmission */
JPH_VehicleTransmissionSettings *JPH_VehicleTransmissionSettings_Create()
{
    JPH::VehicleTransmissionSettings *const settings = new JPH::VehicleTransmissionSettings();
    return ToVehicleTransmissionSettings(settings);
}

void JPH_VehicleTransmissionSettings_Destroy(JPH_VehicleTransmissionSettings *settings)
{
    if (settings != nullptr)
    {
        delete AsVehicleTransmissionSettings(settings);
    }
}

JPH_TransmissionMode JPH_VehicleTransmissionSettings_GetMode(const JPH_VehicleTransmissionSettings *settings)
{
    return static_cast<JPH_TransmissionMode>(AsVehicleTransmissionSettings(settings)->mMode);
}

void JPH_VehicleTransmissionSettings_SetMode(JPH_VehicleTransmissionSettings *settings, JPH_TransmissionMode value)
{
    AsVehicleTransmissionSettings(settings)->mMode = static_cast<JPH::ETransmissionMode>(value);
}

uint32_t JPH_VehicleTransmissionSettings_GetGearRatioCount(const JPH_VehicleTransmissionSettings *settings)
{
    return static_cast<uint32_t>(AsVehicleTransmissionSettings(settings)->mGearRatios.size());
}

float JPH_VehicleTransmissionSettings_GetGearRatio(const JPH_VehicleTransmissionSettings *settings,
                                                   const uint32_t index)
{
    return AsVehicleTransmissionSettings(settings)->mGearRatios[index];
}

void JPH_VehicleTransmissionSettings_SetGearRatio(JPH_VehicleTransmissionSettings *settings,
                                                  const uint32_t index,
                                                  const float value)
{
    AsVehicleTransmissionSettings(settings)->mGearRatios[index] = value;
}

const float *JPH_VehicleTransmissionSettings_GetGearRatios(const JPH_VehicleTransmissionSettings *settings)
{
    JPH_ASSERT(settings);

    return AsVehicleTransmissionSettings(settings)->mGearRatios.data();
}

void JPH_VehicleTransmissionSettings_SetGearRatios(JPH_VehicleTransmissionSettings *settings,
                                                   const float *values,
                                                   uint32_t count)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(values);

    AsVehicleTransmissionSettings(settings)->mGearRatios.resize(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        AsVehicleTransmissionSettings(settings)->mGearRatios[i] = values[i];
    }
}

uint32_t JPH_VehicleTransmissionSettings_GetReverseGearRatioCount(const JPH_VehicleTransmissionSettings *settings)
{
    return static_cast<uint32_t>(AsVehicleTransmissionSettings(settings)->mReverseGearRatios.size());
}

float JPH_VehicleTransmissionSettings_GetReverseGearRatio(const JPH_VehicleTransmissionSettings *settings,
                                                          const uint32_t index)
{
    return AsVehicleTransmissionSettings(settings)->mReverseGearRatios[index];
}

void JPH_VehicleTransmissionSettings_SetReverseGearRatio(JPH_VehicleTransmissionSettings *settings,
                                                         const uint32_t index,
                                                         const float value)
{
    AsVehicleTransmissionSettings(settings)->mReverseGearRatios[index] = value;
}

const float *JPH_VehicleTransmissionSettings_GetReverseGearRatios(const JPH_VehicleTransmissionSettings *settings)
{
    JPH_ASSERT(settings);

    return AsVehicleTransmissionSettings(settings)->mReverseGearRatios.data();
}

void JPH_VehicleTransmissionSettings_SetReverseGearRatios(JPH_VehicleTransmissionSettings *settings,
                                                          const float *values,
                                                          uint32_t count)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(values);

    AsVehicleTransmissionSettings(settings)->mReverseGearRatios.resize(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        AsVehicleTransmissionSettings(settings)->mReverseGearRatios[i] = values[i];
    }
}

float JPH_VehicleTransmissionSettings_GetSwitchTime(const JPH_VehicleTransmissionSettings *settings)
{
    return AsVehicleTransmissionSettings(settings)->mSwitchTime;
}

void JPH_VehicleTransmissionSettings_SetSwitchTime(JPH_VehicleTransmissionSettings *settings, const float value)
{
    AsVehicleTransmissionSettings(settings)->mSwitchTime = value;
}

float JPH_VehicleTransmissionSettings_GetClutchReleaseTime(const JPH_VehicleTransmissionSettings *settings)
{
    return AsVehicleTransmissionSettings(settings)->mClutchReleaseTime;
}

void JPH_VehicleTransmissionSettings_SetClutchReleaseTime(JPH_VehicleTransmissionSettings *settings, const float value)
{
    AsVehicleTransmissionSettings(settings)->mClutchReleaseTime = value;
}

float JPH_VehicleTransmissionSettings_GetSwitchLatency(const JPH_VehicleTransmissionSettings *settings)
{
    return AsVehicleTransmissionSettings(settings)->mSwitchLatency;
}

void JPH_VehicleTransmissionSettings_SetSwitchLatency(JPH_VehicleTransmissionSettings *settings, const float value)
{
    AsVehicleTransmissionSettings(settings)->mSwitchLatency = value;
}

float JPH_VehicleTransmissionSettings_GetShiftUpRPM(const JPH_VehicleTransmissionSettings *settings)
{
    return AsVehicleTransmissionSettings(settings)->mShiftUpRPM;
}

void JPH_VehicleTransmissionSettings_SetShiftUpRPM(JPH_VehicleTransmissionSettings *settings, const float value)
{
    AsVehicleTransmissionSettings(settings)->mShiftUpRPM = value;
}

float JPH_VehicleTransmissionSettings_GetShiftDownRPM(const JPH_VehicleTransmissionSettings *settings)
{
    return AsVehicleTransmissionSettings(settings)->mShiftDownRPM;
}

void JPH_VehicleTransmissionSettings_SetShiftDownRPM(JPH_VehicleTransmissionSettings *settings, const float value)
{
    AsVehicleTransmissionSettings(settings)->mShiftDownRPM = value;
}

float JPH_VehicleTransmissionSettings_GetClutchStrength(const JPH_VehicleTransmissionSettings *settings)
{
    return AsVehicleTransmissionSettings(settings)->mClutchStrength;
}

void JPH_VehicleTransmissionSettings_SetClutchStrength(JPH_VehicleTransmissionSettings *settings, const float value)
{
    AsVehicleTransmissionSettings(settings)->mClutchStrength = value;
}

/* VehicleColliionTester */
void JPH_VehicleCollisionTester_Destroy(const JPH_VehicleCollisionTester *tester)
{
    if (tester != nullptr)
    {
        const JPH::VehicleCollisionTester
                *const joltTester = reinterpret_cast<const JPH::VehicleCollisionTester *>(tester);
        joltTester->Release();
    }
}

JPH_ObjectLayer JPH_VehicleCollisionTester_GetObjectLayer(const JPH_VehicleCollisionTester *tester)
{
    return AsVehicleCollisionTester(tester)->GetObjectLayer();
}

void JPH_VehicleCollisionTester_SetObjectLayer(JPH_VehicleCollisionTester *tester, const JPH_ObjectLayer value)
{
    AsVehicleCollisionTester(tester)->SetObjectLayer(value);
}

JPH_VehicleCollisionTesterRay *JPH_VehicleCollisionTesterRay_Create(const JPH_ObjectLayer layer,
                                                                    const JPH_Vec3 *up,
                                                                    const float maxSlopeAngle)
{
    JPH::VehicleCollisionTesterRay *const tester = new JPH::VehicleCollisionTesterRay(layer, ToJolt(up), maxSlopeAngle);
    tester->AddRef();

    return ToVehicleCollisionTesterRay(tester);
}

JPH_VehicleCollisionTesterCastSphere *JPH_VehicleCollisionTesterCastSphere_Create(const JPH_ObjectLayer layer,
                                                                                  const float radius,
                                                                                  const JPH_Vec3 *up,
                                                                                  const float maxSlopeAngle)
{
    JPH::VehicleCollisionTesterCastSphere *const tester = new JPH::VehicleCollisionTesterCastSphere(layer,
                                                                                                    radius,
                                                                                                    ToJolt(up),
                                                                                                    maxSlopeAngle);
    tester->AddRef();

    return ToVehicleCollisionTesterCastSphere(tester);
}

JPH_VehicleCollisionTesterCastCylinder *JPH_VehicleCollisionTesterCastCylinder_Create(const JPH_ObjectLayer layer,
                                                                                      const float convexRadiusFraction)
{
    JPH::VehicleCollisionTesterCastCylinder
            *const tester = new JPH::VehicleCollisionTesterCastCylinder(layer, convexRadiusFraction);
    tester->AddRef();

    return ToVehicleCollisionTesterCastCylinder(tester);
}

/* VehicleConstraintSettings */
void JPH_VehicleConstraintSettings_FromJolt(JPH_VehicleConstraintSettings *settings,
                                            const JPH::VehicleConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    FromJolt(joltSettings.mUp, &settings->up);
    FromJolt(joltSettings.mForward, &settings->forward);
    settings->maxPitchRollAngle = joltSettings.mMaxPitchRollAngle;
    settings->wheelsCount = 0;
    settings->wheels = nullptr;
    settings->controller = nullptr;
}

void JPH_VehicleConstraintSettings_Init(JPH_VehicleConstraintSettings *settings)
{
    JPH_ASSERT(settings);

    // Copy defaults from jolt
    const JPH::VehicleConstraintSettings joltSettings;
    JPH_VehicleConstraintSettings_FromJolt(settings, joltSettings);
}

void JPH_VehicleConstraintSettings_ToJolt(JPH::VehicleConstraintSettings *joltSettings,
                                          const JPH_VehicleConstraintSettings *settings)
{
    JPH_ASSERT(joltSettings);
    JPH_ASSERT(settings);

    // Base settings
    JPH_ConstraintSettings_ToJolt(joltSettings, &settings->base);

    joltSettings->mUp = ToJolt(settings->up);
    joltSettings->mForward = ToJolt(settings->forward);
    joltSettings->mMaxPitchRollAngle = settings->maxPitchRollAngle;

    joltSettings->mWheels.resize(settings->wheelsCount);
    for (uint32_t i = 0; i < settings->wheelsCount; ++i)
    {
        joltSettings->mWheels[i] = AsWheelSettings(settings->wheels[i]);
    }

    if (settings->antiRollBarsCount > 0)
    {
        joltSettings->mAntiRollBars.resize(settings->antiRollBarsCount);
        for (uint32_t i = 0; i < settings->antiRollBarsCount; ++i)
        {
            JPH::VehicleAntiRollBar joltAntiRollBar{};
            joltAntiRollBar.mLeftWheel = settings->antiRollBars[i].leftWheel;
            joltAntiRollBar.mRightWheel = settings->antiRollBars[i].rightWheel;
            joltAntiRollBar.mStiffness = settings->antiRollBars[i].stiffness;
            joltSettings->mAntiRollBars[i] = joltAntiRollBar;
        }
    }

    if (settings->controller != nullptr)
    {
        joltSettings->mController = AsVehicleControllerSettings(settings->controller);
    }
}

JPH_VehicleConstraint *JPH_VehicleConstraint_Create(JPH_Body *body, const JPH_VehicleConstraintSettings *settings)
{
    JPH_ASSERT(body);
    JPH_ASSERT(settings);

    JPH::VehicleConstraintSettings joltSettings;
    JPH_VehicleConstraintSettings_ToJolt(&joltSettings, settings);

    JPH::VehicleConstraint *vehicleConstraint = new JPH::VehicleConstraint(*AsBody(body), joltSettings);
    vehicleConstraint->AddRef();

    return ToVehicleConstraint(vehicleConstraint);
}

JPH_PhysicsStepListener *JPH_VehicleConstraint_AsPhysicsStepListener(JPH_VehicleConstraint *constraint)
{
    JPH_ASSERT(constraint);

    JPH::PhysicsStepListener *joltListener = AsVehicleConstraint(constraint);
    return ToPhysicsStepListener(joltListener);
}

void JPH_VehicleConstraint_SetMaxPitchRollAngle(JPH_VehicleConstraint *constraint, const float maxPitchRollAngle)
{
    AsVehicleConstraint(constraint)->SetMaxPitchRollAngle(maxPitchRollAngle);
}

void JPH_VehicleConstraint_SetVehicleCollisionTester(JPH_VehicleConstraint *constraint,
                                                     const JPH_VehicleCollisionTester *tester)
{
    AsVehicleConstraint(constraint)->SetVehicleCollisionTester(AsVehicleCollisionTester(tester));
}

void JPH_VehicleConstraint_OverrideGravity(JPH_VehicleConstraint *constraint, const JPH_Vec3 *value)
{
    AsVehicleConstraint(constraint)->OverrideGravity(ToJolt(value));
}

bool JPH_VehicleConstraint_IsGravityOverridden(const JPH_VehicleConstraint *constraint)
{
    return AsVehicleConstraint(constraint)->IsGravityOverridden();
}

void JPH_VehicleConstraint_GetGravityOverride(const JPH_VehicleConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetGravityOverride(), result);
}

void JPH_VehicleConstraint_ResetGravityOverride(JPH_VehicleConstraint *constraint)
{
    AsVehicleConstraint(constraint)->ResetGravityOverride();
}

void JPH_VehicleConstraint_GetLocalForward(const JPH_VehicleConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetLocalForward(), result);
}

void JPH_VehicleConstraint_GetLocalUp(const JPH_VehicleConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetLocalUp(), result);
}

void JPH_VehicleConstraint_GetWorldUp(const JPH_VehicleConstraint *constraint, JPH_Vec3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetWorldUp(), result);
}

const JPH_Body *JPH_VehicleConstraint_GetVehicleBody(const JPH_VehicleConstraint *constraint)
{
    return ToBody(AsVehicleConstraint(constraint)->GetVehicleBody());
}

JPH_VehicleController *JPH_VehicleConstraint_GetController(JPH_VehicleConstraint *constraint)
{
    return ToVehicleController(AsVehicleConstraint(constraint)->GetController());
}

uint32_t JPH_VehicleConstraint_GetWheelsCount(JPH_VehicleConstraint *constraint)
{
    return static_cast<uint32_t>(AsVehicleConstraint(constraint)->GetWheels().size());
}

JPH_Wheel *JPH_VehicleConstraint_GetWheel(JPH_VehicleConstraint *constraint, const uint32_t index)
{
    return ToWheel(AsVehicleConstraint(constraint)->GetWheel(index));
}

void JPH_VehicleConstraint_GetWheelLocalBasis(JPH_VehicleConstraint *constraint,
                                              const JPH_Wheel *wheel,
                                              JPH_Vec3 *outForward,
                                              JPH_Vec3 *outUp,
                                              JPH_Vec3 *outRight)
{
    JPH::Vec3 forward;
    JPH::Vec3 up;
    JPH::Vec3 right;

    AsVehicleConstraint(constraint)->GetWheelLocalBasis(AsWheel(wheel), forward, up, right);
    FromJolt(forward, outForward);
    FromJolt(up, outUp);
    FromJolt(right, outRight);
}

void JPH_VehicleConstraint_GetWheelLocalTransform(JPH_VehicleConstraint *constraint,
                                                  const uint32_t wheelIndex,
                                                  const JPH_Vec3 *wheelRight,
                                                  const JPH_Vec3 *wheelUp,
                                                  JPH_Matrix4x4 *result)
{
    const JPH::Mat44 joltResult = AsVehicleConstraint(constraint)
                                          ->GetWheelLocalTransform(wheelIndex, ToJolt(wheelRight), ToJolt(wheelUp));
    FromJolt(joltResult, result);
}

void JPH_VehicleConstraint_GetWheelWorldTransform(JPH_VehicleConstraint *constraint,
                                                  const uint32_t wheelIndex,
                                                  const JPH_Vec3 *wheelRight,
                                                  const JPH_Vec3 *wheelUp,
                                                  JPH_RMatrix4x4 *result)
{
    const JPH::RMat44 joltResult = AsVehicleConstraint(constraint)
                                           ->GetWheelWorldTransform(wheelIndex, ToJolt(wheelRight), ToJolt(wheelUp));
    FromJolt(joltResult, result);
}

/* VehicleControllerSettings */
void JPH_VehicleControllerSettings_Destroy(JPH_VehicleControllerSettings *settings)
{
    if (settings != nullptr)
    {
        AsVehicleControllerSettings(settings)->Release();
    }
}

const JPH_VehicleConstraint *JPH_VehicleController_GetConstraint(JPH_VehicleController *controller)
{
    const JPH::VehicleConstraint *constraint = &AsVehicleController(controller)->GetConstraint();
    return ToVehicleConstraint(constraint);
}

/* ---- WheelSettingsWV/WheelWV/WheeledVehicleController ---- */
JPH_WheelSettingsWV *JPH_WheelSettingsWV_Create()
{
    JPH::WheelSettingsWV *const settings = new JPH::WheelSettingsWV();
    settings->AddRef();

    return ToWheelSettingsWV(settings);
}

float JPH_WheelSettingsWV_GetInertia(const JPH_WheelSettingsWV *settings)
{
    return AsWheelSettingsWV(settings)->mInertia;
}

void JPH_WheelSettingsWV_SetInertia(JPH_WheelSettingsWV *settings, const float value)
{
    AsWheelSettingsWV(settings)->mInertia = value;
}

float JPH_WheelSettingsWV_GetAngularDamping(const JPH_WheelSettingsWV *settings)
{
    return AsWheelSettingsWV(settings)->mAngularDamping;
}

void JPH_WheelSettingsWV_SetAngularDamping(JPH_WheelSettingsWV *settings, const float value)
{
    AsWheelSettingsWV(settings)->mAngularDamping = value;
}

float JPH_WheelSettingsWV_GetMaxSteerAngle(const JPH_WheelSettingsWV *settings)
{
    return AsWheelSettingsWV(settings)->mMaxSteerAngle;
}

void JPH_WheelSettingsWV_SetMaxSteerAngle(JPH_WheelSettingsWV *settings, const float value)
{
    AsWheelSettingsWV(settings)->mMaxSteerAngle = value;
}

float JPH_WheelSettingsWV_GetMaxBrakeTorque(const JPH_WheelSettingsWV *settings)
{
    return AsWheelSettingsWV(settings)->mMaxBrakeTorque;
}

void JPH_WheelSettingsWV_SetMaxBrakeTorque(JPH_WheelSettingsWV *settings, const float value)
{
    AsWheelSettingsWV(settings)->mMaxBrakeTorque = value;
}

float JPH_WheelSettingsWV_GetMaxHandBrakeTorque(const JPH_WheelSettingsWV *settings)
{
    return AsWheelSettingsWV(settings)->mMaxHandBrakeTorque;
}

void JPH_WheelSettingsWV_SetMaxHandBrakeTorque(JPH_WheelSettingsWV *settings, const float value)
{
    AsWheelSettingsWV(settings)->mMaxHandBrakeTorque = value;
}

JPH_WheelWV *JPH_WheelWV_Create(const JPH_WheelSettingsWV *settings)
{
    JPH_ASSERT(settings);

    JPH::WheelWV *wheel = new JPH::WheelWV(*AsWheelSettingsWV(settings));
    return ToWheelWV(wheel);
}

const JPH_WheelSettingsWV *JPH_WheelWV_GetSettings(const JPH_WheelWV *wheel)
{
    return ToWheelSettingsWV(AsWheelWV(wheel)->GetSettings());
}

void JPH_WheelWV_ApplyTorque(JPH_WheelWV *wheel, const float torque, const float deltaTime)
{
    AsWheelWV(wheel)->ApplyTorque(torque, deltaTime);
}

JPH_WheeledVehicleControllerSettings *JPH_WheeledVehicleControllerSettings_Create()
{
    JPH::WheeledVehicleControllerSettings *const settings = new JPH::WheeledVehicleControllerSettings();
    settings->AddRef();

    return ToWheeledVehicleControllerSettings(settings);
}

void JPH_WheeledVehicleControllerSettings_GetEngine(const JPH_WheeledVehicleControllerSettings *settings,
                                                    JPH_VehicleEngineSettings *result)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(result);

    JPH_VehicleEngineSettings_FromJolt(result, AsWheeledVehicleControllerSettings(settings)->mEngine);
}

void JPH_WheeledVehicleControllerSettings_SetEngine(JPH_WheeledVehicleControllerSettings *settings,
                                                    const JPH_VehicleEngineSettings *value)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(value);

    JPH::VehicleEngineSettings joltEngine;
    JPH_VehicleEngineSettings_ToJolt(&joltEngine, value);

    AsWheeledVehicleControllerSettings(settings)->mEngine = joltEngine;
}

const JPH_VehicleTransmissionSettings *JPH_WheeledVehicleControllerSettings_GetTransmission(
        const JPH_WheeledVehicleControllerSettings *settings)
{
    JPH_ASSERT(settings);

    return ToVehicleTransmissionSettings(&AsWheeledVehicleControllerSettings(settings)->mTransmission);
}

void JPH_WheeledVehicleControllerSettings_SetTransmission(JPH_WheeledVehicleControllerSettings *settings,
                                                          const JPH_VehicleTransmissionSettings *value)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(value);

    AsWheeledVehicleControllerSettings(settings)->mTransmission = *AsVehicleTransmissionSettings(value);
}

uint32_t JPH_WheeledVehicleControllerSettings_GetDifferentialsCount(const JPH_WheeledVehicleControllerSettings
                                                                            *settings)
{
    return static_cast<uint32_t>(AsWheeledVehicleControllerSettings(settings)->mDifferentials.size());
}

void JPH_WheeledVehicleControllerSettings_SetDifferentialsCount(JPH_WheeledVehicleControllerSettings *settings,
                                                                const uint32_t count)
{
    AsWheeledVehicleControllerSettings(settings)->mDifferentials.resize(count);
}

void JPH_WheeledVehicleControllerSettings_GetDifferential(const JPH_WheeledVehicleControllerSettings *settings,
                                                          uint32_t index,
                                                          JPH_VehicleDifferentialSettings *result)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(result);

    JPH_VehicleDifferentialSettings_FromJolt(result,
                                             AsWheeledVehicleControllerSettings(settings)->mDifferentials[index]);
}

void JPH_WheeledVehicleControllerSettings_SetDifferential(JPH_WheeledVehicleControllerSettings *settings,
                                                          uint32_t index,
                                                          const JPH_VehicleDifferentialSettings *value)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(value);

    const JPH::VehicleDifferentialSettings joltDifferential = JPH_VehicleDifferentialSettings_ToJolt(*value);
    AsWheeledVehicleControllerSettings(settings)->mDifferentials[index] = joltDifferential;
}

void JPH_WheeledVehicleControllerSettings_SetDifferentials(JPH_WheeledVehicleControllerSettings *settings,
                                                           const JPH_VehicleDifferentialSettings *values,
                                                           uint32_t count)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(values);

    AsWheeledVehicleControllerSettings(settings)->mDifferentials.resize(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        const JPH::VehicleDifferentialSettings joltDifferential = JPH_VehicleDifferentialSettings_ToJolt(values[i]);
        AsWheeledVehicleControllerSettings(settings)->mDifferentials[i] = joltDifferential;
    }
}

float JPH_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(const JPH_WheeledVehicleControllerSettings
                                                                                   *settings)
{
    return AsWheeledVehicleControllerSettings(settings)->mDifferentialLimitedSlipRatio;
}

void JPH_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(JPH_WheeledVehicleControllerSettings
                                                                                  *settings,
                                                                          const float value)
{
    AsWheeledVehicleControllerSettings(settings)->mDifferentialLimitedSlipRatio = value;
}

void JPH_WheeledVehicleController_SetDriverInput(JPH_WheeledVehicleController *controller,
                                                 const float forward,
                                                 const float right,
                                                 const float brake,
                                                 const float handBrake)
{
    AsWheeledVehicleController(controller)->SetDriverInput(forward, right, brake, handBrake);
}

void JPH_WheeledVehicleController_SetForwardInput(JPH_WheeledVehicleController *controller, const float forward)
{
    AsWheeledVehicleController(controller)->SetForwardInput(forward);
}

float JPH_WheeledVehicleController_GetForwardInput(const JPH_WheeledVehicleController *controller)
{
    return AsWheeledVehicleController(controller)->GetForwardInput();
}

void JPH_WheeledVehicleController_SetRightInput(JPH_WheeledVehicleController *controller, const float right)
{
    AsWheeledVehicleController(controller)->SetRightInput(right);
}

float JPH_WheeledVehicleController_GetRightInput(const JPH_WheeledVehicleController *controller)
{
    return AsWheeledVehicleController(controller)->GetRightInput();
}

void JPH_WheeledVehicleController_SetBrakeInput(JPH_WheeledVehicleController *controller, const float brakeInput)
{
    AsWheeledVehicleController(controller)->SetBrakeInput(brakeInput);
}

float JPH_WheeledVehicleController_GetBrakeInput(const JPH_WheeledVehicleController *controller)
{
    return AsWheeledVehicleController(controller)->GetBrakeInput();
}

void JPH_WheeledVehicleController_SetHandBrakeInput(JPH_WheeledVehicleController *controller,
                                                    const float handBrakeInput)
{
    AsWheeledVehicleController(controller)->SetHandBrakeInput(handBrakeInput);
}

float JPH_WheeledVehicleController_GetHandBrakeInput(const JPH_WheeledVehicleController *controller)
{
    return AsWheeledVehicleController(controller)->GetHandBrakeInput();
}

float JPH_WheeledVehicleController_GetWheelSpeedAtClutch(const JPH_WheeledVehicleController *controller)
{
    return AsWheeledVehicleController(controller)->GetWheelSpeedAtClutch();
}

/* WheelSettingsTV - WheelTV - TrackedVehicleController */
JPH_WheelSettingsTV *JPH_WheelSettingsTV_Create()
{
    JPH::WheelSettingsTV *const settings = new JPH::WheelSettingsTV();
    settings->AddRef();

    return ToWheelSettingsTV(settings);
}

float JPH_WheelSettingsTV_GetLongitudinalFriction(const JPH_WheelSettingsTV *settings)
{
    return AsWheelSettingsTV(settings)->mLongitudinalFriction;
}

void JPH_WheelSettingsTV_SetLongitudinalFriction(JPH_WheelSettingsTV *settings, const float value)
{
    AsWheelSettingsTV(settings)->mLongitudinalFriction = value;
}

float JPH_WheelSettingsTV_GetLateralFriction(const JPH_WheelSettingsTV *settings)
{
    return AsWheelSettingsTV(settings)->mLateralFriction;
}

void JPH_WheelSettingsTV_SetLateralFriction(JPH_WheelSettingsTV *settings, const float value)
{
    AsWheelSettingsTV(settings)->mLateralFriction = value;
}

JPH_WheelTV *JPH_WheelTV_Create(const JPH_WheelSettingsTV *settings)
{
    JPH_ASSERT(settings);

    JPH::WheelTV *wheel = new JPH::WheelTV(*AsWheelSettingsTV(settings));
    return ToWheelTV(wheel);
}

const JPH_WheelSettingsTV *JPH_WheelTV_GetSettings(const JPH_WheelTV *wheel)
{
    return ToWheelSettingsTV(AsWheelTV(wheel)->GetSettings());
}

JPH_TrackedVehicleControllerSettings *JPH_TrackedVehicleControllerSettings_Create()
{
    JPH::TrackedVehicleControllerSettings *const settings = new JPH::TrackedVehicleControllerSettings();
    settings->AddRef();
    return ToTrackedVehicleControllerSettings(settings);
}

void JPH_TrackedVehicleControllerSettings_GetEngine(const JPH_TrackedVehicleControllerSettings *settings,
                                                    JPH_VehicleEngineSettings *result)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(result);

    JPH_VehicleEngineSettings_FromJolt(result, AsTrackedVehicleControllerSettings(settings)->mEngine);
}

void JPH_TrackedVehicleControllerSettings_SetEngine(JPH_TrackedVehicleControllerSettings *settings,
                                                    const JPH_VehicleEngineSettings *value)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(value);

    JPH::VehicleEngineSettings joltEngine;
    JPH_VehicleEngineSettings_ToJolt(&joltEngine, value);

    AsTrackedVehicleControllerSettings(settings)->mEngine = joltEngine;
}

const JPH_VehicleTransmissionSettings *JPH_TrackedVehicleControllerSettings_GetTransmission(
        const JPH_TrackedVehicleControllerSettings *settings)
{
    JPH_ASSERT(settings);
    return ToVehicleTransmissionSettings(&AsTrackedVehicleControllerSettings(settings)->mTransmission);
}

void JPH_TrackedVehicleControllerSettings_SetTransmission(JPH_TrackedVehicleControllerSettings *settings,
                                                          const JPH_VehicleTransmissionSettings *value)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(value);

    AsTrackedVehicleControllerSettings(settings)->mTransmission = *AsVehicleTransmissionSettings(value);
}

void JPH_TrackedVehicleController_SetDriverInput(JPH_TrackedVehicleController *controller,
                                                 const float forward,
                                                 const float leftRatio,
                                                 const float rightRatio,
                                                 const float brake)
{
    AsTrackedVehicleController(controller)->SetDriverInput(forward, leftRatio, rightRatio, brake);
}

float JPH_TrackedVehicleController_GetForwardInput(const JPH_TrackedVehicleController *controller)
{
    return AsTrackedVehicleController(controller)->GetForwardInput();
}

void JPH_TrackedVehicleController_SetForwardInput(JPH_TrackedVehicleController *controller, const float value)
{
    AsTrackedVehicleController(controller)->SetForwardInput(value);
}

float JPH_TrackedVehicleController_GetLeftRatio(const JPH_TrackedVehicleController *controller)
{
    return AsTrackedVehicleController(controller)->GetLeftRatio();
}

void JPH_TrackedVehicleController_SetLeftRatio(JPH_TrackedVehicleController *controller, const float value)
{
    AsTrackedVehicleController(controller)->SetLeftRatio(value);
}

float JPH_TrackedVehicleController_GetRightRatio(const JPH_TrackedVehicleController *controller)
{
    return AsTrackedVehicleController(controller)->GetRightRatio();
}

void JPH_TrackedVehicleController_SetRightRatio(JPH_TrackedVehicleController *controller, const float value)
{
    AsTrackedVehicleController(controller)->SetRightRatio(value);
}

float JPH_TrackedVehicleController_GetBrakeInput(const JPH_TrackedVehicleController *controller)
{
    return AsTrackedVehicleController(controller)->GetBrakeInput();
}

void JPH_TrackedVehicleController_SetBrakeInput(JPH_TrackedVehicleController *controller, const float value)
{
    AsTrackedVehicleController(controller)->SetBrakeInput(value);
}

/* MotorcycleController */
JPH_MotorcycleControllerSettings *JPH_MotorcycleControllerSettings_Create()
{
    JPH::MotorcycleControllerSettings *const settings = new JPH::MotorcycleControllerSettings();
    settings->AddRef();

    return ToMotorcycleControllerSettings(settings);
}

float JPH_MotorcycleControllerSettings_GetMaxLeanAngle(const JPH_MotorcycleControllerSettings *settings)
{
    return AsMotorcycleControllerSettings(settings)->mMaxLeanAngle;
}

void JPH_MotorcycleControllerSettings_SetMaxLeanAngle(JPH_MotorcycleControllerSettings *settings, const float value)
{
    AsMotorcycleControllerSettings(settings)->mMaxLeanAngle = value;
}

float JPH_MotorcycleControllerSettings_GetLeanSpringConstant(const JPH_MotorcycleControllerSettings *settings)
{
    return AsMotorcycleControllerSettings(settings)->mLeanSpringConstant;
}

void JPH_MotorcycleControllerSettings_SetLeanSpringConstant(JPH_MotorcycleControllerSettings *settings,
                                                            const float value)
{
    AsMotorcycleControllerSettings(settings)->mLeanSpringConstant = value;
}

float JPH_MotorcycleControllerSettings_GetLeanSpringDamping(const JPH_MotorcycleControllerSettings *settings)
{
    return AsMotorcycleControllerSettings(settings)->mLeanSpringDamping;
}

void JPH_MotorcycleControllerSettings_SetLeanSpringDamping(JPH_MotorcycleControllerSettings *settings,
                                                           const float value)
{
    AsMotorcycleControllerSettings(settings)->mLeanSpringDamping = value;
}

float JPH_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficient(const JPH_MotorcycleControllerSettings
                                                                                   *settings)
{
    return AsMotorcycleControllerSettings(settings)->mLeanSpringIntegrationCoefficient;
}

void JPH_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficient(JPH_MotorcycleControllerSettings *settings,
                                                                          const float value)
{
    AsMotorcycleControllerSettings(settings)->mLeanSpringIntegrationCoefficient = value;
}

float JPH_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficientDecay(const JPH_MotorcycleControllerSettings
                                                                                        *settings)
{
    return AsMotorcycleControllerSettings(settings)->mLeanSpringIntegrationCoefficientDecay;
}

void JPH_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficientDecay(JPH_MotorcycleControllerSettings
                                                                                       *settings,
                                                                               const float value)
{
    AsMotorcycleControllerSettings(settings)->mLeanSpringIntegrationCoefficientDecay = value;
}

float JPH_MotorcycleControllerSettings_GetLeanSmoothingFactor(const JPH_MotorcycleControllerSettings *settings)
{
    return AsMotorcycleControllerSettings(settings)->mLeanSmoothingFactor;
}

void JPH_MotorcycleControllerSettings_SetLeanSmoothingFactor(JPH_MotorcycleControllerSettings *settings,
                                                             const float value)
{
    AsMotorcycleControllerSettings(settings)->mLeanSmoothingFactor = value;
}

float JPH_MotorcycleController_GetWheelBase(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->GetWheelBase();
}

bool JPH_MotorcycleController_IsLeanControllerEnabled(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->IsLeanControllerEnabled();
}

void JPH_MotorcycleController_EnableLeanController(JPH_MotorcycleController *controller, const bool value)
{
    AsMotorcycleController(controller)->EnableLeanController(value);
}

bool JPH_MotorcycleController_IsLeanSteeringLimitEnabled(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->IsLeanSteeringLimitEnabled();
}

void JPH_MotorcycleController_EnableLeanSteeringLimit(JPH_MotorcycleController *controller, const bool value)
{
    AsMotorcycleController(controller)->EnableLeanSteeringLimit(value);
}

float JPH_MotorcycleController_GetLeanSpringConstant(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->GetLeanSpringConstant();
}

void JPH_MotorcycleController_SetLeanSpringConstant(JPH_MotorcycleController *controller, const float value)
{
    AsMotorcycleController(controller)->SetLeanSpringConstant(value);
}

float JPH_MotorcycleController_GetLeanSpringDamping(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->GetLeanSpringDamping();
}

void JPH_MotorcycleController_SetLeanSpringDamping(JPH_MotorcycleController *controller, const float value)
{
    AsMotorcycleController(controller)->SetLeanSpringDamping(value);
}

float JPH_MotorcycleController_GetLeanSpringIntegrationCoefficient(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->GetLeanSpringIntegrationCoefficient();
}

void JPH_MotorcycleController_SetLeanSpringIntegrationCoefficient(JPH_MotorcycleController *controller,
                                                                  const float value)
{
    AsMotorcycleController(controller)->SetLeanSpringIntegrationCoefficient(value);
}

float JPH_MotorcycleController_GetLeanSpringIntegrationCoefficientDecay(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->GetLeanSpringIntegrationCoefficientDecay();
}

void JPH_MotorcycleController_SetLeanSpringIntegrationCoefficientDecay(JPH_MotorcycleController *controller,
                                                                       const float value)
{
    AsMotorcycleController(controller)->SetLeanSpringIntegrationCoefficientDecay(value);
}

float JPH_MotorcycleController_GetLeanSmoothingFactor(const JPH_MotorcycleController *controller)
{
    return AsMotorcycleController(controller)->GetLeanSmoothingFactor();
}

void JPH_MotorcycleController_SetLeanSmoothingFactor(JPH_MotorcycleController *controller, const float value)
{
    AsMotorcycleController(controller)->SetLeanSmoothingFactor(value);
}
