// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

// ReSharper disable CppDFATimeOver

#include <cfloat>
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <joltc/enums.h>
#include <joltc/Geometry/AABox.h>
#include <joltc/joltc.h>
#include <joltc/Math/Mat44.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/Body.h>
#include <joltc/Physics/Body/BodyFilter.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Body/BodyInterface.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <joltc/Physics/Collision/NarrowPhaseQuery.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <joltc/Physics/Collision/Shape/SubShapeIDPair.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>
#include <joltc/types.h>
#include <new>
#include <thread>
#include <Jolt/Jolt.h>
#include <Jolt/Core/Array.h>
#include <Jolt/Core/Color.h>
#include <Jolt/Core/Core.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/JobSystemWithBarrier.h>
#include <Jolt/Core/LinearCurve.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Core/RTTI.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/Geometry/Triangle.h>
#include <Jolt/Math/Float3.h>
#include <Jolt/Math/MathTypes.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Trigonometry.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Vector.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyType.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterBase.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/VehicleEngine.h>
#include <Jolt/Physics/Vehicle/VehicleTrack.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/Physics/Vehicle/Wheel.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Math/Mat44.hpp>
#include <Math/Quat.hpp>
#include <Math/RVec3.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/Body.hpp>
#include <Physics/Body/BodyFilter.hpp>
#include <Physics/Collision/CollideShape.hpp>
#include <Physics/Collision/PhysicsMaterial.hpp>
#include <Physics/Collision/Shape/Shape.hpp>
#include <Physics/Collision/ShapeCast.hpp>
#include <Physics/Collision/ShapeFilter.hpp>

#ifdef JPH_DEBUG_RENDERER
#include <Geometry/AABox.hpp>
#endif

// NOLINTBEGIN(*-macro-parentheses)
#define DEF_MAP_DECL(JoltType, c_type) \
    static inline const JPH::JoltType *As##JoltType(const c_type *t) \
    { \
        return reinterpret_cast<const JPH::JoltType *>(t); \
    } \
    static inline JPH::JoltType *As##JoltType(c_type *t) \
    { \
        return reinterpret_cast<JPH::JoltType *>(t); \
    } \
    static inline const c_type *To##JoltType(const JPH::JoltType *t) \
    { \
        return reinterpret_cast<const c_type *>(t); \
    } \
    static inline c_type *To##JoltType(JPH::JoltType *t) \
    { \
        return reinterpret_cast<c_type *>(t); \
    }
// NOLINTEND(*-macro-parentheses)

DEF_MAP_DECL(ContactManifold, JPH_ContactManifold)
DEF_MAP_DECL(BodyLockInterface, JPH_BodyLockInterface)
DEF_MAP_DECL(EmptyShapeSettings, JPH_EmptyShapeSettings)
DEF_MAP_DECL(CompoundShape, JPH_CompoundShape)
DEF_MAP_DECL(CompoundShapeSettings, JPH_CompoundShapeSettings)
DEF_MAP_DECL(MutableCompoundShape, JPH_MutableCompoundShape)
DEF_MAP_DECL(MutableCompoundShapeSettings, JPH_MutableCompoundShapeSettings)
DEF_MAP_DECL(MeshShape, JPH_MeshShape)
DEF_MAP_DECL(MeshShapeSettings, JPH_MeshShapeSettings)
DEF_MAP_DECL(HeightFieldShape, JPH_HeightFieldShape)
DEF_MAP_DECL(HeightFieldShapeSettings, JPH_HeightFieldShapeSettings)
DEF_MAP_DECL(ConvexShape, JPH_ConvexShape)
DEF_MAP_DECL(ConvexShapeSettings, JPH_ConvexShapeSettings)
DEF_MAP_DECL(ConvexHullShape, JPH_ConvexHullShape)
DEF_MAP_DECL(ConvexHullShapeSettings, JPH_ConvexHullShapeSettings)
DEF_MAP_DECL(SphereShape, JPH_SphereShape)
DEF_MAP_DECL(SphereShapeSettings, JPH_SphereShapeSettings)
DEF_MAP_DECL(BoxShape, JPH_BoxShape)
DEF_MAP_DECL(BoxShapeSettings, JPH_BoxShapeSettings)
DEF_MAP_DECL(PlaneShape, JPH_PlaneShape)
DEF_MAP_DECL(PlaneShapeSettings, JPH_PlaneShapeSettings)
DEF_MAP_DECL(CapsuleShape, JPH_CapsuleShape)
DEF_MAP_DECL(CapsuleShapeSettings, JPH_CapsuleShapeSettings)
DEF_MAP_DECL(CylinderShape, JPH_CylinderShape)
DEF_MAP_DECL(CylinderShapeSettings, JPH_CylinderShapeSettings)
DEF_MAP_DECL(TriangleShape, JPH_TriangleShape)
DEF_MAP_DECL(TriangleShapeSettings, JPH_TriangleShapeSettings)
DEF_MAP_DECL(TaperedCylinderShape, JPH_TaperedCylinderShape)
DEF_MAP_DECL(TaperedCylinderShapeSettings, JPH_TaperedCylinderShapeSettings)
DEF_MAP_DECL(TaperedCapsuleShape, JPH_TaperedCapsuleShape)
DEF_MAP_DECL(TaperedCapsuleShapeSettings, JPH_TaperedCapsuleShapeSettings)
DEF_MAP_DECL(StaticCompoundShapeSettings, JPH_StaticCompoundShapeSettings)
DEF_MAP_DECL(DecoratedShape, JPH_DecoratedShape)
DEF_MAP_DECL(RotatedTranslatedShape, JPH_RotatedTranslatedShape)
DEF_MAP_DECL(RotatedTranslatedShapeSettings, JPH_RotatedTranslatedShapeSettings)
DEF_MAP_DECL(ScaledShape, JPH_ScaledShape)
DEF_MAP_DECL(ScaledShapeSettings, JPH_ScaledShapeSettings)
DEF_MAP_DECL(OffsetCenterOfMassShape, JPH_OffsetCenterOfMassShape)
DEF_MAP_DECL(OffsetCenterOfMassShapeSettings, JPH_OffsetCenterOfMassShapeSettings)
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
DEF_MAP_DECL(SkeletonPose, JPH_SkeletonPose)
DEF_MAP_DECL(SkeletalAnimation, JPH_SkeletalAnimation)
DEF_MAP_DECL(SkeletonMapper, JPH_SkeletonMapper)
DEF_MAP_DECL(RagdollSettings, JPH_RagdollSettings)
DEF_MAP_DECL(Ragdoll, JPH_Ragdoll)
DEF_MAP_DECL(GroupFilterTable, JPH_GroupFilterTable)
DEF_MAP_DECL(PhysicsStepListener, JPH_PhysicsStepListener)
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
DEF_MAP_DECL(VehicleTrack, JPH_VehicleTrack)
DEF_MAP_DECL(VehicleEngine, JPH_VehicleEngine)
DEF_MAP_DECL(VehicleTransmission, JPH_VehicleTransmission)
DEF_MAP_DECL(VehicleTransmissionSettings, JPH_VehicleTransmissionSettings)
DEF_MAP_DECL(VehicleCollisionTester, JPH_VehicleCollisionTester)
DEF_MAP_DECL(VehicleCollisionTesterRay, JPH_VehicleCollisionTesterRay)
DEF_MAP_DECL(VehicleCollisionTesterCastSphere, JPH_VehicleCollisionTesterCastSphere)
DEF_MAP_DECL(VehicleCollisionTesterCastCylinder, JPH_VehicleCollisionTesterCastCylinder)
DEF_MAP_DECL(VehicleConstraint, JPH_VehicleConstraint)
DEF_MAP_DECL(LinearCurve, JPH_LinearCurve)

// Callback for traces, connect this to your own trace function if you have one
static JPH_TraceFunc s_TraceFunc = nullptr;

// NOLINTBEGIN clang-tidy REALLY dislikes this function but because of Jolt's API there isn't really a better way
static void TraceImpl(const char *fmt, ...)
{
    // Format the message
    va_list list;
    va_start(list, fmt);
    char buffer[1024];
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
// NOLINTEND

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
    std::cout << std::endl; // NOLINT(*-avoid-endl)

    // Breakpoint
    return true;
}

#endif // JPH_ENABLE_ASSERTS

// TODO: The functions in this region could use with optimization from std::copy_n
#pragma region FromJoltFunctions

static inline void FromJolt(const JPH::Plane &value, JPH_Plane *result)
{
    FromJolt(value.GetNormal(), &result->normal);
    result->distance = value.GetConstant();
}

static inline void FromJolt(const JPH::LinearCurve::Point &jolt, JPH_Point *result)
{
    result->x = jolt.mX;
    result->y = jolt.mY;
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

static inline void FromJolt(const JPH::ContactSettings &jolt, JPH_ContactSettings *result)
{
    JPH_ASSERT(result);

    result->combinedFriction = jolt.mCombinedFriction;
    result->combinedRestitution = jolt.mCombinedRestitution;
    result->invMassScale1 = jolt.mInvMassScale1;
    result->invInertiaScale1 = jolt.mInvInertiaScale1;
    result->invMassScale2 = jolt.mInvMassScale2;
    result->invInertiaScale2 = jolt.mInvInertiaScale2;
    result->isSensor = jolt.mIsSensor;
    FromJolt(jolt.mRelativeLinearSurfaceVelocity, &result->relativeLinearSurfaceVelocity);
    FromJolt(jolt.mRelativeAngularSurfaceVelocity, &result->relativeAngularSurfaceVelocity);
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

#pragma endregion FromJoltFunctions

// TODO: The functions in this region could use with optimization from std::copy_n
#pragma region ToJoltFunctions

static inline JPH::Plane ToJolt(const JPH_Plane *value)
{
    return {ToJolt(value->normal), value->distance};
}

static inline JPH::Float3 ToJoltFloat3(const Vector3 &vec)
{
    return {vec.x, vec.y, vec.z};
}

static inline JPH::AABox ToJolt(const JPH_AABox *value)
{
    return JPH::AABox{ToJolt(value->min), ToJolt(value->max)};
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

static inline JPH::ContactSettings ToJolt(const JPH_ContactSettings *settings)
{
    JPH::ContactSettings result{};
    result.mCombinedFriction = settings->combinedFriction;
    result.mCombinedRestitution = settings->combinedRestitution;
    result.mInvMassScale1 = settings->invMassScale1;
    result.mInvInertiaScale1 = settings->invInertiaScale1;
    result.mInvMassScale2 = settings->invMassScale2;
    result.mInvInertiaScale2 = settings->invInertiaScale2;
    result.mIsSensor = settings->isSensor;
    result.mRelativeLinearSurfaceVelocity = ToJolt(settings->relativeLinearSurfaceVelocity);
    result.mRelativeAngularSurfaceVelocity = ToJolt(settings->relativeAngularSurfaceVelocity);
    return result;
}

#pragma endregion ToJoltFunctions

void JPH_RayCast_GetPointOnRay(const Vector3 *origin, const Vector3 *direction, float fraction, Vector3 *result)
{
    JPH_ASSERT(origin);
    JPH_ASSERT(direction);
    JPH_ASSERT(result);

    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::Vec3 point = ray.GetPointOnRay(fraction);
    FromJolt(point, result);
}

void JPH_RRayCast_GetPointOnRay(const JPH_RVec3 *origin, const Vector3 *direction, float fraction, JPH_RVec3 *result)
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
static bool s_initialized = false;

class JobSystemCallback final: public JPH::JobSystemWithBarrier
{
    public:
        explicit JobSystemCallback(const JPH_JobSystemConfig *config): mConfig(*config)
        {
            Init(config->maxBarriers > 0 ? config->maxBarriers : JPH::cMaxPhysicsBarriers);
            mJobs.Init(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsJobs);
        }

        JobHandle CreateJob(const char *name,
                            JPH::ColorArg color,
                            const JobFunction &callback,
                            uint32_t dependencies = 0) override
        {
            uint32_t index = 0;

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

        [[nodiscard]] int GetMaxConcurrency() const override
        {
            return mConfig.maxConcurrency;
        }

    protected:
        void FreeJob(Job *job) override
        {
            mJobs.DestructObject(job);
        }

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

JPH_JobSystem *JPH_JobSystemThreadPool_Create(const JPH_JobSystemThreadPoolConfig *config)
{
    JPH_JobSystemThreadPoolConfig createConfig{};
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

void JPH_Init()
{
    if (s_initialized)
    {
        return;
    }

    JPH::RegisterDefaultAllocator();

    // TODO
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    // Create a factory
    JPH::Factory::sInstance = new JPH::Factory();

    // Register all Jolt physics types
    JPH::RegisterTypes();

    // Init temp allocator
    s_TempAllocator = new JPH::TempAllocatorImplWithMallocFallback(32 * 1024 * 1024);
    s_initialized = true;
}

void JPH_Shutdown()
{
    if (!s_initialized)
    {
        return;
    }

    delete s_TempAllocator;
    s_TempAllocator = nullptr;

    // Unregisters all types with the factory and cleans up the default material
    JPH::UnregisterTypes();

    // Destroy the factory
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    s_initialized = false;
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

void JPH_CollisionEstimationResult_FreeMembers(const JPH_CollisionEstimationResult *result)
{
    if (result->impulseCount != 0)
    {
        delete[] result->impulses;
    }
}

/* JPH_BroadPhaseLayerInterface */
class ManagedBroadPhaseLayerInterface final: public JPH::BroadPhaseLayerInterface
{
    public:
        ManagedBroadPhaseLayerInterface(const uint32_t broadPhaseLayerCount,
                                        const JPH_BroadPhaseLayerInterface_Impl *impl):
            impl(impl),
            broadPhaseLayerCount(broadPhaseLayerCount)
        {}

        [[nodiscard]] uint32_t GetNumBroadPhaseLayers() const override
        {
            return broadPhaseLayerCount;
        }
        [[nodiscard]] JPH::BroadPhaseLayer GetBroadPhaseLayer(const JPH::ObjectLayer inLayer) const override
        {
            if (impl != nullptr && impl->GetBroadPhaseLayer != nullptr)
            {
                return static_cast<JPH::BroadPhaseLayer>(impl->GetBroadPhaseLayer(inLayer));
            }
            return JPH::cBroadPhaseLayerInvalid;
        }

        const JPH_BroadPhaseLayerInterface_Impl *impl{};
        uint32_t broadPhaseLayerCount{};
};

JPH_BroadPhaseLayerInterface *JPH_BroadPhaseLayerInterface_Create(const uint32_t broadPhaseLayerCount,
                                                                  const JPH_BroadPhaseLayerInterface_Impl *impl)
{
    ManagedBroadPhaseLayerInterface *const layerInterface = new ManagedBroadPhaseLayerInterface(broadPhaseLayerCount,
                                                                                                impl);
    return reinterpret_cast<JPH_BroadPhaseLayerInterface *>(layerInterface);
}

void JPH_BroadPhaseLayerInterface_Destroy(JPH_BroadPhaseLayerInterface *broadPhaseLayerInterface)
{
    delete reinterpret_cast<ManagedBroadPhaseLayerInterface *>(broadPhaseLayerInterface);
}

void JPH_BroadPhaseLayerInterface_SetImpl(JPH_BroadPhaseLayerInterface *broadPhaseLayerInterface,
                                          const JPH_BroadPhaseLayerInterface_Impl *impl)
{
    JPH_ASSERT(broadPhaseLayerInterface);
    reinterpret_cast<ManagedBroadPhaseLayerInterface *>(broadPhaseLayerInterface)->impl = impl;
}

/* JPH_ObjectLayerPairFilter */
class ManagedObjectLayerPairFilter final: public JPH::ObjectLayerPairFilter
{
    public:
        explicit ManagedObjectLayerPairFilter(const JPH_ObjectLayerPairFilter_Impl *impl): impl(impl) {}

        [[nodiscard]] bool ShouldCollide(const JPH::ObjectLayer inLayer1,
                                         const JPH::ObjectLayer inLayer2) const override
        {
            if (impl != nullptr && impl->ShouldCollide != nullptr)
            {
                return impl->ShouldCollide(inLayer1, inLayer2);
            }
            return true;
        }

        const JPH_ObjectLayerPairFilter_Impl *impl{};
};

JPH_ObjectLayerPairFilter *JPH_ObjectLayerPairFilter_Create(const JPH_ObjectLayerPairFilter_Impl *impl)
{
    ManagedObjectLayerPairFilter *const filter = new ManagedObjectLayerPairFilter(impl);
    return reinterpret_cast<JPH_ObjectLayerPairFilter *>(filter);
}

void JPH_ObjectLayerPairFilter_Destroy(JPH_ObjectLayerPairFilter *filter)
{
    delete reinterpret_cast<ManagedObjectLayerPairFilter *>(filter);
}

void JPH_ObjectLayerPairFilter_SetImpl(JPH_ObjectLayerPairFilter *filter, const JPH_ObjectLayerPairFilter_Impl *impl)
{
    JPH_ASSERT(filter);
    reinterpret_cast<ManagedObjectLayerPairFilter *>(filter)->impl = impl;
}

/* JPH_ObjectVsBroadPhaseLayerFilter */
class ManagedObjectVsBroadPhaseLayerFilter final: public JPH::ObjectVsBroadPhaseLayerFilter
{
    public:
        explicit ManagedObjectVsBroadPhaseLayerFilter(const JPH_ObjectVsBroadPhaseLayerFilter_Impl *impl): impl(impl) {}

        [[nodiscard]] bool ShouldCollide(const JPH::ObjectLayer inLayer1,
                                         const JPH::BroadPhaseLayer inLayer2) const override
        {
            if (impl != nullptr && impl->ShouldCollide != nullptr)
            {
                return impl->ShouldCollide(inLayer1, static_cast<JPH_BroadPhaseLayer>(inLayer2));
            }
            return true;
        }

        const JPH_ObjectVsBroadPhaseLayerFilter_Impl *impl{};
};

JPH_ObjectVsBroadPhaseLayerFilter *JPH_ObjectVsBroadPhaseLayerFilter_Create(const JPH_ObjectVsBroadPhaseLayerFilter_Impl
                                                                                    *impl)
{
    ManagedObjectVsBroadPhaseLayerFilter *const filter = new ManagedObjectVsBroadPhaseLayerFilter(impl);
    return reinterpret_cast<JPH_ObjectVsBroadPhaseLayerFilter *>(filter);
}

void JPH_ObjectVsBroadPhaseLayerFilter_Destroy(JPH_ObjectVsBroadPhaseLayerFilter *filter)
{
    delete reinterpret_cast<ManagedObjectVsBroadPhaseLayerFilter *>(filter);
}

void JPH_ObjectVsBroadPhaseLayerFilter_SetImpl(JPH_ObjectVsBroadPhaseLayerFilter *filter,
                                               const JPH_ObjectVsBroadPhaseLayerFilter_Impl *impl)
{
    JPH_ASSERT(filter);
    reinterpret_cast<ManagedObjectVsBroadPhaseLayerFilter *>(filter)->impl = impl;
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
    system->broadPhaseLayerInterface =
            reinterpret_cast<JPH::BroadPhaseLayerInterface *>(settings->broadPhaseLayerInterface);
    system->objectLayerPairFilter = reinterpret_cast<JPH::ObjectLayerPairFilter *>(settings->objectLayerPairFilter);
    system->objectVsBroadPhaseLayerFilter =
            reinterpret_cast<JPH::ObjectVsBroadPhaseLayerFilter *>(settings->objectVsBroadPhaseLayerFilter);

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

/* JPH_SimShapeFilter */
class ManagedSimShapeFilter final: public JPH::SimShapeFilter
{
    public:
        static const JPH_SimShapeFilter_Impl *s_Impl;
        void *userData = nullptr;

        explicit ManagedSimShapeFilter(void *userData_): userData(userData_) {}

        bool ShouldCollide([[maybe_unused]] const JPH::Body &inBody1,
                           [[maybe_unused]] const JPH::Shape *inShape1,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape1,
                           [[maybe_unused]] const JPH::Body &inBody2,
                           [[maybe_unused]] const JPH::Shape *inShape2,
                           [[maybe_unused]] const JPH::SubShapeID &inSubShapeIDOfShape2) const override
        {
            if (s_Impl != nullptr && s_Impl->ShouldCollide != nullptr)
            {
                const uint32_t subShapeIDOfShape1 = inSubShapeIDOfShape1.GetValue();
                const uint32_t subShapeIDOfShape2 = inSubShapeIDOfShape2.GetValue();
                return s_Impl->ShouldCollide(userData,
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

const JPH_SimShapeFilter_Impl *ManagedSimShapeFilter::s_Impl = nullptr;

void JPH_SimShapeFilter_SetImpl(const JPH_SimShapeFilter_Impl *impl)
{
    ManagedSimShapeFilter::s_Impl = impl;
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
float JPH_Sin(const float value)
{
    return JPH::Sin(value);
}

float JPH_Cos(const float value)
{
    return JPH::Cos(value);
}

/* GroupFilter/GroupFilterTable */
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

/* ConvexShape */
float JPH_ConvexShapeSettings_GetDensity(const JPH_ConvexShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsConvexShapeSettings(settings)->mDensity;
}

void JPH_ConvexShapeSettings_SetDensity(JPH_ConvexShapeSettings *settings, const float value)
{
    JPH_ASSERT(settings);
    AsConvexShapeSettings(settings)->SetDensity(value);
}

float JPH_ConvexShape_GetDensity(const JPH_ConvexShape *shape)
{
    JPH_ASSERT(shape);
    return AsConvexShape(shape)->GetDensity();
}

void JPH_ConvexShape_SetDensity(JPH_ConvexShape *shape, const float density)
{
    JPH_ASSERT(shape);
    AsConvexShape(shape)->SetDensity(density);
}

/* BoxShape */
JPH_BoxShapeSettings *JPH_BoxShapeSettings_Create(const Vector3 *halfExtent, const float convexRadius)
{
    JPH::BoxShapeSettings *const settings = new JPH::BoxShapeSettings(ToJolt(halfExtent), convexRadius);
    settings->AddRef();

    return ToBoxShapeSettings(settings);
}

JPH_BoxShape *JPH_BoxShapeSettings_CreateShape(const JPH_BoxShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::BoxShapeSettings::ShapeResult shapeResult = AsBoxShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_BoxShape *>(shape);
}

JPH_BoxShape *JPH_BoxShape_Create(const Vector3 *halfExtent, const float convexRadius)
{
    JPH::BoxShape *const shape = new JPH::BoxShape(ToJolt(halfExtent), convexRadius);
    shape->AddRef();

    return ToBoxShape(shape);
}

void JPH_BoxShape_GetHalfExtent(const JPH_BoxShape *shape, Vector3 *halfExtent)
{
    JPH_ASSERT(shape);
    const JPH::Vec3 joltVector = AsBoxShape(shape)->GetHalfExtent();
    FromJolt(joltVector, halfExtent);
}

float JPH_BoxShape_GetConvexRadius(const JPH_BoxShape *shape)
{
    JPH_ASSERT(shape);
    return AsBoxShape(shape)->GetConvexRadius();
}

/* SphereShapeSettings */
JPH_SphereShapeSettings *JPH_SphereShapeSettings_Create(const float radius)
{
    JPH::SphereShapeSettings *const settings = new JPH::SphereShapeSettings(radius);
    settings->AddRef();

    return ToSphereShapeSettings(settings);
}

JPH_SphereShape *JPH_SphereShapeSettings_CreateShape(const JPH_SphereShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::SphereShapeSettings::ShapeResult shapeResult = AsSphereShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_SphereShape *>(shape);
}

float JPH_SphereShapeSettings_GetRadius(const JPH_SphereShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsSphereShapeSettings(settings)->mRadius;
}

void JPH_SphereShapeSettings_SetRadius(JPH_SphereShapeSettings *settings, float radius)
{
    JPH_ASSERT(settings);
    AsSphereShapeSettings(settings)->mRadius = radius;
}

JPH_SphereShape *JPH_SphereShape_Create(const float radius)
{
    JPH::SphereShape *const shape = new JPH::SphereShape(radius);
    shape->AddRef();

    return ToSphereShape(shape);
}

float JPH_SphereShape_GetRadius(const JPH_SphereShape *shape)
{
    JPH_ASSERT(shape);
    return AsSphereShape(shape)->GetRadius();
}

/* PlaneShape */
JPH_PlaneShapeSettings *JPH_PlaneShapeSettings_Create(const JPH_Plane *plane,
                                                      const JPH_PhysicsMaterial *material,
                                                      const float halfExtent)
{
    const JPH::PhysicsMaterial *joltMaterial = AsPhysicsMaterial(material);

    JPH::PlaneShapeSettings *const settings = new JPH::PlaneShapeSettings(ToJolt(plane), joltMaterial, halfExtent);
    settings->AddRef();

    return ToPlaneShapeSettings(settings);
}

JPH_PlaneShape *JPH_PlaneShapeSettings_CreateShape(const JPH_PlaneShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::PlaneShapeSettings::ShapeResult shapeResult = AsPlaneShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_PlaneShape *>(shape);
}

JPH_PlaneShape *JPH_PlaneShape_Create(const JPH_Plane *plane,
                                      const JPH_PhysicsMaterial *material,
                                      const float halfExtent)
{
    const JPH::PhysicsMaterial *joltMaterial = AsPhysicsMaterial(material);

    JPH::PlaneShape *const shape = new JPH::PlaneShape(ToJolt(plane), joltMaterial, halfExtent);
    shape->AddRef();

    return ToPlaneShape(shape);
}

void JPH_PlaneShape_GetPlane(const JPH_PlaneShape *shape, JPH_Plane *result)
{
    JPH_ASSERT(shape);
    FromJolt(AsPlaneShape(shape)->GetPlane(), result);
}

float JPH_PlaneShape_GetHalfExtent(const JPH_PlaneShape *shape)
{
    JPH_ASSERT(shape);
    return AsPlaneShape(shape)->GetHalfExtent();
}

/* TriangleShape */
JPH_TriangleShapeSettings *JPH_TriangleShapeSettings_Create(const Vector3 *v1,
                                                            const Vector3 *v2,
                                                            const Vector3 *v3,
                                                            const float convexRadius)
{
    JPH::TriangleShapeSettings *const settings = new JPH::TriangleShapeSettings(ToJolt(v1),
                                                                                ToJolt(v2),
                                                                                ToJolt(v3),
                                                                                convexRadius);
    settings->AddRef();

    return ToTriangleShapeSettings(settings);
}

JPH_TriangleShape *JPH_TriangleShapeSettings_CreateShape(const JPH_TriangleShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::TriangleShapeSettings::ShapeResult shapeResult = AsTriangleShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_TriangleShape *>(shape);
}

JPH_TriangleShape *JPH_TriangleShape_Create(const Vector3 *v1,
                                            const Vector3 *v2,
                                            const Vector3 *v3,
                                            const float convexRadius)
{
    JPH::TriangleShape *const shape = new JPH::TriangleShape(ToJolt(v1), ToJolt(v2), ToJolt(v3), convexRadius);
    shape->AddRef();

    return ToTriangleShape(shape);
}

float JPH_TriangleShape_GetConvexRadius(const JPH_TriangleShape *shape)
{
    JPH_ASSERT(shape);
    return AsTriangleShape(shape)->GetConvexRadius();
}

void JPH_TriangleShape_GetVertex1(const JPH_TriangleShape *shape, Vector3 *result)
{
    JPH_ASSERT(shape);
    FromJolt(AsTriangleShape(shape)->GetVertex1(), result);
}

void JPH_TriangleShape_GetVertex2(const JPH_TriangleShape *shape, Vector3 *result)
{
    JPH_ASSERT(shape);
    FromJolt(AsTriangleShape(shape)->GetVertex2(), result);
}

void JPH_TriangleShape_GetVertex3(const JPH_TriangleShape *shape, Vector3 *result)
{
    JPH_ASSERT(shape);
    FromJolt(AsTriangleShape(shape)->GetVertex3(), result);
}

/* CapsuleShapeSettings */
JPH_CapsuleShapeSettings *JPH_CapsuleShapeSettings_Create(const float halfHeightOfCylinder, const float radius)
{
    JPH::CapsuleShapeSettings *const settings = new JPH::CapsuleShapeSettings(halfHeightOfCylinder, radius);
    settings->AddRef();

    return ToCapsuleShapeSettings(settings);
}

JPH_CapsuleShape *JPH_CapsuleShapeSettings_CreateShape(const JPH_CapsuleShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::CapsuleShapeSettings::ShapeResult shapeResult = AsCapsuleShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_CapsuleShape *>(shape);
}

JPH_CapsuleShape *JPH_CapsuleShape_Create(const float halfHeightOfCylinder, const float radius)
{
    JPH::CapsuleShape *const shape = new JPH::CapsuleShape(halfHeightOfCylinder, radius, nullptr);
    shape->AddRef();

    return ToCapsuleShape(shape);
}

float JPH_CapsuleShape_GetRadius(const JPH_CapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return AsCapsuleShape(shape)->GetRadius();
}

float JPH_CapsuleShape_GetHalfHeightOfCylinder(const JPH_CapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return AsCapsuleShape(shape)->GetHalfHeightOfCylinder();
}

/* CylinderShapeSettings */
JPH_CylinderShapeSettings *JPH_CylinderShapeSettings_Create(const float halfHeight,
                                                            const float radius,
                                                            const float convexRadius)
{
    JPH::CylinderShapeSettings *const settings = new JPH::CylinderShapeSettings(halfHeight, radius, convexRadius);
    settings->AddRef();

    return ToCylinderShapeSettings(settings);
}

JPH_CylinderShape *JPH_CylinderShapeSettings_CreateShape(const JPH_CylinderShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::CylinderShapeSettings::ShapeResult shapeResult = AsCylinderShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_CylinderShape *>(shape);
}

JPH_CylinderShape *JPH_CylinderShape_Create(const float halfHeight, const float radius)
{
    JPH::CylinderShape *const shape = new JPH::CylinderShape(halfHeight, radius, 0.f, nullptr);
    shape->AddRef();

    return ToCylinderShape(shape);
}

float JPH_CylinderShape_GetRadius(const JPH_CylinderShape *shape)
{
    JPH_ASSERT(shape);
    return AsCylinderShape(shape)->GetRadius();
}

float JPH_CylinderShape_GetHalfHeight(const JPH_CylinderShape *shape)
{
    JPH_ASSERT(shape);
    return AsCylinderShape(shape)->GetHalfHeight();
}

/* TaperedCylinderShape */
JPH_TaperedCylinderShapeSettings *JPH_TaperedCylinderShapeSettings_Create(
        const float halfHeightOfTaperedCylinder,
        const float topRadius,
        const float bottomRadius,
        const float convexRadius /* = cDefaultConvexRadius*/,
        const JPH_PhysicsMaterial *material /* = NULL*/)
{
    const JPH::PhysicsMaterial *joltMaterial = AsPhysicsMaterial(material);

    JPH::TaperedCylinderShapeSettings *const settings =
            new JPH::TaperedCylinderShapeSettings(halfHeightOfTaperedCylinder,
                                                  topRadius,
                                                  bottomRadius,
                                                  convexRadius,
                                                  joltMaterial);
    settings->AddRef();

    return ToTaperedCylinderShapeSettings(settings);
}

JPH_TaperedCylinderShape *JPH_TaperedCylinderShapeSettings_CreateShape(const JPH_TaperedCylinderShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::TaperedCylinderShapeSettings::ShapeResult shapeResult =
            AsTaperedCylinderShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_TaperedCylinderShape *>(shape);
}

float JPH_TaperedCylinderShape_GetTopRadius(const JPH_TaperedCylinderShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCylinderShape(shape)->GetTopRadius();
}

float JPH_TaperedCylinderShape_GetBottomRadius(const JPH_TaperedCylinderShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCylinderShape(shape)->GetBottomRadius();
}

float JPH_TaperedCylinderShape_GetConvexRadius(const JPH_TaperedCylinderShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCylinderShape(shape)->GetConvexRadius();
}

float JPH_TaperedCylinderShape_GetHalfHeight(const JPH_TaperedCylinderShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCylinderShape(shape)->GetHalfHeight();
}

/* ConvexHullShape */
JPH_ConvexHullShapeSettings *JPH_ConvexHullShapeSettings_Create(const Vector3 *points,
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

    return ToConvexHullShapeSettings(settings);
}

JPH_ConvexHullShape *JPH_ConvexHullShapeSettings_CreateShape(const JPH_ConvexHullShapeSettings *settings)
{
    const JPH::ConvexHullShapeSettings::ShapeResult shapeResult = AsConvexHullShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_ConvexHullShape *>(shape);
}

JPH_ConvexHullShape *JPH_ConvexHullShape_Create(const Vector3 *points,
                                                const uint32_t pointsCount,
                                                const float maxConvexRadius)
{
    JPH::Array<JPH::Vec3> joltPoints;
    joltPoints.reserve(pointsCount);

    for (uint32_t i = 0; i < pointsCount; i++)
    {
        joltPoints.push_back(ToJolt(&points[i]));
    }

    const JPH::ConvexHullShapeSettings settings(joltPoints, maxConvexRadius);
    JPH::Shape *shape = settings.Create().Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_ConvexHullShape *>(shape);
}

uint32_t JPH_ConvexHullShape_GetNumPoints(const JPH_ConvexHullShape *shape)
{
    JPH_ASSERT(shape);
    return AsConvexHullShape(shape)->GetNumPoints();
}

void JPH_ConvexHullShape_GetPoint(const JPH_ConvexHullShape *shape, const uint32_t index, Vector3 *result)
{
    JPH_ASSERT(shape);
    FromJolt(AsConvexHullShape(shape)->GetPoint(index), result);
}

uint32_t JPH_ConvexHullShape_GetNumFaces(const JPH_ConvexHullShape *shape)
{
    JPH_ASSERT(shape);
    return AsConvexHullShape(shape)->GetNumFaces();
}

uint32_t JPH_ConvexHullShape_GetNumVerticesInFace(const JPH_ConvexHullShape *shape, const uint32_t faceIndex)
{
    JPH_ASSERT(shape);
    return reinterpret_cast<const JPH::ConvexHullShape *>(shape)->GetNumVerticesInFace(faceIndex);
}

uint32_t JPH_ConvexHullShape_GetFaceVertices(const JPH_ConvexHullShape *shape,
                                             const uint32_t faceIndex,
                                             const uint32_t maxVertices,
                                             uint32_t *vertices)
{
    JPH_ASSERT(shape);
    return AsConvexHullShape(shape)->GetFaceVertices(faceIndex, maxVertices, vertices);
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

JPH_MeshShapeSettings *JPH_MeshShapeSettings_Create2(const Vector3 *vertices,
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
    JPH_ASSERT(settings);
    return AsMeshShapeSettings(settings)->mMaxTrianglesPerLeaf;
}

void JPH_MeshShapeSettings_SetMaxTrianglesPerLeaf(JPH_MeshShapeSettings *settings, const uint32_t value)
{
    JPH_ASSERT(settings);
    AsMeshShapeSettings(settings)->mMaxTrianglesPerLeaf = value;
}
float JPH_MeshShapeSettings_GetActiveEdgeCosThresholdAngle(const JPH_MeshShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsMeshShapeSettings(settings)->mActiveEdgeCosThresholdAngle;
}

void JPH_MeshShapeSettings_SetActiveEdgeCosThresholdAngle(JPH_MeshShapeSettings *settings, const float value)
{
    JPH_ASSERT(settings);
    AsMeshShapeSettings(settings)->mActiveEdgeCosThresholdAngle = value;
}

bool JPH_MeshShapeSettings_GetPerTriangleUserData(const JPH_MeshShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsMeshShapeSettings(settings)->mPerTriangleUserData;
}

void JPH_MeshShapeSettings_SetPerTriangleUserData(JPH_MeshShapeSettings *settings, const bool value)
{
    JPH_ASSERT(settings);
    AsMeshShapeSettings(settings)->mPerTriangleUserData = value;
}

JPH_Mesh_Shape_BuildQuality JPH_MeshShapeSettings_GetBuildQuality(const JPH_MeshShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return static_cast<JPH_Mesh_Shape_BuildQuality>(AsMeshShapeSettings(settings)->mBuildQuality);
}

void JPH_MeshShapeSettings_SetBuildQuality(JPH_MeshShapeSettings *settings, JPH_Mesh_Shape_BuildQuality value)
{
    JPH_ASSERT(settings);
    AsMeshShapeSettings(settings)->mBuildQuality = static_cast<JPH::MeshShapeSettings::EBuildQuality>(value);
}

void JPH_MeshShapeSettings_Sanitize(JPH_MeshShapeSettings *settings)
{
    JPH_ASSERT(settings);
    AsMeshShapeSettings(settings)->Sanitize();
}

JPH_MeshShape *JPH_MeshShapeSettings_CreateShape(const JPH_MeshShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::MeshShapeSettings::ShapeResult shapeResult = AsMeshShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_MeshShape *>(shape);
}

uint32_t JPH_MeshShape_GetTriangleUserData(const JPH_MeshShape *shape, const JPH_SubShapeID id)
{
    JPH_ASSERT(shape);
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(id);
    return AsMeshShape(shape)->GetTriangleUserData(joltSubShapeID);
}

/* HeightFieldShapeSettings */
JPH_HeightFieldShapeSettings *JPH_HeightFieldShapeSettings_Create(const float *samples,
                                                                  const Vector3 *offset,
                                                                  const Vector3 *scale,
                                                                  const uint32_t sampleCount,
                                                                  const uint8_t *materialIndices)
{
    JPH::HeightFieldShapeSettings *const settings = new JPH::HeightFieldShapeSettings(samples,
                                                                                      ToJolt(offset),
                                                                                      ToJolt(scale),
                                                                                      sampleCount,
                                                                                      materialIndices);
    settings->AddRef();

    return ToHeightFieldShapeSettings(settings);
}

JPH_HeightFieldShape *JPH_HeightFieldShapeSettings_CreateShape(JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::HeightFieldShapeSettings::ShapeResult shapeResult = AsHeightFieldShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_HeightFieldShape *>(shape);
}

void JPH_HeightFieldShapeSettings_DetermineMinAndMaxSample(const JPH_HeightFieldShapeSettings *settings,
                                                           float *pOutMinValue,
                                                           float *pOutMaxValue,
                                                           float *pOutQuantizationScale)
{
    JPH_ASSERT(settings);
    float outMinValue = 0;
    float outMaxValue = 0;
    float outQuantizationScale = 0;
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
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->CalculateBitsPerSampleForError(maxError);
}

void JPH_HeightFieldShapeSettings_GetOffset(const JPH_HeightFieldShapeSettings *settings, Vector3 *result)
{
    JPH_ASSERT(settings);
    FromJolt(AsHeightFieldShapeSettings(settings)->mOffset, result);
}

void JPH_HeightFieldShapeSettings_SetOffset(JPH_HeightFieldShapeSettings *settings, const Vector3 *value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mOffset = ToJolt(value);
}

void JPH_HeightFieldShapeSettings_GetScale(const JPH_HeightFieldShapeSettings *settings, Vector3 *result)
{
    JPH_ASSERT(settings);
    FromJolt(AsHeightFieldShapeSettings(settings)->mScale, result);
}

void JPH_HeightFieldShapeSettings_SetScale(JPH_HeightFieldShapeSettings *settings, const Vector3 *value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mScale = ToJolt(value);
}

uint32_t JPH_HeightFieldShapeSettings_GetSampleCount(const JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->mSampleCount;
}

void JPH_HeightFieldShapeSettings_SetSampleCount(JPH_HeightFieldShapeSettings *settings, uint32_t value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mSampleCount = value;
}

float JPH_HeightFieldShapeSettings_GetMinHeightValue(const JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->mMinHeightValue;
}

void JPH_HeightFieldShapeSettings_SetMinHeightValue(JPH_HeightFieldShapeSettings *settings, float value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mMinHeightValue = value;
}

float JPH_HeightFieldShapeSettings_GetMaxHeightValue(const JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->mMaxHeightValue;
}

void JPH_HeightFieldShapeSettings_SetMaxHeightValue(JPH_HeightFieldShapeSettings *settings, float value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mMaxHeightValue = value;
}

uint32_t JPH_HeightFieldShapeSettings_GetBlockSize(const JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->mBlockSize;
}

void JPH_HeightFieldShapeSettings_SetBlockSize(JPH_HeightFieldShapeSettings *settings, uint32_t value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mBlockSize = value;
}

uint32_t JPH_HeightFieldShapeSettings_GetBitsPerSample(const JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->mBitsPerSample;
}

void JPH_HeightFieldShapeSettings_SetBitsPerSample(JPH_HeightFieldShapeSettings *settings, uint32_t value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mBitsPerSample = value;
}

float JPH_HeightFieldShapeSettings_GetActiveEdgeCosThresholdAngle(const JPH_HeightFieldShapeSettings *settings)
{
    JPH_ASSERT(settings);
    return AsHeightFieldShapeSettings(settings)->mActiveEdgeCosThresholdAngle;
}

void JPH_HeightFieldShapeSettings_SetActiveEdgeCosThresholdAngle(JPH_HeightFieldShapeSettings *settings, float value)
{
    JPH_ASSERT(settings);
    AsHeightFieldShapeSettings(settings)->mActiveEdgeCosThresholdAngle = value;
}

uint32_t JPH_HeightFieldShape_GetSampleCount(const JPH_HeightFieldShape *shape)
{
    JPH_ASSERT(shape);
    return AsHeightFieldShape(shape)->GetSampleCount();
}

uint32_t JPH_HeightFieldShape_GetBlockSize(const JPH_HeightFieldShape *shape)
{
    JPH_ASSERT(shape);
    return AsHeightFieldShape(shape)->GetBlockSize();
}

const JPH_PhysicsMaterial *JPH_HeightFieldShape_GetMaterial(const JPH_HeightFieldShape *shape,
                                                            const uint32_t x,
                                                            const uint32_t y)
{
    JPH_ASSERT(shape);
    return ToPhysicsMaterial(AsHeightFieldShape(shape)->GetMaterial(x, y));
}

void JPH_HeightFieldShape_GetPosition(const JPH_HeightFieldShape *shape,
                                      const uint32_t x,
                                      const uint32_t y,
                                      Vector3 *result)
{
    JPH_ASSERT(shape);
    FromJolt(AsHeightFieldShape(shape)->GetPosition(x, y), result);
}

bool JPH_HeightFieldShape_IsNoCollision(const JPH_HeightFieldShape *shape, const uint32_t x, const uint32_t y)
{
    JPH_ASSERT(shape);
    return AsHeightFieldShape(shape)->IsNoCollision(x, y);
}

bool JPH_HeightFieldShape_ProjectOntoSurface(const JPH_HeightFieldShape *shape,
                                             const Vector3 *localPosition,
                                             Vector3 *outSurfacePosition,
                                             JPH_SubShapeID *outSubShapeID)
{
    JPH_ASSERT(outSurfacePosition);
    JPH_ASSERT(outSubShapeID);

    JPH::Vec3 surfacePosition{};
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
    JPH_ASSERT(shape);
    return AsHeightFieldShape(shape)->GetMinHeightValue();
}

float JPH_HeightFieldShape_GetMaxHeightValue(const JPH_HeightFieldShape *shape)
{
    JPH_ASSERT(shape);
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

    return ToTaperedCapsuleShapeSettings(settings);
}

JPH_TaperedCapsuleShape *JPH_TaperedCapsuleShapeSettings_CreateShape(const JPH_TaperedCapsuleShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::TaperedCapsuleShapeSettings::ShapeResult shapeResult = AsTaperedCapsuleShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_TaperedCapsuleShape *>(shape);
}

float JPH_TaperedCapsuleShape_GetTopRadius(const JPH_TaperedCapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCapsuleShape(shape)->GetTopRadius();
}

float JPH_TaperedCapsuleShape_GetBottomRadius(const JPH_TaperedCapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCapsuleShape(shape)->GetBottomRadius();
}

float JPH_TaperedCapsuleShape_GetHalfHeight(const JPH_TaperedCapsuleShape *shape)
{
    JPH_ASSERT(shape);
    return AsTaperedCapsuleShape(shape)->GetHalfHeight();
}

/* CompoundShape */
void JPH_CompoundShapeSettings_AddShape(JPH_CompoundShapeSettings *settings,
                                        const Vector3 *position,
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
                                         const Vector3 *position,
                                         const JPH_Quat *rotation,
                                         const JPH_Shape *shape,
                                         const uint32_t userData)
{
    AsCompoundShapeSettings(settings)->AddShape(ToJolt(position), ToJolt(rotation), AsShape(shape), userData);
}

uint32_t JPH_CompoundShape_GetNumSubShapes(const JPH_CompoundShape *shape)
{
    JPH_ASSERT(shape);
    return AsCompoundShape(shape)->GetNumSubShapes();
}

void JPH_CompoundShape_GetSubShape(const JPH_CompoundShape *shape,
                                   uint32_t index,
                                   const JPH_Shape **subShape,
                                   Vector3 *positionCOM,
                                   JPH_Quat *rotation,
                                   uint32_t *userData)
{
    JPH_ASSERT(shape);
    const JPH::CompoundShape::SubShape &sub = AsCompoundShape(shape)->GetSubShape(index);
    if (subShape != nullptr)
    {
        *subShape = ToShape(sub.mShape.GetPtr());
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
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(id);
    JPH::SubShapeID joltRemainder = JPH::SubShapeID();
    const uint32_t index = AsCompoundShape(shape)->GetSubShapeIndexFromID(joltSubShapeID, joltRemainder);
    *remainder = joltRemainder.GetValue();
    return index;
}

/* StaticCompoundShape */
JPH_StaticCompoundShapeSettings *JPH_StaticCompoundShapeSettings_Create()
{
    JPH::StaticCompoundShapeSettings *const settings = new JPH::StaticCompoundShapeSettings();
    settings->AddRef();

    return ToStaticCompoundShapeSettings(settings);
}

JPH_StaticCompoundShape *JPH_StaticCompoundShape_Create(const JPH_StaticCompoundShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::StaticCompoundShapeSettings::ShapeResult shapeResult = AsStaticCompoundShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_StaticCompoundShape *>(shape);
}

/* MutableCompoundShape */
JPH_MutableCompoundShapeSettings *JPH_MutableCompoundShapeSettings_Create()
{
    JPH::MutableCompoundShapeSettings *const settings = new JPH::MutableCompoundShapeSettings();
    settings->AddRef();

    return ToMutableCompoundShapeSettings(settings);
}

JPH_MutableCompoundShape *JPH_MutableCompoundShape_Create(const JPH_MutableCompoundShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::MutableCompoundShapeSettings::ShapeResult shapeResult =
            AsMutableCompoundShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_MutableCompoundShape *>(shape);
}

uint32_t JPH_MutableCompoundShape_AddShape(JPH_MutableCompoundShape *shape,
                                           const Vector3 *position,
                                           const JPH_Quat *rotation,
                                           const JPH_Shape *child,
                                           const uint32_t userData,
                                           const uint32_t index)
{
    JPH_ASSERT(shape);
    return AsMutableCompoundShape(shape)->AddShape(ToJolt(position), ToJolt(rotation), AsShape(child), userData, index);
}

void JPH_MutableCompoundShape_RemoveShape(JPH_MutableCompoundShape *shape, const uint32_t index)
{
    JPH_ASSERT(shape);
    AsMutableCompoundShape(shape)->RemoveShape(index);
}

void JPH_MutableCompoundShape_ModifyShape(JPH_MutableCompoundShape *shape,
                                          const uint32_t index,
                                          const Vector3 *position,
                                          const JPH_Quat *rotation)
{
    JPH_ASSERT(shape);
    AsMutableCompoundShape(shape)->ModifyShape(index, ToJolt(position), ToJolt(rotation));
}

void JPH_MutableCompoundShape_ModifyShape2(JPH_MutableCompoundShape *shape,
                                           const uint32_t index,
                                           const Vector3 *position,
                                           const JPH_Quat *rotation,
                                           const JPH_Shape *newShape)
{
    JPH_ASSERT(shape);
    AsMutableCompoundShape(shape)->ModifyShape(index, ToJolt(position), ToJolt(rotation), AsShape(newShape));
}

void JPH_MutableCompoundShape_AdjustCenterOfMass(JPH_MutableCompoundShape *shape)
{
    reinterpret_cast<JPH::MutableCompoundShape *>(shape)->AdjustCenterOfMass();
}

/* DecoratedShape */
const JPH_Shape *JPH_DecoratedShape_GetInnerShape(const JPH_DecoratedShape *shape)
{
    JPH_ASSERT(shape);
    return ToShape(AsDecoratedShape(shape)->GetInnerShape());
}

/* RotatedTranslatedShape */
JPH_RotatedTranslatedShapeSettings *JPH_RotatedTranslatedShapeSettings_Create(const Vector3 *position,
                                                                              const JPH_Quat *rotation,
                                                                              const JPH_ShapeSettings *shapeSettings)
{
    JPH::RotatedTranslatedShapeSettings *const settings =
            new JPH::RotatedTranslatedShapeSettings(ToJolt(position),
                                                    rotation != nullptr ? ToJolt(rotation) : JPH::Quat::sIdentity(),
                                                    AsShapeSettings(shapeSettings));
    settings->AddRef();

    return ToRotatedTranslatedShapeSettings(settings);
}

JPH_RotatedTranslatedShapeSettings *JPH_RotatedTranslatedShapeSettings_Create2(const Vector3 *position,
                                                                               const JPH_Quat *rotation,
                                                                               const JPH_Shape *shape)
{
    JPH::RotatedTranslatedShapeSettings *const settings =
            new JPH::RotatedTranslatedShapeSettings(ToJolt(position),
                                                    rotation != nullptr ? ToJolt(rotation) : JPH::Quat::sIdentity(),
                                                    AsShape(shape));
    settings->AddRef();

    return ToRotatedTranslatedShapeSettings(settings);
}

JPH_RotatedTranslatedShape *JPH_RotatedTranslatedShapeSettings_CreateShape(const JPH_RotatedTranslatedShapeSettings
                                                                                   *settings)
{
    JPH_ASSERT(settings);
    const JPH::RotatedTranslatedShapeSettings::ShapeResult shapeResult =
            AsRotatedTranslatedShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_RotatedTranslatedShape *>(shape);
}

JPH_RotatedTranslatedShape *JPH_RotatedTranslatedShape_Create(const Vector3 *position,
                                                              const JPH_Quat *rotation,
                                                              const JPH_Shape *shape)
{
    JPH::RotatedTranslatedShape *const rotatedTranslatedShape =
            new JPH::RotatedTranslatedShape(ToJolt(position),
                                            rotation != nullptr ? ToJolt(rotation) : JPH::Quat::sIdentity(),
                                            AsShape(shape));
    rotatedTranslatedShape->AddRef();

    return ToRotatedTranslatedShape(rotatedTranslatedShape);
}

void JPH_RotatedTranslatedShape_GetPosition(const JPH_RotatedTranslatedShape *shape, Vector3 *position)
{
    JPH_ASSERT(shape);
    const JPH::Vec3 joltVector = AsRotatedTranslatedShape(shape)->GetPosition();
    FromJolt(joltVector, position);
}

void JPH_RotatedTranslatedShape_GetRotation(const JPH_RotatedTranslatedShape *shape, JPH_Quat *rotation)
{
    JPH_ASSERT(shape);
    const JPH::Quat joltQuat = AsRotatedTranslatedShape(shape)->GetRotation();
    FromJolt(joltQuat, rotation);
}

JPH_ScaledShapeSettings *JPH_ScaledShapeSettings_Create(const JPH_ShapeSettings *shapeSettings, const Vector3 *scale)
{
    JPH::ScaledShapeSettings *const settings = new JPH::ScaledShapeSettings(AsShapeSettings(shapeSettings),
                                                                            ToJolt(scale));
    settings->AddRef();

    return ToScaledShapeSettings(settings);
}

JPH_ScaledShapeSettings *JPH_ScaledShapeSettings_Create2(const JPH_Shape *shape, const Vector3 *scale)
{
    JPH::ScaledShapeSettings *const settings = new JPH::ScaledShapeSettings(AsShape(shape), ToJolt(scale));
    settings->AddRef();

    return ToScaledShapeSettings(settings);
}

JPH_ScaledShape *JPH_ScaledShapeSettings_CreateShape(const JPH_ScaledShapeSettings *settings)
{
    JPH_ASSERT(settings);
    const JPH::SphereShapeSettings::ShapeResult shapeResult = AsScaledShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_ScaledShape *>(shape);
}

JPH_ScaledShape *JPH_ScaledShape_Create(const JPH_Shape *shape, const Vector3 *scale)
{
    JPH::ScaledShape *const scaledShape = new JPH::ScaledShape(AsShape(shape), ToJolt(scale));
    scaledShape->AddRef();

    return ToScaledShape(scaledShape);
}

void JPH_ScaledShape_GetScale(const JPH_ScaledShape *shape, Vector3 *result)
{
    JPH_ASSERT(shape);
    const JPH::Vec3 joltScale = AsScaledShape(shape)->GetScale();
    FromJolt(joltScale, result);
}

/* JPH_OffsetCenterOfMassShape */
JPH_OffsetCenterOfMassShapeSettings *JPH_OffsetCenterOfMassShapeSettings_Create(const Vector3 *offset,
                                                                                const JPH_ShapeSettings *shapeSettings)
{
    JPH::OffsetCenterOfMassShapeSettings *const settings =
            new JPH::OffsetCenterOfMassShapeSettings(ToJolt(offset), AsShapeSettings(shapeSettings));
    settings->AddRef();

    return ToOffsetCenterOfMassShapeSettings(settings);
}

JPH_OffsetCenterOfMassShapeSettings *JPH_OffsetCenterOfMassShapeSettings_Create2(const Vector3 *offset,
                                                                                 const JPH_Shape *shape)
{
    JPH::OffsetCenterOfMassShapeSettings *const rotatedTranslatedShape =
            new JPH::OffsetCenterOfMassShapeSettings(ToJolt(offset), AsShape(shape));
    rotatedTranslatedShape->AddRef();

    return ToOffsetCenterOfMassShapeSettings(rotatedTranslatedShape);
}

JPH_OffsetCenterOfMassShape *JPH_OffsetCenterOfMassShapeSettings_CreateShape(const JPH_OffsetCenterOfMassShapeSettings
                                                                                     *settings)
{
    JPH_ASSERT(settings);
    const JPH::OffsetCenterOfMassShapeSettings::ShapeResult shapeResult =
            AsOffsetCenterOfMassShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_OffsetCenterOfMassShape *>(shape);
}

JPH_OffsetCenterOfMassShape *JPH_OffsetCenterOfMassShape_Create(const Vector3 *offset, const JPH_Shape *shape)
{
    JPH::OffsetCenterOfMassShape *const offsetCenterOfMassShape = new JPH::OffsetCenterOfMassShape(AsShape(shape),
                                                                                                   ToJolt(offset));
    offsetCenterOfMassShape->AddRef();

    return reinterpret_cast<JPH_OffsetCenterOfMassShape *>(offsetCenterOfMassShape);
}

void JPH_OffsetCenterOfMassShape_GetOffset(const JPH_OffsetCenterOfMassShape *shape, Vector3 *result)
{
    FromJolt(AsOffsetCenterOfMassShape(shape)->GetOffset(), result);
}

/* EmptyShape */
JPH_EmptyShapeSettings *JPH_EmptyShapeSettings_Create(const Vector3 *centerOfMass)
{
    JPH::EmptyShapeSettings *const settings = new JPH::EmptyShapeSettings(ToJolt(centerOfMass));
    settings->AddRef();

    return ToEmptyShapeSettings(settings);
}

JPH_EmptyShape *JPH_EmptyShapeSettings_CreateShape(const JPH_EmptyShapeSettings *settings)
{
    const JPH::EmptyShapeSettings::ShapeResult shapeResult = AsEmptyShapeSettings(settings)->Create();
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *shape = shapeResult.Get().GetPtr();
    shape->AddRef();

    return reinterpret_cast<JPH_EmptyShape *>(shape);
}

/* JPH_ConstraintSettings */
static inline void JPH_ConstraintSettings_Init(const JPH::ConstraintSettings &joltSettings,
                                               JPH_ConstraintSettings *settings)
{
    // Copy defaults from jolt
    settings->enabled = joltSettings.mEnabled;
    settings->constraintPriority = joltSettings.mConstraintPriority;
    settings->numVelocityStepsOverride = joltSettings.mNumVelocityStepsOverride;
    settings->numPositionStepsOverride = joltSettings.mNumPositionStepsOverride;
    settings->drawConstraintSize = joltSettings.mDrawConstraintSize;
    settings->userData = joltSettings.mUserData;
}

static inline void JPH_ConstraintSettings_ToJolt(JPH::ConstraintSettings *joltSettings,
                                                 const JPH_ConstraintSettings *settings)
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

void JPH_Constraint_NotifyShapeChanged(JPH_Constraint *constraint, const JPH_BodyID bodyID, const Vector3 *deltaCOM)
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
    return reinterpret_cast<JPH_Body *>(AsTwoBodyConstraint(constraint)->GetBody1());
}

JPH_Body *JPH_TwoBodyConstraint_GetBody2(const JPH_TwoBodyConstraint *constraint)
{
    return reinterpret_cast<JPH_Body *>(AsTwoBodyConstraint(constraint)->GetBody2());
}

void JPH_TwoBodyConstraint_GetConstraintToBody1Matrix(const JPH_TwoBodyConstraint *constraint, JPH_Mat44 *result)
{
    const JPH::Mat44 joltMatrix = AsTwoBodyConstraint(constraint)->GetConstraintToBody1Matrix();
    FromJolt(joltMatrix, result);
}

void JPH_TwoBodyConstraint_GetConstraintToBody2Matrix(const JPH_TwoBodyConstraint *constraint, JPH_Mat44 *result)
{
    const JPH::Mat44 joltMatrix = AsTwoBodyConstraint(constraint)->GetConstraintToBody2Matrix();
    FromJolt(joltMatrix, result);
}

/* JPH_FixedConstraintSettings */
static inline void JPH_FixedConstraintSettings_FromJolt(JPH_FixedConstraintSettings *settings,
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

static inline void JPH_FixedConstraintSettings_ToJolt(JPH::FixedConstraintSettings *joltSettings,
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

    JPH::FixedConstraint *constraint = JPH::StaticCast<JPH::FixedConstraint>(joltSettings.Create(*joltBody1,
                                                                                                 *joltBody2));
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

    const JPH::Ref<JPH::FixedConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::FixedConstraintSettings>(AsFixedConstraint(constraint)->GetConstraintSettings());
    JPH_FixedConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_FixedConstraint_GetTotalLambdaPosition(const JPH_FixedConstraint *constraint, Vector3 *result)
{
    const JPH::Vec3 lambda = AsFixedConstraint(constraint)->GetTotalLambdaPosition();
    FromJolt(lambda, result);
}

void JPH_FixedConstraint_GetTotalLambdaRotation(const JPH_FixedConstraint *constraint, Vector3 *result)
{
    const JPH::Vec3 lambda = AsFixedConstraint(constraint)->GetTotalLambdaRotation();
    FromJolt(lambda, result);
}

/* JPH_DistanceConstraintSettings */
static inline void JPH_DistanceConstraintSettings_FromJolt(JPH_DistanceConstraintSettings *settings,
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

static inline void JPH_DistanceConstraintSettings_ToJolt(JPH::DistanceConstraintSettings *joltSettings,
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

    JPH::DistanceConstraint *constraint = JPH::StaticCast<JPH::DistanceConstraint>(joltSettings.Create(*joltBody1,
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

    const JPH::Ref<JPH::DistanceConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::DistanceConstraintSettings>(AsDistanceConstraint(constraint)->GetConstraintSettings());
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
static inline void JPH_PointConstraintSettings_FromJolt(JPH_PointConstraintSettings *settings,
                                                        const JPH::PointConstraintSettings &joltSettings)
{
    JPH_ASSERT(settings);

    // Base
    JPH_ConstraintSettings_Init(joltSettings, &settings->base);

    settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
    FromJolt(joltSettings.mPoint1, &settings->point1);
    FromJolt(joltSettings.mPoint2, &settings->point2);
}

static inline void JPH_PointConstraintSettings_ToJolt(JPH::PointConstraintSettings *joltSettings,
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

    JPH::PointConstraint *constraint = JPH::StaticCast<JPH::PointConstraint>(joltSettings.Create(*joltBody1,
                                                                                                 *joltBody2));
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

    const JPH::Ref<JPH::PointConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::PointConstraintSettings>(AsPointConstraint(constraint)->GetConstraintSettings());
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

void JPH_PointConstraint_GetLocalSpacePoint1(const JPH_PointConstraint *constraint, Vector3 *result)
{
    FromJolt(AsPointConstraint(constraint)->GetLocalSpacePoint1(), result);
}

void JPH_PointConstraint_GetLocalSpacePoint2(const JPH_PointConstraint *constraint, Vector3 *result)
{
    FromJolt(AsPointConstraint(constraint)->GetLocalSpacePoint2(), result);
}

void JPH_PointConstraint_GetTotalLambdaPosition(const JPH_PointConstraint *constraint, Vector3 *result)
{
    FromJolt(AsPointConstraint(constraint)->GetTotalLambdaPosition(), result);
}

/* JPH_HingeConstraintSettings */
static inline void JPH_HingeConstraintSettings_FromJolt(JPH_HingeConstraintSettings *settings,
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

static inline void JPH_HingeConstraintSettings_ToJolt(JPH::HingeConstraintSettings *joltSettings,
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

    JPH::HingeConstraint *constraint = JPH::StaticCast<JPH::HingeConstraint>(joltSettings.Create(*joltBody1,
                                                                                                 *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToHingeConstraint(constraint);
}

void JPH_HingeConstraint_GetSettings(JPH_HingeConstraint *constraint, JPH_HingeConstraintSettings *settings)
{
    const JPH::Ref<JPH::HingeConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::HingeConstraintSettings>(AsHingeConstraint(constraint)->GetConstraintSettings());
    JPH_HingeConstraintSettings_FromJolt(settings, *joltSettings);
}

void JPH_HingeConstraint_GetLocalSpacePoint1(const JPH_HingeConstraint *constraint, Vector3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpacePoint1(), result);
}

void JPH_HingeConstraint_GetLocalSpacePoint2(const JPH_HingeConstraint *constraint, Vector3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpacePoint2(), result);
}

void JPH_HingeConstraint_GetLocalSpaceHingeAxis1(const JPH_HingeConstraint *constraint, Vector3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceHingeAxis1(), result);
}

void JPH_HingeConstraint_GetLocalSpaceHingeAxis2(const JPH_HingeConstraint *constraint, Vector3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceHingeAxis2(), result);
}

void JPH_HingeConstraint_GetLocalSpaceNormalAxis1(const JPH_HingeConstraint *constraint, Vector3 *result)
{
    FromJolt(AsHingeConstraint(constraint)->GetLocalSpaceNormalAxis1(), result);
}

void JPH_HingeConstraint_GetLocalSpaceNormalAxis2(const JPH_HingeConstraint *constraint, Vector3 *result)
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

void JPH_HingeConstraint_GetTotalLambdaPosition(const JPH_HingeConstraint *constraint, Vector3 *result)
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
static inline void JPH_SliderConstraintSettings_FromJolt(JPH_SliderConstraintSettings *settings,
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

static inline void JPH_SliderConstraintSettings_ToJolt(JPH::SliderConstraintSettings *joltSettings,
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

void JPH_SliderConstraintSettings_SetSliderAxis(JPH_SliderConstraintSettings *settings, const Vector3 *axis)
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

    JPH::SliderConstraint *constraint = JPH::StaticCast<JPH::SliderConstraint>(joltSettings.Create(*joltBody1,
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
    const JPH::Ref<JPH::SliderConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::SliderConstraintSettings>(AsSliderConstraint(constraint)->GetConstraintSettings());
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

void JPH_SliderConstraint_GetTotalLambdaRotation(const JPH_SliderConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSliderConstraint(constraint)->GetTotalLambdaRotation(), result);
}

float JPH_SliderConstraint_GetTotalLambdaMotor(const JPH_SliderConstraint *constraint)
{
    return AsSliderConstraint(constraint)->GetTotalLambdaMotor();
}

/* JPH_ConeConstraintSettings */
static inline void JPH_ConeConstraintSettings_FromJolt(JPH_ConeConstraintSettings *settings,
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

static inline void JPH_ConeConstraintSettings_ToJolt(JPH::ConeConstraintSettings *joltSettings,
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

    JPH::ConeConstraint *constraint = JPH::StaticCast<JPH::ConeConstraint>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToConeConstraint(constraint);
}

void JPH_ConeConstraint_GetSettings(JPH_ConeConstraint *constraint, JPH_ConeConstraintSettings *settings)
{
    const JPH::Ref<JPH::ConeConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::ConeConstraintSettings>(AsConeConstraint(constraint)->GetConstraintSettings());
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

void JPH_ConeConstraint_GetTotalLambdaPosition(const JPH_ConeConstraint *constraint, Vector3 *result)
{
    FromJolt(AsConeConstraint(constraint)->GetTotalLambdaPosition(), result);
}

float JPH_ConeConstraint_GetTotalLambdaRotation(const JPH_ConeConstraint *constraint)
{
    return AsConeConstraint(constraint)->GetTotalLambdaRotation();
}

/* JPH_SwingTwistConstraintSettings */
static inline void JPH_SwingTwistConstraintSettings_FromJolt(JPH_SwingTwistConstraintSettings *settings,
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

static inline void JPH_SwingTwistConstraintSettings_ToJolt(JPH::SwingTwistConstraintSettings *joltSettings,
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

    JPH::SwingTwistConstraint *constraint = JPH::StaticCast<JPH::SwingTwistConstraint>(joltSettings.Create(*joltBody1,
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

void JPH_SwingTwistConstraint_GetTotalLambdaPosition(const JPH_SwingTwistConstraint *constraint, Vector3 *result)
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

void JPH_SwingTwistConstraint_GetTotalLambdaMotor(const JPH_SwingTwistConstraint *constraint, Vector3 *result)
{
    JPH_ASSERT(constraint);
    const JPH::SwingTwistConstraint *joltConstraint = reinterpret_cast<const JPH::SwingTwistConstraint *>(constraint);
    const JPH::Vec3 lambda = joltConstraint->GetTotalLambdaMotor();
    FromJolt(lambda, result);
}

/* JPH_SixDOFConstraintSettings */
static inline void JPH_SixDOFConstraintSettings_FromJolt(JPH_SixDOFConstraintSettings *settings,
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

static inline void JPH_SixDOFConstraintSettings_ToJolt(JPH::SixDOFConstraintSettings *joltSettings,
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

    JPH::SixDOFConstraint *constraint = JPH::StaticCast<JPH::SixDOFConstraint>(joltSettings.Create(*joltBody1,
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
    const JPH::Ref<JPH::SixDOFConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::SixDOFConstraintSettings>(AsSixDOFConstraint(constraint)->GetConstraintSettings());
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

void JPH_SixDOFConstraint_GetTotalLambdaPosition(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaPosition(), result);
}

void JPH_SixDOFConstraint_GetTotalLambdaRotation(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaRotation(), result);
}

void JPH_SixDOFConstraint_GetTotalLambdaMotorTranslation(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaMotorTranslation(), result);
}

void JPH_SixDOFConstraint_GetTotalLambdaMotorRotation(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTotalLambdaMotorRotation(), result);
}

void JPH_SixDOFConstraint_GetTranslationLimitsMin(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTranslationLimitsMin(), result);
}

void JPH_SixDOFConstraint_GetTranslationLimitsMax(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTranslationLimitsMax(), result);
}

void JPH_SixDOFConstraint_GetRotationLimitsMin(const JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetRotationLimitsMin(), result);
}

void JPH_SixDOFConstraint_GetRotationLimitsMax(const JPH_SixDOFConstraint *constraint, Vector3 *result)
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
    return static_cast<JPH_MotorState>(
            AsSixDOFConstraint(constraint)->GetMotorState(static_cast<JPH::SixDOFConstraint::EAxis>(axis)));
}

void JPH_SixDOFConstraint_SetTargetVelocityCS(JPH_SixDOFConstraint *constraint, const Vector3 *inVelocity)
{
    AsSixDOFConstraint(constraint)->SetTargetVelocityCS(ToJolt(inVelocity));
}

void JPH_SixDOFConstraint_GetTargetVelocityCS(JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTargetVelocityCS(), result);
}

void JPH_SixDOFConstraint_SetTargetAngularVelocityCS(JPH_SixDOFConstraint *constraint, const Vector3 *inAngularVelocity)
{
    AsSixDOFConstraint(constraint)->SetTargetAngularVelocityCS(ToJolt(inAngularVelocity));
}

void JPH_SixDOFConstraint_GetTargetAngularVelocityCS(JPH_SixDOFConstraint *constraint, Vector3 *result)
{
    FromJolt(AsSixDOFConstraint(constraint)->GetTargetAngularVelocityCS(), result);
}

void JPH_SixDOFConstraint_SetTargetPositionCS(JPH_SixDOFConstraint *constraint, const Vector3 *inPosition)
{
    AsSixDOFConstraint(constraint)->SetTargetPositionCS(ToJolt(inPosition));
}

void JPH_SixDOFConstraint_GetTargetPositionCS(JPH_SixDOFConstraint *constraint, Vector3 *result)
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
static inline void JPH_GearConstraintSettings_FromJolt(JPH_GearConstraintSettings *settings,
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

static inline void JPH_GearConstraintSettings_ToJolt(JPH::GearConstraintSettings *joltSettings,
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

    JPH::GearConstraint *constraint = JPH::StaticCast<JPH::GearConstraint>(joltSettings.Create(*joltBody1, *joltBody2));
    if (constraint == nullptr)
    {
        return nullptr;
    }

    constraint->AddRef();
    return ToGearConstraint(constraint);
}

void JPH_GearConstraint_GetSettings(JPH_GearConstraint *constraint, JPH_GearConstraintSettings *settings)
{
    const JPH::Ref<JPH::GearConstraintSettings> joltSettings =
            JPH::StaticCast<JPH::GearConstraintSettings>(AsGearConstraint(constraint)->GetConstraintSettings());
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

void JPH_PhysicsSystem_SetSimShapeFilter(JPH_PhysicsSystem *system, const JPH_SimShapeFilter *filter)
{
    JPH_ASSERT(system);

    const JPH::SimShapeFilter *joltFilter = reinterpret_cast<const JPH::SimShapeFilter *>(filter);
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

void JPH_PhysicsSystem_SetGravity(JPH_PhysicsSystem *system, const Vector3 *value)
{
    JPH_ASSERT(system);

    system->physicsSystem->SetGravity(ToJolt(value));
}

void JPH_PhysicsSystem_GetGravity(JPH_PhysicsSystem *system, Vector3 *result)
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
    for (int i = 0; i < count; ++i)
    {
        // NOLINTNEXTLINE(*-const-correctness) clang-tidy is just straight up wrong
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
    for (int i = 0; i < count; ++i)
    {
        // NOLINTNEXTLINE(*-const-correctness) clang-tidy is just straight up wrong
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
        ids[i] = bodies.at(i).GetIndexAndSequenceNumber();
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
        constraints[i] = ToConstraint(list.at(i).GetPtr());
    }
}

void JPH_PhysicsSystem_ActivateBodiesInAABox(const JPH_PhysicsSystem *system,
                                             const JPH_AABox *box,
                                             const JPH_ObjectLayer layer)
{
    system->physicsSystem->GetBodyInterface().ActivateBodiesInAABox(
            ToJolt(box),
            system->physicsSystem->GetDefaultBroadPhaseLayerFilter(layer),
            system->physicsSystem->GetDefaultLayerFilter(layer));
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
    result.mDrawSoftBodyConstraintColor =
            static_cast<JPH::ESoftBodyConstraintColor>(settings->drawSoftBodyConstraintColor);
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
        static const JPH_PhysicsStepListener_Impl *s_Impl;
        void *userData = nullptr;

        explicit ManagedPhysicsStepListener(void *userData_): userData(userData_) {}

        void OnStep(const JPH::PhysicsStepListenerContext &inContext) override
        {
            if (s_Impl != nullptr && s_Impl->OnStep != nullptr)
            {
                JPH_PhysicsStepListenerContext context{};
                context.deltaTime = inContext.mDeltaTime;
                context.isFirstStep = inContext.mIsFirstStep;
                context.isLastStep = inContext.mIsLastStep;
                context.physicsSystem = s_PhysicsSystems[inContext.mPhysicsSystem];
                s_Impl->OnStep(userData, &context);
            }
        }
};

const JPH_PhysicsStepListener_Impl *ManagedPhysicsStepListener::s_Impl = nullptr;

void JPH_PhysicsStepListener_SetImpl(const JPH_PhysicsStepListener_Impl *impl)
{
    ManagedPhysicsStepListener::s_Impl = impl;
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
    const JPH::BodyLockInterface *joltBodyLockInterface =
            reinterpret_cast<const JPH::BodyLockInterface *>(lockInterface);

    ::new (outLock) JPH::BodyLockWrite(*joltBodyLockInterface, JPH::BodyID(bodyID));
}

void JPH_BodyLockInterface_UnlockWrite(const JPH_BodyLockInterface *lockInterface, JPH_BodyLockWrite *ioLock)
{
    JPH_UNUSED(lockInterface);
    JPH_ASSERT(ioLock != nullptr);
    JPH_ASSERT(lockInterface != nullptr && lockInterface == ioLock->lockInterface); // NOLINT(*-simplify-boolean-expr)

    reinterpret_cast<const JPH::BodyLockWrite *>(ioLock)->~BodyLockWrite();
}

struct JPH_BodyLockMultiRead final
{
        JPH::Array<JPH::BodyID> bodyIDs;
        JPH::BodyLockMultiRead *joltLock = nullptr;
};

JPH_BodyLockMultiRead *JPH_BodyLockInterface_LockMultiRead(const JPH_BodyLockInterface *lockInterface,
                                                           const JPH_BodyID *bodyIDs,
                                                           const int count)
{
    const JPH::BodyLockInterface *const joltBodyLockInterface = AsBodyLockInterface(lockInterface);
    JPH_BodyLockMultiRead *read = new JPH_BodyLockMultiRead();
    read->bodyIDs.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        read->bodyIDs.push_back(JPH::BodyID(bodyIDs[i]));
    }

    read->joltLock = new JPH::BodyLockMultiRead(*joltBodyLockInterface, read->bodyIDs.data(), count);
    return read;
}

void JPH_BodyLockMultiRead_Destroy(JPH_BodyLockMultiRead *ioLock)
{
    if (ioLock != nullptr)
    {
        ioLock->bodyIDs.clear();
        delete ioLock->joltLock;
        delete ioLock;
    }
}

const JPH_Body *JPH_BodyLockMultiRead_GetBody(JPH_BodyLockMultiRead *ioLock, const int bodyIndex)
{
    JPH_ASSERT(ioLock && ioLock->joltLock);
    return ToBody(ioLock->joltLock->GetBody(bodyIndex));
}

struct JPH_BodyLockMultiWrite final
{
        JPH::Array<JPH::BodyID> bodyIDs;
        JPH::BodyLockMultiWrite *joltLock = nullptr;
};

JPH_BodyLockMultiWrite *JPH_BodyLockInterface_LockMultiWrite(const JPH_BodyLockInterface *lockInterface,
                                                             const JPH_BodyID *bodyIDs,
                                                             const int count)
{
    const JPH::BodyLockInterface *const joltBodyLockInterface = AsBodyLockInterface(lockInterface);
    JPH_BodyLockMultiWrite *write = new JPH_BodyLockMultiWrite();
    write->bodyIDs.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        write->bodyIDs.push_back(JPH::BodyID(bodyIDs[i]));
    }

    write->joltLock = new JPH::BodyLockMultiWrite(*joltBodyLockInterface, write->bodyIDs.data(), count);
    return write;
}

void JPH_BodyLockMultiWrite_Destroy(JPH_BodyLockMultiWrite *ioLock)
{
    if (ioLock != nullptr)
    {
        ioLock->bodyIDs.clear();
        delete ioLock->joltLock;
        delete ioLock;
    }
}

JPH_Body *JPH_BodyLockMultiWrite_GetBody(JPH_BodyLockMultiWrite *ioLock, const int bodyIndex)
{
    JPH_ASSERT(ioLock && ioLock->joltLock);
    return ToBody(ioLock->joltLock->GetBody(bodyIndex));
}

/* Contact Listener */
class ManagedContactListener final: public JPH::ContactListener
{
    public:
        const JPH_ContactListener_Impl *s_Impl = nullptr;

        JPH::ValidateResult OnContactValidate(const JPH::Body &inBody1,
                                              const JPH::Body &inBody2,
                                              JPH::RVec3Arg inBaseOffset,
                                              const JPH::CollideShapeResult &inCollisionResult) override
        {
            JPH_UNUSED(inCollisionResult);
            JPH_RVec3 baseOffset;
            FromJolt(inBaseOffset, &baseOffset);

            if (s_Impl != nullptr && s_Impl->OnContactValidate != nullptr)
            {
                const JPH_CollideShapeResult *collideShapeResult = nullptr;
                FromJolt(&inCollisionResult, collideShapeResult);

                const JPH_ValidateResult result =
                        s_Impl->OnContactValidate(reinterpret_cast<const JPH_Body *>(&inBody1),
                                                  reinterpret_cast<const JPH_Body *>(&inBody2),
                                                  &baseOffset,
                                                  collideShapeResult);

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

            if (s_Impl != nullptr && s_Impl->OnContactAdded != nullptr)
            {
                JPH_ContactSettings settings;
                FromJolt(ioSettings, &settings);
                s_Impl->OnContactAdded(reinterpret_cast<const JPH_Body *>(&inBody1),
                                       reinterpret_cast<const JPH_Body *>(&inBody2),
                                       ToContactManifold(&inManifold),
                                       &settings);
                ioSettings = ToJolt(&settings);
            }
        }

        void OnContactPersisted(const JPH::Body &inBody1,
                                const JPH::Body &inBody2,
                                const JPH::ContactManifold &inManifold,
                                JPH::ContactSettings &ioSettings) override
        {
            JPH_UNUSED(inManifold);
            JPH_UNUSED(ioSettings);

            if (s_Impl != nullptr && s_Impl->OnContactPersisted != nullptr)
            {
                JPH_ContactSettings settings;
                FromJolt(ioSettings, &settings);
                s_Impl->OnContactPersisted(reinterpret_cast<const JPH_Body *>(&inBody1),
                                           reinterpret_cast<const JPH_Body *>(&inBody2),
                                           ToContactManifold(&inManifold),
                                           &settings);
                ioSettings = ToJolt(&settings);
            }
        }

        void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
        {
            if (s_Impl != nullptr && s_Impl->OnContactRemoved != nullptr)
            {
                s_Impl->OnContactRemoved(reinterpret_cast<const JPH_SubShapeIDPair *>(&inSubShapePair));
            }
        }
};

void JPH_ContactListener_SetImpl(JPH_ContactListener *listener, const JPH_ContactListener_Impl *impl)
{
    reinterpret_cast<ManagedContactListener *>(listener)->s_Impl = impl;
}

JPH_ContactListener *JPH_ContactListener_Create()
{
    ManagedContactListener *const listener = new ManagedContactListener();
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
        static const JPH_BodyActivationListener_Impl *s_Impl;
        void *userData = nullptr;

        explicit ManagedBodyActivationListener(void *userData_): userData(userData_) {}

        void OnBodyActivated(const JPH::BodyID &inBodyID, const uint64_t inBodyUserData) override
        {
            if (s_Impl != nullptr && s_Impl->OnBodyActivated != nullptr)
            {
                s_Impl->OnBodyActivated(userData, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
            }
        }

        void OnBodyDeactivated(const JPH::BodyID &inBodyID, const uint64_t inBodyUserData) override
        {
            if (s_Impl != nullptr && s_Impl->OnBodyDeactivated != nullptr)
            {
                s_Impl->OnBodyDeactivated(userData, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
            }
        }
};

const JPH_BodyActivationListener_Impl *ManagedBodyActivationListener::s_Impl = nullptr;

void JPH_BodyActivationListener_SetImpl(const JPH_BodyActivationListener_Impl *impl)
{
    ManagedBodyActivationListener::s_Impl = impl;
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
void JPH_ContactManifold_GetWorldSpaceNormal(const JPH_ContactManifold *manifold, Vector3 *result)
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

/* CharacterBaseSettings */
static inline void JPH_CharacterBaseSettings_Init(const JPH::CharacterBaseSettings &joltSettings,
                                                  JPH_CharacterBaseSettings *settings)
{
    // Copy defaults from jolt
    if (Vector3_IsNearZero(&settings->up, 1.0e-12f))
    {
        FromJolt(joltSettings.mUp, &settings->up);
    }
    if (settings->supportingVolume.distance == 0 && Vector3_IsNearZero(&settings->supportingVolume.normal, 1.0e-12f))
    {
        FromJolt(joltSettings.mSupportingVolume, &settings->supportingVolume);
    }
    if (settings->maxSlopeAngle == 0)
    {
        settings->maxSlopeAngle = joltSettings.mMaxSlopeAngle;
    }
    if (settings->shape == nullptr)
    {
        if (joltSettings.mShape != nullptr)
        {
            settings->shape = ToShape(joltSettings.mShape.GetPtr());
        } else
        {
            constexpr Vector3 vec = {0.0f, 0.0f, 0.0f};
            const JPH_EmptyShapeSettings *empty_shape_settings = JPH_EmptyShapeSettings_Create(&vec);
            const JPH_EmptyShape *shape = JPH_EmptyShapeSettings_CreateShape(empty_shape_settings);
            settings->shape = reinterpret_cast<const JPH_Shape *>(shape);
        }
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

void JPH_CharacterBase_GetUp(JPH_CharacterBase *character, Vector3 *result)
{
    FromJolt(reinterpret_cast<JPH::CharacterBase *>(character)->GetUp(), result);
}

void JPH_CharacterBase_SetUp(JPH_CharacterBase *character, const Vector3 *value)
{
    JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    joltCharacter->SetUp(ToJolt(value));
}

bool JPH_CharacterBase_IsSlopeTooSteep(JPH_CharacterBase *character, const Vector3 *value)
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

void JPH_CharacterBase_GetGroundNormal(JPH_CharacterBase *character, Vector3 *normal)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    const JPH::Vec3 jolt_vector = joltCharacter->GetGroundNormal();
    FromJolt(jolt_vector, normal);
}

void JPH_CharacterBase_GetGroundVelocity(JPH_CharacterBase *character, Vector3 *velocity)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    const JPH::Vec3 jolt_vector = joltCharacter->GetGroundVelocity();
    FromJolt(jolt_vector, velocity);
}

const JPH_PhysicsMaterial *JPH_CharacterBase_GetGroundMaterial(JPH_CharacterBase *character)
{
    const JPH::CharacterBase *const joltCharacter = reinterpret_cast<JPH::CharacterBase *>(character);
    return ToPhysicsMaterial(joltCharacter->GetGroundMaterial());
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
                                               const Vector3 *linearVelocity,
                                               const Vector3 *angularVelocity,
                                               const bool lockBodies)
{
    AsCharacter(character)->SetLinearAndAngularVelocity(ToJolt(linearVelocity), ToJolt(angularVelocity), lockBodies);
}

void JPH_Character_GetLinearVelocity(JPH_Character *character, Vector3 *result)
{
    FromJolt(AsCharacter(character)->GetLinearVelocity(), result);
}

void JPH_Character_SetLinearVelocity(JPH_Character *character, const Vector3 *value, const bool lockBodies)
{
    AsCharacter(character)->SetLinearVelocity(ToJolt(value), lockBodies);
}

void JPH_Character_AddLinearVelocity(JPH_Character *character, const Vector3 *value, const bool lockBodies)
{
    AsCharacter(character)->AddLinearVelocity(ToJolt(value), lockBodies);
}

void JPH_Character_AddImpulse(JPH_Character *character, const Vector3 *value, const bool lockBodies)
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
    JPH::Quat joltRotation{};
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

bool JPH_Character_SetShape(JPH_Character *character,
                            const JPH_Shape *shape,
                            const float maxPenetrationDepth,
                            const bool lockBodies)
{
    return AsCharacter(character)->SetShape(AsShape(shape), maxPenetrationDepth, lockBodies);
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

void JPH_Character_GetWorldTransform(JPH_Character *character, JPH_RMat44 *result, const bool lockBodies)
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

    settings->ID = settings->ID == 0 ? joltSettings.mID.GetValue() : settings->ID;
    settings->mass = settings->mass == 0 ? joltSettings.mMass : settings->mass;
    settings->maxStrength = settings->maxStrength == 0 ? joltSettings.mMaxStrength : settings->maxStrength;
    if (Vector3_IsNearZero(&settings->shapeOffset, 1.0e-12f))
    {
        FromJolt(joltSettings.mShapeOffset, &settings->shapeOffset);
    }
    settings->backFaceMode = settings->backFaceMode == 0 ? static_cast<JPH_BackFaceMode>(joltSettings.mBackFaceMode)
                                                         : settings->backFaceMode;
    settings->predictiveContactDistance = settings->predictiveContactDistance == 0
                                                  ? joltSettings.mPredictiveContactDistance
                                                  : settings->predictiveContactDistance;
    settings->maxCollisionIterations = settings->maxCollisionIterations == 0 ? joltSettings.mMaxCollisionIterations
                                                                             : settings->maxCollisionIterations;
    settings->maxConstraintIterations = settings->maxConstraintIterations == 0 ? joltSettings.mMaxConstraintIterations
                                                                               : settings->maxConstraintIterations;
    settings->minTimeRemaining = settings->minTimeRemaining == 0 ? joltSettings.mMinTimeRemaining
                                                                 : settings->minTimeRemaining;
    settings->collisionTolerance = settings->collisionTolerance == 0 ? joltSettings.mCollisionTolerance
                                                                     : settings->collisionTolerance;
    settings->characterPadding = settings->characterPadding == 0 ? joltSettings.mCharacterPadding
                                                                 : settings->characterPadding;
    settings->maxNumHits = settings->maxNumHits == 0 ? joltSettings.mMaxNumHits : settings->maxNumHits;
    settings->hitReductionCosMaxAngle = settings->hitReductionCosMaxAngle == 0 ? joltSettings.mHitReductionCosMaxAngle
                                                                               : settings->hitReductionCosMaxAngle;
    settings->penetrationRecoverySpeed = settings->penetrationRecoverySpeed == 0
                                                 ? joltSettings.mPenetrationRecoverySpeed
                                                 : settings->penetrationRecoverySpeed;
    if (settings->innerBodyShape == nullptr && joltSettings.mInnerBodyShape != nullptr)
    {
        settings->innerBodyShape = reinterpret_cast<const JPH_Shape *>(joltSettings.mInnerBodyShape.GetPtr());
    }
    settings->innerBodyIDOverride = settings->innerBodyIDOverride == 0
                                            ? joltSettings.mInnerBodyIDOverride.GetIndexAndSequenceNumber()
                                            : settings->innerBodyIDOverride;
    settings->innerBodyLayer = settings->innerBodyLayer == 0 ? joltSettings.mInnerBodyLayer : settings->innerBodyLayer;
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
JPH_CAPI void JPH_CharacterVirtual_Destroy(JPH_CharacterVirtual *characterVirtual)
{
    if (characterVirtual != nullptr)
    {
        reinterpret_cast<JPH::CharacterVirtual *>(characterVirtual)->Release();
    }
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
        JPH::CharacterVsCharacterCollision *const joltCharacterVsCharacterCollision =
                reinterpret_cast<JPH::CharacterVsCharacterCollision *>(characterVsCharacterCollision);
        AsCharacterVirtual(character)->SetCharacterVsCharacterCollision(joltCharacterVsCharacterCollision);
    } else
    {
        AsCharacterVirtual(character)->SetCharacterVsCharacterCollision(nullptr);
    }
}

void JPH_CharacterVirtual_GetLinearVelocity(JPH_CharacterVirtual *character, Vector3 *velocity)
{
    FromJolt(AsCharacterVirtual(character)->GetLinearVelocity(), velocity);
}

void JPH_CharacterVirtual_SetLinearVelocity(JPH_CharacterVirtual *character, const Vector3 *velocity)
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

void JPH_CharacterVirtual_GetWorldTransform(JPH_CharacterVirtual *character, JPH_RMat44 *result)
{
    const JPH::RMat44 &mat = AsCharacterVirtual(character)->GetWorldTransform();
    FromJolt(mat, result);
}

void JPH_CharacterVirtual_GetCenterOfMassTransform(JPH_CharacterVirtual *character, JPH_RMat44 *result)
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

void JPH_CharacterVirtual_GetShapeOffset(JPH_CharacterVirtual *character, Vector3 *result)
{
    const JPH::Vec3 joltVector = AsCharacterVirtual(character)->GetShapeOffset();
    FromJolt(joltVector, result);
}

void JPH_CharacterVirtual_SetShapeOffset(JPH_CharacterVirtual *character, const Vector3 *value)
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
                                                           const Vector3 *desiredVelocity,
                                                           Vector3 *velocity)
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
                                         const JPH_PhysicsSystem *system,
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

bool JPH_CharacterVirtual_CanWalkStairs(JPH_CharacterVirtual *character, const Vector3 *linearVelocity)
{
    return AsCharacterVirtual(character)->CanWalkStairs(ToJolt(linearVelocity));
}

bool JPH_CharacterVirtual_WalkStairs(JPH_CharacterVirtual *character,
                                     const float deltaTime,
                                     const Vector3 *stepUp,
                                     const Vector3 *stepForward,
                                     const Vector3 *stepForwardTest,
                                     const Vector3 *stepDownExtra,
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
                                       const Vector3 *stepDown,
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
        explicit ManagedCharacterContactListener(const JPH_CharacterContactListener_Impl *impl): impl(impl) {}

        void OnAdjustBodyVelocity(const JPH::CharacterVirtual *inCharacter,
                                  const JPH::Body &inBody2,
                                  JPH::Vec3 &ioLinearVelocity,
                                  JPH::Vec3 &ioAngularVelocity) override
        {
            Vector3 linearVelocity;
            Vector3 angularVelocity;
            FromJolt(ioLinearVelocity, &linearVelocity);
            FromJolt(ioAngularVelocity, &angularVelocity);

            if (impl != nullptr && impl->OnAdjustBodyVelocity != nullptr)
            {
                impl->OnAdjustBodyVelocity(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnContactValidate != nullptr)
            {
                return impl->OnContactValidate(ToCharacterVirtual(inCharacter),
                                               inBodyID2.GetIndexAndSequenceNumber(),
                                               inSubShapeID2.GetValue());
            }

            return true;
        }

        bool OnCharacterContactValidate(const JPH::CharacterVirtual *inCharacter,
                                        const JPH::CharacterVirtual *inOtherCharacter,
                                        const JPH::SubShapeID &inSubShapeID2) override
        {
            if (impl != nullptr && impl->OnCharacterContactValidate != nullptr)
            {
                return impl->OnCharacterContactValidate(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnContactAdded != nullptr)
            {
                JPH_RVec3 contactPosition;
                Vector3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                impl->OnContactAdded(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnContactPersisted != nullptr)
            {
                JPH_RVec3 contactPosition;
                Vector3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                impl->OnContactPersisted(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnContactRemoved != nullptr)
            {
                impl->OnContactRemoved(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnCharacterContactAdded != nullptr)
            {
                JPH_RVec3 contactPosition;
                Vector3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                impl->OnCharacterContactAdded(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnCharacterContactPersisted != nullptr)
            {
                JPH_RVec3 contactPosition;
                Vector3 contactNormal;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);

                JPH_CharacterContactSettings settings = {};
                settings.canPushCharacter = ioSettings.mCanPushCharacter;
                settings.canReceiveImpulses = ioSettings.mCanReceiveImpulses;

                impl->OnCharacterContactPersisted(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnCharacterContactRemoved != nullptr)
            {
                impl->OnCharacterContactRemoved(ToCharacterVirtual(inCharacter),
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
            if (impl != nullptr && impl->OnContactSolve != nullptr)
            {
                JPH_RVec3 contactPosition;
                Vector3 contactNormal;
                Vector3 contactVelocity;
                Vector3 characterVelocity;
                Vector3 newCharacterVelocity;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);
                FromJolt(inContactVelocity, &contactVelocity);
                FromJolt(inCharacterVelocity, &characterVelocity);
                FromJolt(ioNewCharacterVelocity, &newCharacterVelocity);

                impl->OnContactSolve(ToCharacterVirtual(inCharacter),
                                     inBodyID2.GetIndexAndSequenceNumber(),
                                     inSubShapeID2.GetValue(),
                                     &contactPosition,
                                     &contactNormal,
                                     &contactVelocity,
                                     ToPhysicsMaterial(inContactMaterial),
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
            if (impl != nullptr && impl->OnCharacterContactSolve != nullptr)
            {
                JPH_RVec3 contactPosition;
                Vector3 contactNormal;
                Vector3 contactVelocity;
                Vector3 characterVelocity;
                Vector3 newCharacterVelocity;

                FromJolt(inContactPosition, &contactPosition);
                FromJolt(inContactNormal, &contactNormal);
                FromJolt(inContactVelocity, &contactVelocity);
                FromJolt(inCharacterVelocity, &characterVelocity);
                FromJolt(ioNewCharacterVelocity, &newCharacterVelocity);

                impl->OnCharacterContactSolve(ToCharacterVirtual(inCharacter),
                                              ToCharacterVirtual(inOtherCharacter),
                                              inSubShapeID2.GetValue(),
                                              &contactPosition,
                                              &contactNormal,
                                              &contactVelocity,
                                              ToPhysicsMaterial(inContactMaterial),
                                              &characterVelocity,
                                              &newCharacterVelocity);

                ioNewCharacterVelocity = ToJolt(newCharacterVelocity);
            }
        }

        const JPH_CharacterContactListener_Impl *impl = nullptr;
};

JPH_CharacterContactListener *JPH_CharacterContactListener_Create(const JPH_CharacterContactListener_Impl *impl)
{
    ManagedCharacterContactListener *const listener = new ManagedCharacterContactListener(impl);
    return reinterpret_cast<JPH_CharacterContactListener *>(listener);
}

void JPH_CharacterContactListener_Destroy(JPH_CharacterContactListener *listener)
{
    if (listener != nullptr)
    {
        delete reinterpret_cast<ManagedCharacterContactListener *>(listener);
    }
}

void JPH_CharacterContactListener_SetImpl(JPH_CharacterContactListener *listener,
                                          const JPH_CharacterContactListener_Impl *impl)
{
    reinterpret_cast<ManagedCharacterContactListener *>(listener)->impl = impl;
}

/* JPH_CharacterVsCharacterCollision */
class ManagedCharacterVsCharacterCollision final: public JPH::CharacterVsCharacterCollision
{
    public:
        static const JPH_CharacterVsCharacterCollision_Impl *s_Impl;
        void *userData = nullptr;

        explicit ManagedCharacterVsCharacterCollision(void *userData_): userData(userData_) {}

        void CollideCharacter(const JPH::CharacterVirtual *inCharacter,
                              JPH::RMat44Arg inCenterOfMassTransform,
                              const JPH::CollideShapeSettings &inCollideShapeSettings,
                              JPH::RVec3Arg inBaseOffset,
                              JPH::CollideShapeCollector & /*ioCollector*/) const override
        {
            if (s_Impl != nullptr && s_Impl->CollideCharacter != nullptr)
            {
                JPH_RMat44 centerOfMassTransform;
                JPH_RVec3 baseOffset;

                const JPH_CollideShapeSettings collideShapeSettings = FromJolt(inCollideShapeSettings);
                FromJolt(inCenterOfMassTransform, &centerOfMassTransform);
                FromJolt(inBaseOffset, &baseOffset);

                s_Impl->CollideCharacter(userData,
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
            if (s_Impl != nullptr && s_Impl->CastCharacter != nullptr)
            {
                JPH_RMat44 centerOfMassTransform;
                Vector3 direction;
                JPH_RVec3 baseOffset;

                const JPH_ShapeCastSettings shapeCastSettings = FromJolt(inShapeCastSettings);
                FromJolt(inCenterOfMassTransform, &centerOfMassTransform);
                FromJolt(inDirection, &direction);
                FromJolt(inBaseOffset, &baseOffset);

                s_Impl->CastCharacter(userData,
                                      ToCharacterVirtual(inCharacter),
                                      &centerOfMassTransform,
                                      &direction,
                                      &shapeCastSettings,
                                      &baseOffset);
            }
        }
};

const JPH_CharacterVsCharacterCollision_Impl *ManagedCharacterVsCharacterCollision::s_Impl = nullptr;

void JPH_CharacterVsCharacterCollision_SetImpl(const JPH_CharacterVsCharacterCollision_Impl *impl)
{
    ManagedCharacterVsCharacterCollision::s_Impl = impl;
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

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRendererSimple.h>

/* JPH_BodyDrawFilter */
class ManagedBodyDrawFilter final: public JPH::BodyDrawFilter
{
    public:
        static const JPH_BodyDrawFilter_Impl *s_Impl;
        void *userData = nullptr;

        explicit ManagedBodyDrawFilter(void *userData_): userData(userData_) {}

        [[nodiscard]] bool ShouldDraw([[maybe_unused]] const JPH::Body &inBody) const override
        {
            if (s_Impl != nullptr && s_Impl->ShouldDraw != nullptr)
            {
                return s_Impl->ShouldDraw(userData, reinterpret_cast<const JPH_Body *>(&inBody));
            }

            return true;
        }
};
const JPH_BodyDrawFilter_Impl *ManagedBodyDrawFilter::s_Impl = nullptr;

void JPH_BodyDrawFilter_SetImpl(const JPH_BodyDrawFilter_Impl *impl)
{
    ManagedBodyDrawFilter::s_Impl = impl;
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
        static const JPH_DebugRenderer_Impl *s_Impl;
        void *userData = nullptr;

        explicit ManagedDebugRendererSimple(void *userData_): userData(userData_) {}

        void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, const JPH::ColorArg inColor) override
        {
            if (s_Impl != nullptr && s_Impl->DrawLine != nullptr)
            {
                JPH_RVec3 from;
                JPH_RVec3 to;

                FromJolt(inFrom, &from);
                FromJolt(inTo, &to);

                s_Impl->DrawLine(userData, &from, &to, inColor.GetUInt32());
            }
        }

        void DrawTriangle(JPH::RVec3Arg inV1,
                          JPH::RVec3Arg inV2,
                          JPH::RVec3Arg inV3,
                          const JPH::ColorArg inColor,
                          ECastShadow inCastShadow = ECastShadow::Off) override
        {
            if (s_Impl != nullptr && s_Impl->DrawTriangle != nullptr)
            {
                JPH_RVec3 v1;
                JPH_RVec3 v2;
                JPH_RVec3 v3;

                FromJolt(inV1, &v1);
                FromJolt(inV2, &v2);
                FromJolt(inV3, &v3);

                s_Impl->DrawTriangle(userData,
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
            if (s_Impl != nullptr && s_Impl->DrawText3D != nullptr)
            {
                JPH_RVec3 position;

                FromJolt(inPosition, &position);

                s_Impl->DrawText3D(userData, &position, inString.data(), inColor.GetUInt32(), inHeight);
            }
        }
};

const JPH_DebugRenderer_Impl *ManagedDebugRendererSimple::s_Impl = nullptr;

void JPH_DebugRenderer_SetImpl(const JPH_DebugRenderer_Impl *impl)
{
    ManagedDebugRendererSimple::s_Impl = impl;
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
                                    const JPH_RMat44 *matrix,
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

void JPH_DebugRenderer_DrawCoordinateSystem(JPH_DebugRenderer *renderer, const JPH_RMat44 *matrix, const float size)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawCoordinateSystem(ToJolt(matrix), size);
}

void JPH_DebugRenderer_DrawPlane(JPH_DebugRenderer *renderer,
                                 const JPH_RVec3 *point,
                                 const Vector3 *normal,
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
                                          const JPH_RMat44 *matrix,
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
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawTriangle(
            ToJolt(v1),
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
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawBox(
            ToJolt(box),
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawBox2(JPH_DebugRenderer *renderer,
                                const JPH_RMat44 *matrix,
                                const JPH_AABox *box,
                                const JPH_Color color,
                                JPH_DebugRenderer_CastShadow castShadow,
                                JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawBox(
            ToJolt(matrix),
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
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawSphere(
            ToJolt(center),
            radius,
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawUnitSphere(JPH_DebugRenderer *renderer,
                                      const JPH_RMat44 *matrix,
                                      const JPH_Color color,
                                      JPH_DebugRenderer_CastShadow castShadow,
                                      JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawUnitSphere(
            ToJolt(matrix),
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawCapsule(JPH_DebugRenderer *renderer,
                                   const JPH_RMat44 *matrix,
                                   const float halfHeightOfCylinder,
                                   const float radius,
                                   const JPH_Color color,
                                   JPH_DebugRenderer_CastShadow castShadow,
                                   JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawCapsule(
            ToJolt(matrix),
            halfHeightOfCylinder,
            radius,
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawCylinder(JPH_DebugRenderer *renderer,
                                    const JPH_RMat44 *matrix,
                                    const float halfHeight,
                                    const float radius,
                                    const JPH_Color color,
                                    JPH_DebugRenderer_CastShadow castShadow,
                                    JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawCylinder(
            ToJolt(matrix),
            halfHeight,
            radius,
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawOpenCone(JPH_DebugRenderer *renderer,
                                    const JPH_RVec3 *top,
                                    const Vector3 *axis,
                                    const Vector3 *perpendicular,
                                    const float halfAngle,
                                    const float length,
                                    const JPH_Color color,
                                    JPH_DebugRenderer_CastShadow castShadow,
                                    JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawOpenCone(
            ToJolt(top),
            ToJolt(axis),
            ToJolt(perpendicular),
            halfAngle,
            length,
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawSwingConeLimits(JPH_DebugRenderer *renderer,
                                           const JPH_RMat44 *matrix,
                                           const float swingYHalfAngle,
                                           const float swingZHalfAngle,
                                           const float edgeLength,
                                           const JPH_Color color,
                                           JPH_DebugRenderer_CastShadow castShadow,
                                           JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawSwingConeLimits(
            ToJolt(matrix),
            swingYHalfAngle,
            swingZHalfAngle,
            edgeLength,
            JPH::Color(color),
            static_cast<JPH::DebugRenderer::ECastShadow>(castShadow),
            static_cast<JPH::DebugRenderer::EDrawMode>(drawMode));
}

void JPH_DebugRenderer_DrawSwingPyramidLimits(JPH_DebugRenderer *renderer,
                                              const JPH_RMat44 *matrix,
                                              const float minSwingYAngle,
                                              const float maxSwingYAngle,
                                              const float minSwingZAngle,
                                              const float maxSwingZAngle,
                                              const float edgeLength,
                                              const JPH_Color color,
                                              JPH_DebugRenderer_CastShadow castShadow,
                                              JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawSwingPyramidLimits(
            ToJolt(matrix),
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
                               const Vector3 *normal,
                               const Vector3 *axis,
                               const float minAngle,
                               const float maxAngle,
                               const JPH_Color color,
                               JPH_DebugRenderer_CastShadow castShadow,
                               JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawPie(
            ToJolt(center),
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
                                           const JPH_RMat44 *inMatrix,
                                           const float top,
                                           const float bottom,
                                           const float topRadius,
                                           const float bottomRadius,
                                           const JPH_Color color,
                                           JPH_DebugRenderer_CastShadow castShadow,
                                           JPH_DebugRenderer_DrawMode drawMode)
{
    reinterpret_cast<ManagedDebugRendererSimple *>(renderer)->DrawTaperedCylinder(
            ToJolt(inMatrix),
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

/* SkeletonPose */
JPH_SkeletonPose *JPH_SkeletonPose_Create()
{
    return ToSkeletonPose(new JPH::SkeletonPose());
}

void JPH_SkeletonPose_Destroy(JPH_SkeletonPose *pose)
{
    delete AsSkeletonPose(pose);
}

void JPH_SkeletonPose_SetSkeleton(JPH_SkeletonPose *pose, const JPH_Skeleton *skeleton)
{
    AsSkeletonPose(pose)->SetSkeleton(AsSkeleton(skeleton));
}

const JPH_Skeleton *JPH_SkeletonPose_GetSkeleton(const JPH_SkeletonPose *pose)
{
    return ToSkeleton(AsSkeletonPose(pose)->GetSkeleton());
}

void JPH_SkeletonPose_SetRootOffset(JPH_SkeletonPose *pose, const JPH_RVec3 *offset)
{
    AsSkeletonPose(pose)->SetRootOffset(ToJolt(offset));
}

void JPH_SkeletonPose_GetRootOffset(const JPH_SkeletonPose *pose, JPH_RVec3 *result)
{
    FromJolt(AsSkeletonPose(pose)->GetRootOffset(), result);
}

size_t JPH_SkeletonPose_GetJointCount(const JPH_SkeletonPose *pose)
{
    return AsSkeletonPose(pose)->GetJointCount();
}

void JPH_SkeletonPose_GetJointState(const JPH_SkeletonPose *pose,
                                    const int index,
                                    Vector3 *outTranslation,
                                    JPH_Quat *outRotation)
{
    const JPH::SkeletalAnimation::JointState &joint = AsSkeletonPose(pose)->GetJoint(index);
    if (outTranslation != nullptr)
    {
        FromJolt(joint.mTranslation, outTranslation);
    }
    if (outRotation != nullptr)
    {
        FromJolt(joint.mRotation, outRotation);
    }
}

void JPH_SkeletonPose_SetJointState(JPH_SkeletonPose *pose,
                                    const int index,
                                    const Vector3 *translation,
                                    const JPH_Quat *rotation)
{
    JPH::SkeletalAnimation::JointState &joint = AsSkeletonPose(pose)->GetJoint(index);
    if (translation != nullptr)
    {
        joint.mTranslation = ToJolt(translation);
    }
    if (rotation != nullptr)
    {
        joint.mRotation = ToJolt(rotation);
    }
}

void JPH_SkeletonPose_GetJointMatrix(const JPH_SkeletonPose *pose, const int index, JPH_Mat44 *result)
{
    FromJolt(AsSkeletonPose(pose)->GetJointMatrix(index), result);
}

void JPH_SkeletonPose_SetJointMatrix(JPH_SkeletonPose *pose, const int index, const JPH_Mat44 *matrix)
{
    AsSkeletonPose(pose)->GetJointMatrix(index) = ToJolt(matrix);
}

void JPH_SkeletonPose_GetJointMatrices(const JPH_SkeletonPose *pose, JPH_Mat44 *outMatrices, const size_t count)
{
    const JPH::Array<JPH::Mat44> &matrices = AsSkeletonPose(pose)->GetJointMatrices();
    const size_t copyCount = matrices.size() < count ? matrices.size() : count;
    for (size_t i = 0; i < copyCount; i++)
    {
        FromJolt(matrices.at(i), &outMatrices[i]);
    }
}

void JPH_SkeletonPose_SetJointMatrices(JPH_SkeletonPose *pose, const JPH_Mat44 *matrices, const size_t count)
{
    JPH::Array<JPH::Mat44> &poseMatrices = AsSkeletonPose(pose)->GetJointMatrices();
    const size_t copyCount = poseMatrices.size() < count ? poseMatrices.size() : count;
    for (size_t i = 0; i < copyCount; i++)
    {
        poseMatrices.at(i) = ToJolt(&matrices[i]);
    }
}

void JPH_SkeletonPose_CalculateJointMatrices(JPH_SkeletonPose *pose)
{
    AsSkeletonPose(pose)->CalculateJointMatrices();
}

void JPH_SkeletonPose_CalculateJointStates(JPH_SkeletonPose *pose)
{
    AsSkeletonPose(pose)->CalculateJointStates();
}

void JPH_SkeletonPose_CalculateLocalSpaceJointMatrices(const JPH_SkeletonPose *pose, JPH_Mat44 *outMatrices)
{
    const uint32_t count = AsSkeletonPose(pose)->GetJointCount();
    JPH::Array<JPH::Mat44> matrices(count);
    AsSkeletonPose(pose)->CalculateLocalSpaceJointMatrices(matrices.data());
    for (uint32_t i = 0; i < count; i++)
    {
        FromJolt(matrices.at(i), &outMatrices[i]);
    }
}

/* SkeletalAnimation */
JPH_SkeletalAnimation *JPH_SkeletalAnimation_Create()
{
    JPH::SkeletalAnimation *const animation = new JPH::SkeletalAnimation();
    animation->AddRef();
    return ToSkeletalAnimation(animation);
}

void JPH_SkeletalAnimation_Destroy(JPH_SkeletalAnimation *animation)
{
    if (animation != nullptr)
    {
        AsSkeletalAnimation(animation)->Release();
    }
}

float JPH_SkeletalAnimation_GetDuration(const JPH_SkeletalAnimation *animation)
{
    return AsSkeletalAnimation(animation)->GetDuration();
}

bool JPH_SkeletalAnimation_IsLooping(const JPH_SkeletalAnimation *animation)
{
    return AsSkeletalAnimation(animation)->IsLooping();
}

void JPH_SkeletalAnimation_SetIsLooping(JPH_SkeletalAnimation *animation, const bool looping)
{
    AsSkeletalAnimation(animation)->SetIsLooping(looping);
}

void JPH_SkeletalAnimation_ScaleJoints(JPH_SkeletalAnimation *animation, const float scale)
{
    AsSkeletalAnimation(animation)->ScaleJoints(scale);
}

void JPH_SkeletalAnimation_Sample(const JPH_SkeletalAnimation *animation, const float time, JPH_SkeletonPose *pose)
{
    AsSkeletalAnimation(animation)->Sample(time, *AsSkeletonPose(pose));
}

size_t JPH_SkeletalAnimation_GetAnimatedJointCount(const JPH_SkeletalAnimation *animation)
{
    return AsSkeletalAnimation(animation)->GetAnimatedJoints().size();
}

void JPH_SkeletalAnimation_AddAnimatedJoint(JPH_SkeletalAnimation *animation, const char *jointName)
{
    JPH::Array<JPH::SkeletalAnimation::AnimatedJoint> &joints = AsSkeletalAnimation(animation)->GetAnimatedJoints();
    joints.push_back(JPH::SkeletalAnimation::AnimatedJoint());
    joints.back().mJointName = jointName;
}

void JPH_SkeletalAnimation_AddKeyframe(JPH_SkeletalAnimation *animation,
                                       const size_t jointIndex,
                                       const float time,
                                       const Vector3 *translation,
                                       const JPH_Quat *rotation)
{
    JPH::Array<JPH::SkeletalAnimation::AnimatedJoint> &joints = AsSkeletalAnimation(animation)->GetAnimatedJoints();
    if (jointIndex < joints.size())
    {
        JPH::SkeletalAnimation::Keyframe keyframe;
        keyframe.mTime = time;
        if (translation != nullptr)
        {
            keyframe.mTranslation = ToJolt(translation);
        }
        if (rotation != nullptr)
        {
            keyframe.mRotation = ToJolt(rotation);
        }
        joints.at(jointIndex).mKeyframes.push_back(keyframe);
    }
}

/* SkeletonMapper */
JPH_SkeletonMapper *JPH_SkeletonMapper_Create()
{
    JPH::SkeletonMapper *const mapper = new JPH::SkeletonMapper();
    mapper->AddRef();
    return ToSkeletonMapper(mapper);
}

void JPH_SkeletonMapper_Destroy(JPH_SkeletonMapper *mapper)
{
    if (mapper != nullptr)
    {
        AsSkeletonMapper(mapper)->Release();
    }
}

void JPH_SkeletonMapper_Initialize(JPH_SkeletonMapper *mapper,
                                   const JPH_Skeleton *skeleton1,
                                   const JPH_Mat44 *neutralPose1,
                                   const JPH_Skeleton *skeleton2,
                                   const JPH_Mat44 *neutralPose2)
{
    AsSkeletonMapper(mapper)->Initialize(AsSkeleton(skeleton1),
                                         reinterpret_cast<const JPH::Mat44 *>(neutralPose1),
                                         AsSkeleton(skeleton2),
                                         reinterpret_cast<const JPH::Mat44 *>(neutralPose2));
}

void JPH_SkeletonMapper_LockAllTranslations(JPH_SkeletonMapper *mapper,
                                            const JPH_Skeleton *skeleton2,
                                            const JPH_Mat44 *neutralPose2)
{
    AsSkeletonMapper(mapper)->LockAllTranslations(AsSkeleton(skeleton2),
                                                  reinterpret_cast<const JPH::Mat44 *>(neutralPose2));
}

void JPH_SkeletonMapper_LockTranslations(JPH_SkeletonMapper *mapper,
                                         const JPH_Skeleton *skeleton2,
                                         const bool *lockedTranslations,
                                         const JPH_Mat44 *neutralPose2)
{
    AsSkeletonMapper(mapper)->LockTranslations(AsSkeleton(skeleton2),
                                               lockedTranslations,
                                               reinterpret_cast<const JPH::Mat44 *>(neutralPose2));
}

void JPH_SkeletonMapper_Map(const JPH_SkeletonMapper *mapper,
                            const JPH_Mat44 *pose1ModelSpace,
                            const JPH_Mat44 *pose2LocalSpace,
                            JPH_Mat44 *outPose2ModelSpace)
{
    const JPH::Array<JPH::SkeletonMapper::Mapping> &mappings = AsSkeletonMapper(mapper)->GetMappings();
    if (mappings.empty())
    {
        return;
    }

    AsSkeletonMapper(mapper)->Map(reinterpret_cast<const JPH::Mat44 *>(pose1ModelSpace),
                                  reinterpret_cast<const JPH::Mat44 *>(pose2LocalSpace),
                                  reinterpret_cast<JPH::Mat44 *>(outPose2ModelSpace));
}

void JPH_SkeletonMapper_MapReverse(const JPH_SkeletonMapper *mapper,
                                   const JPH_Mat44 *pose2ModelSpace,
                                   JPH_Mat44 *outPose1ModelSpace)
{
    const JPH::Array<JPH::SkeletonMapper::Mapping> &mappings = AsSkeletonMapper(mapper)->GetMappings();
    if (mappings.empty())
    {
        return;
    }

    AsSkeletonMapper(mapper)->MapReverse(reinterpret_cast<const JPH::Mat44 *>(pose2ModelSpace),
                                         reinterpret_cast<JPH::Mat44 *>(outPose1ModelSpace));
}

int JPH_SkeletonMapper_GetMappedJointIndex(const JPH_SkeletonMapper *mapper, const int joint1Index)
{
    return AsSkeletonMapper(mapper)->GetMappedJointIdx(joint1Index);
}

bool JPH_SkeletonMapper_IsJointTranslationLocked(const JPH_SkeletonMapper *mapper, const int joint2Index)
{
    return AsSkeletonMapper(mapper)->IsJointTranslationLocked(joint2Index);
}

/* RagdollSettings */
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
                                                      const JPH_Mat44 *jointMatrices,
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

/* Ragdoll */
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

void JPH_Ragdoll_SetPose(JPH_Ragdoll *ragdoll, const JPH_SkeletonPose *pose, const bool lockBodies)
{
    AsRagdoll(ragdoll)->SetPose(*AsSkeletonPose(pose), lockBodies);
}

void JPH_Ragdoll_SetPose2(JPH_Ragdoll *ragdoll,
                          const JPH_RVec3 *rootOffset,
                          const JPH_Mat44 *jointMatrices,
                          const bool lockBodies)
{
    AsRagdoll(ragdoll)->SetPose(ToJolt(rootOffset), reinterpret_cast<const JPH::Mat44 *>(jointMatrices), lockBodies);
}

void JPH_Ragdoll_GetPose(JPH_Ragdoll *ragdoll, JPH_SkeletonPose *outPose, const bool lockBodies)
{
    AsRagdoll(ragdoll)->GetPose(*AsSkeletonPose(outPose), lockBodies);
}

void JPH_Ragdoll_GetPose2(JPH_Ragdoll *ragdoll,
                          JPH_RVec3 *outRootOffset,
                          JPH_Mat44 *outJointMatrices,
                          const bool lockBodies)
{
    JPH::RVec3 rootOffset;
    AsRagdoll(ragdoll)->GetPose(rootOffset, reinterpret_cast<JPH::Mat44 *>(outJointMatrices), lockBodies);
    if (outRootOffset != nullptr)
    {
        FromJolt(rootOffset, outRootOffset);
    }
}

void JPH_Ragdoll_DriveToPoseUsingMotors(JPH_Ragdoll *ragdoll, const JPH_SkeletonPose *pose)
{
    AsRagdoll(ragdoll)->DriveToPoseUsingMotors(*AsSkeletonPose(pose));
}

void JPH_Ragdoll_DriveToPoseUsingKinematics(JPH_Ragdoll *ragdoll,
                                            const JPH_SkeletonPose *pose,
                                            const float deltaTime,
                                            const bool lockBodies)
{
    AsRagdoll(ragdoll)->DriveToPoseUsingKinematics(*AsSkeletonPose(pose), deltaTime, lockBodies);
}

size_t JPH_Ragdoll_GetBodyCount(const JPH_Ragdoll *ragdoll)
{
    return AsRagdoll(ragdoll)->GetBodyCount();
}

JPH_BodyID JPH_Ragdoll_GetBodyID(const JPH_Ragdoll *ragdoll, const int bodyIndex)
{
    return AsRagdoll(ragdoll)->GetBodyID(bodyIndex).GetIndexAndSequenceNumber();
}

size_t JPH_Ragdoll_GetConstraintCount(const JPH_Ragdoll *ragdoll)
{
    return AsRagdoll(ragdoll)->GetConstraintCount();
}

JPH_TwoBodyConstraint *JPH_Ragdoll_GetConstraint(JPH_Ragdoll *ragdoll, const int constraintIndex)
{
    return ToTwoBodyConstraint(AsRagdoll(ragdoll)->GetConstraint(constraintIndex));
}

void JPH_Ragdoll_GetRootTransform(const JPH_Ragdoll *ragdoll,
                                  JPH_RVec3 *outPosition,
                                  JPH_Quat *outRotation,
                                  const bool lockBodies)
{
    JPH::RVec3 position{};
    JPH::Quat rotation{};
    AsRagdoll(ragdoll)->GetRootTransform(position, rotation, lockBodies);
    if (outPosition != nullptr)
    {
        FromJolt(position, outPosition);
    }
    if (outRotation != nullptr)
    {
        FromJolt(rotation, outRotation);
    }
}

const JPH_RagdollSettings *JPH_Ragdoll_GetRagdollSettings(const JPH_Ragdoll *ragdoll)
{
    return ToRagdollSettings(AsRagdoll(ragdoll)->GetRagdollSettings());
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
        result->impulses = new JPH_CollisionEstimationResultImpulse[joltResult.mImpulses.size()];
        for (uint32_t i = 0; i < result->impulseCount; i++)
        {
            result->impulses[i].contactImpulse = joltResult.mImpulses.at(i).mContactImpulse;
            result->impulses[i].frictionImpulse1 = joltResult.mImpulses.at(i).mFrictionImpulse1;
            result->impulses[i].frictionImpulse2 = joltResult.mImpulses.at(i).mFrictionImpulse2;
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

void JPH_WheelSettings_GetPosition(const JPH_WheelSettings *settings, Vector3 *result)
{
    FromJolt(AsWheelSettings(settings)->mPosition, result);
}

void JPH_WheelSettings_SetPosition(JPH_WheelSettings *settings, const Vector3 *value)
{
    AsWheelSettings(settings)->mPosition = ToJolt(value);
}

void JPH_WheelSettings_GetSuspensionForcePoint(const JPH_WheelSettings *settings, Vector3 *result)
{
    FromJolt(AsWheelSettings(settings)->mSuspensionForcePoint, result);
}

void JPH_WheelSettings_SetSuspensionForcePoint(JPH_WheelSettings *settings, const Vector3 *value)
{
    AsWheelSettings(settings)->mSuspensionForcePoint = ToJolt(value);
}

void JPH_WheelSettings_GetSuspensionDirection(const JPH_WheelSettings *settings, Vector3 *result)
{
    FromJolt(AsWheelSettings(settings)->mSuspensionDirection, result);
}

void JPH_WheelSettings_SetSuspensionDirection(JPH_WheelSettings *settings, const Vector3 *value)
{
    AsWheelSettings(settings)->mSuspensionDirection = ToJolt(value);
}

void JPH_WheelSettings_GetSteeringAxis(const JPH_WheelSettings *settings, Vector3 *result)
{
    FromJolt(AsWheelSettings(settings)->mSteeringAxis, result);
}

void JPH_WheelSettings_SetSteeringAxis(JPH_WheelSettings *settings, const Vector3 *value)
{
    AsWheelSettings(settings)->mSteeringAxis = ToJolt(value);
}

void JPH_WheelSettings_GetWheelUp(const JPH_WheelSettings *settings, Vector3 *result)
{
    FromJolt(AsWheelSettings(settings)->mWheelUp, result);
}

void JPH_WheelSettings_SetWheelUp(JPH_WheelSettings *settings, const Vector3 *value)
{
    AsWheelSettings(settings)->mWheelUp = ToJolt(value);
}

void JPH_WheelSettings_GetWheelForward(const JPH_WheelSettings *settings, Vector3 *result)
{
    FromJolt(AsWheelSettings(settings)->mWheelForward, result);
}

void JPH_WheelSettings_SetWheelForward(JPH_WheelSettings *settings, const Vector3 *value)
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

void JPH_Wheel_GetContactPointVelocity(const JPH_Wheel *wheel, Vector3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactPointVelocity(), result);
}

void JPH_Wheel_GetContactNormal(const JPH_Wheel *wheel, Vector3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactNormal(), result);
}

void JPH_Wheel_GetContactLongitudinal(const JPH_Wheel *wheel, Vector3 *result)
{
    FromJolt(AsWheel(wheel)->GetContactLongitudinal(), result);
}

void JPH_Wheel_GetContactLateral(const JPH_Wheel *wheel, Vector3 *result)
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

/* VehicleEngineSettings */
void JPH_VehicleEngineSettings_Init(JPH_VehicleEngineSettings *settings)
{
    JPH_ASSERT(settings);

    const JPH::VehicleEngineSettings joltSettings{};
    settings->maxTorque = joltSettings.mMaxTorque;
    settings->minRPM = joltSettings.mMinRPM;
    settings->maxRPM = joltSettings.mMaxRPM;
    settings->inertia = joltSettings.mInertia;
    settings->angularDamping = joltSettings.mAngularDamping;

    // Copy default normalized torque to JPH_VehicleEngineSettings->normalizedTorque
    const JPH::LinearCurve &joltNormalizedTorque = joltSettings.mNormalizedTorque;
    const size_t pointCount = joltNormalizedTorque.mPoints.size();
    JPH_LinearCurve *normalizedTorque = JPH_LinearCurve_Create();
    JPH_LinearCurve_Reserve(normalizedTorque, pointCount);
    for (size_t i = 0; i < pointCount; ++i)
    {
        const JPH::LinearCurve::Point point = joltNormalizedTorque.mPoints.at(i);
        JPH_LinearCurve_AddPoint(normalizedTorque, point.mX, point.mY);
    }
    settings->normalizedTorque = normalizedTorque;
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
    settings->normalizedTorque = ToLinearCurve(&joltSettings.mNormalizedTorque);
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
    joltSettings->mNormalizedTorque = *AsLinearCurve(settings->normalizedTorque);
}

/* VehicleEngine */
void JPH_VehicleEngine_ClampRPM(JPH_VehicleEngine *engine)
{
    AsVehicleEngine(engine)->ClampRPM();
}

float JPH_VehicleEngine_GetCurrentRPM(const JPH_VehicleEngine *engine)
{
    return AsVehicleEngine(engine)->GetCurrentRPM();
}

void JPH_VehicleEngine_SetCurrentRPM(JPH_VehicleEngine *engine, const float rpm)
{
    AsVehicleEngine(engine)->SetCurrentRPM(rpm);
}

float JPH_VehicleEngine_GetAngularVelocity(const JPH_VehicleEngine *engine)
{
    return AsVehicleEngine(engine)->GetAngularVelocity();
}

float JPH_VehicleEngine_GetTorque(const JPH_VehicleEngine *engine, const float acceleration)
{
    return AsVehicleEngine(engine)->GetTorque(acceleration);
}

void JPH_VehicleEngine_ApplyTorque(JPH_VehicleEngine *engine, const float torque, const float deltaTime)
{
    AsVehicleEngine(engine)->ApplyTorque(torque, deltaTime);
}

void JPH_VehicleEngine_ApplyDamping(JPH_VehicleEngine *engine, const float deltaTime)
{
    AsVehicleEngine(engine)->ApplyDamping(deltaTime);
}

bool JPH_VehicleEngine_AllowSleep(const JPH_VehicleEngine *engine)
{
    return AsVehicleEngine(engine)->AllowSleep();
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

/* VehicleTransmissionSettings */
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
    return AsVehicleTransmissionSettings(settings)->mGearRatios.at(index);
}

void JPH_VehicleTransmissionSettings_SetGearRatio(JPH_VehicleTransmissionSettings *settings,
                                                  const uint32_t index,
                                                  const float value)
{
    AsVehicleTransmissionSettings(settings)->mGearRatios.at(index) = value;
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
        AsVehicleTransmissionSettings(settings)->mGearRatios.at(i) = values[i];
    }
}

uint32_t JPH_VehicleTransmissionSettings_GetReverseGearRatioCount(const JPH_VehicleTransmissionSettings *settings)
{
    return static_cast<uint32_t>(AsVehicleTransmissionSettings(settings)->mReverseGearRatios.size());
}

float JPH_VehicleTransmissionSettings_GetReverseGearRatio(const JPH_VehicleTransmissionSettings *settings,
                                                          const uint32_t index)
{
    return AsVehicleTransmissionSettings(settings)->mReverseGearRatios.at(index);
}

void JPH_VehicleTransmissionSettings_SetReverseGearRatio(JPH_VehicleTransmissionSettings *settings,
                                                         const uint32_t index,
                                                         const float value)
{
    AsVehicleTransmissionSettings(settings)->mReverseGearRatios.at(index) = value;
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
        AsVehicleTransmissionSettings(settings)->mReverseGearRatios.at(i) = values[i];
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

/* VehicleTransmission */
void JPH_VehicleTransmission_Set(JPH_VehicleTransmission *transmission,
                                 const int currentGear,
                                 const float clutchFriction)
{
    AsVehicleTransmission(transmission)->Set(currentGear, clutchFriction);
}

void JPH_VehicleTransmission_Update(JPH_VehicleTransmission *transmission,
                                    const float deltaTime,
                                    const float currentRPM,
                                    const float forwardInput,
                                    const bool canShiftUp)
{
    AsVehicleTransmission(transmission)->Update(deltaTime, currentRPM, forwardInput, canShiftUp);
}

int JPH_VehicleTransmission_GetCurrentGear(const JPH_VehicleTransmission *transmission)
{
    return AsVehicleTransmission(transmission)->GetCurrentGear();
}

float JPH_VehicleTransmission_GetClutchFriction(const JPH_VehicleTransmission *transmission)
{
    return AsVehicleTransmission(transmission)->GetClutchFriction();
}

bool JPH_VehicleTransmission_IsSwitchingGear(const JPH_VehicleTransmission *transmission)
{
    return AsVehicleTransmission(transmission)->IsSwitchingGear();
}

float JPH_VehicleTransmission_GetCurrentRatio(const JPH_VehicleTransmission *transmission)
{
    return AsVehicleTransmission(transmission)->GetCurrentRatio();
}

bool JPH_VehicleTransmission_AllowSleep(const JPH_VehicleTransmission *transmission)
{
    return AsVehicleTransmission(transmission)->AllowSleep();
}

/* VehicleCollisionTester */
void JPH_VehicleCollisionTester_Destroy(const JPH_VehicleCollisionTester *tester)
{
    if (tester != nullptr)
    {
        AsVehicleCollisionTester(tester)->Release();
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
                                                                    const Vector3 *up,
                                                                    const float maxSlopeAngle)
{
    JPH::VehicleCollisionTesterRay *const tester = new JPH::VehicleCollisionTesterRay(layer, ToJolt(up), maxSlopeAngle);
    tester->AddRef();

    return ToVehicleCollisionTesterRay(tester);
}

JPH_VehicleCollisionTesterCastSphere *JPH_VehicleCollisionTesterCastSphere_Create(const JPH_ObjectLayer layer,
                                                                                  const float radius,
                                                                                  const Vector3 *up,
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
    JPH::VehicleCollisionTesterCastCylinder *const tester =
            new JPH::VehicleCollisionTesterCastCylinder(layer, convexRadiusFraction);
    tester->AddRef();

    return ToVehicleCollisionTesterCastCylinder(tester);
}

/* VehicleConstraintSettings */
static inline void JPH_VehicleConstraintSettings_FromJolt(JPH_VehicleConstraintSettings *settings,
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

static inline void JPH_VehicleConstraintSettings_ToJolt(JPH::VehicleConstraintSettings *joltSettings,
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
        joltSettings->mWheels.at(i) = AsWheelSettings(settings->wheels[i]);
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
            joltSettings->mAntiRollBars.at(i) = joltAntiRollBar;
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

void JPH_VehicleConstraint_OverrideGravity(JPH_VehicleConstraint *constraint, const Vector3 *value)
{
    AsVehicleConstraint(constraint)->OverrideGravity(ToJolt(value));
}

bool JPH_VehicleConstraint_IsGravityOverridden(const JPH_VehicleConstraint *constraint)
{
    return AsVehicleConstraint(constraint)->IsGravityOverridden();
}

void JPH_VehicleConstraint_GetGravityOverride(const JPH_VehicleConstraint *constraint, Vector3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetGravityOverride(), result);
}

void JPH_VehicleConstraint_ResetGravityOverride(JPH_VehicleConstraint *constraint)
{
    AsVehicleConstraint(constraint)->ResetGravityOverride();
}

void JPH_VehicleConstraint_GetLocalForward(const JPH_VehicleConstraint *constraint, Vector3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetLocalForward(), result);
}

void JPH_VehicleConstraint_GetLocalUp(const JPH_VehicleConstraint *constraint, Vector3 *result)
{
    FromJolt(AsVehicleConstraint(constraint)->GetLocalUp(), result);
}

void JPH_VehicleConstraint_GetWorldUp(const JPH_VehicleConstraint *constraint, Vector3 *result)
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
                                              Vector3 *outForward,
                                              Vector3 *outUp,
                                              Vector3 *outRight)
{
    JPH::Vec3 forward{};
    JPH::Vec3 up{};
    JPH::Vec3 right{};

    AsVehicleConstraint(constraint)->GetWheelLocalBasis(AsWheel(wheel), forward, up, right);
    FromJolt(forward, outForward);
    FromJolt(up, outUp);
    FromJolt(right, outRight);
}

void JPH_VehicleConstraint_GetWheelLocalTransform(JPH_VehicleConstraint *constraint,
                                                  const uint32_t wheelIndex,
                                                  const Vector3 *wheelRight,
                                                  const Vector3 *wheelUp,
                                                  JPH_Mat44 *result)
{
    const JPH::Mat44 joltResult =
            AsVehicleConstraint(constraint)->GetWheelLocalTransform(wheelIndex, ToJolt(wheelRight), ToJolt(wheelUp));
    FromJolt(joltResult, result);
}

void JPH_VehicleConstraint_GetWheelWorldTransform(JPH_VehicleConstraint *constraint,
                                                  const uint32_t wheelIndex,
                                                  const Vector3 *wheelRight,
                                                  const Vector3 *wheelUp,
                                                  JPH_RMat44 *result)
{
    const JPH::RMat44 joltResult =
            AsVehicleConstraint(constraint)->GetWheelWorldTransform(wheelIndex, ToJolt(wheelRight), ToJolt(wheelUp));
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

const JPH_LinearCurve *JPH_WheelSettingsWV_GetLongitudinalFriction(const JPH_WheelSettingsWV *settings)
{
    return ToLinearCurve(&AsWheelSettingsWV(settings)->mLongitudinalFriction);
}

void JPH_WheelSettingsWV_SetLongitudinalFriction(JPH_WheelSettingsWV *settings, const JPH_LinearCurve *value)
{
    AsWheelSettingsWV(settings)->mLongitudinalFriction = *AsLinearCurve(value);
}

const JPH_LinearCurve *JPH_WheelSettingsWV_GetLateralFriction(const JPH_WheelSettingsWV *settings)
{
    return ToLinearCurve(&AsWheelSettingsWV(settings)->mLateralFriction);
}

void JPH_WheelSettingsWV_SetLateralFriction(JPH_WheelSettingsWV *settings, const JPH_LinearCurve *value)
{
    AsWheelSettingsWV(settings)->mLateralFriction = *AsLinearCurve(value);
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
                                             AsWheeledVehicleControllerSettings(settings)->mDifferentials.at(index));
}

void JPH_WheeledVehicleControllerSettings_SetDifferential(JPH_WheeledVehicleControllerSettings *settings,
                                                          uint32_t index,
                                                          const JPH_VehicleDifferentialSettings *value)
{
    JPH_ASSERT(settings);
    JPH_ASSERT(value);

    const JPH::VehicleDifferentialSettings joltDifferential = JPH_VehicleDifferentialSettings_ToJolt(*value);
    AsWheeledVehicleControllerSettings(settings)->mDifferentials.at(index) = joltDifferential;
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
        AsWheeledVehicleControllerSettings(settings)->mDifferentials.at(i) = joltDifferential;
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

void JPH_WheeledVehicleController_SetTireMaxImpulseCallback(JPH_WheeledVehicleController *controller,
                                                            JPH_TireMaxImpulseCallback tireMaxImpulseCallback)
{
    AsWheeledVehicleController(controller)
            ->SetTireMaxImpulseCallback([&tireMaxImpulseCallback](const uint32_t inWheelIndex,
                                                                  float &outLongitudinalImpulse,
                                                                  float &outLateralImpulse,
                                                                  const float inSuspensionImpulse,
                                                                  const float inLongitudinalFriction,
                                                                  const float inLateralFriction,
                                                                  const float inLongitudinalSlip,
                                                                  const float inLateralSlip,
                                                                  const float inDeltaTime) -> void {
                tireMaxImpulseCallback(inWheelIndex,
                                       &outLongitudinalImpulse,
                                       &outLateralImpulse,
                                       inSuspensionImpulse,
                                       inLongitudinalFriction,
                                       inLateralFriction,
                                       inLongitudinalSlip,
                                       inLateralSlip,
                                       inDeltaTime);
            });
}

const JPH_VehicleEngine *JPH_WheeledVehicleController_GetEngine(const JPH_WheeledVehicleController *controller)
{
    return ToVehicleEngine(&AsWheeledVehicleController(controller)->GetEngine());
}

const JPH_VehicleTransmission *JPH_WheeledVehicleController_GetTransmission(const JPH_WheeledVehicleController
                                                                                    *controller)
{
    return ToVehicleTransmission(&AsWheeledVehicleController(controller)->GetTransmission());
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

const JPH_VehicleEngine *JPH_TrackedVehicleController_GetEngine(const JPH_TrackedVehicleController *controller)
{
    return ToVehicleEngine(&AsTrackedVehicleController(controller)->GetEngine());
}

const JPH_VehicleTransmission *JPH_TrackedVehicleController_GetTransmission(const JPH_TrackedVehicleController
                                                                                    *controller)
{
    return ToVehicleTransmission(&AsTrackedVehicleController(controller)->GetTransmission());
}

/* VehicleTrack */
void JPH_VehicleTrackSettings_Init(JPH_VehicleTrackSettings *settings)
{
    settings->drivenWheel = 0;
    settings->wheels = nullptr;
    settings->wheelsCount = 0;
    settings->inertia = 10.0f;
    settings->angularDamping = 0.5f;
    settings->maxBrakeTorque = 15000.0f;
    settings->differentialRatio = 6.0f;
}

float JPH_VehicleTrack_GetAngularVelocity(const JPH_VehicleTrack *track)
{
    return AsVehicleTrack(track)->mAngularVelocity;
}

void JPH_VehicleTrack_SetAngularVelocity(JPH_VehicleTrack *track, const float velocity)
{
    AsVehicleTrack(track)->mAngularVelocity = velocity;
}

uint32_t JPH_VehicleTrack_GetDrivenWheel(const JPH_VehicleTrack *track)
{
    return AsVehicleTrack(track)->mDrivenWheel;
}

float JPH_VehicleTrack_GetInertia(const JPH_VehicleTrack *track)
{
    return AsVehicleTrack(track)->mInertia;
}

float JPH_VehicleTrack_GetAngularDamping(const JPH_VehicleTrack *track)
{
    return AsVehicleTrack(track)->mAngularDamping;
}

float JPH_VehicleTrack_GetMaxBrakeTorque(const JPH_VehicleTrack *track)
{
    return AsVehicleTrack(track)->mMaxBrakeTorque;
}

float JPH_VehicleTrack_GetDifferentialRatio(const JPH_VehicleTrack *track)
{
    return AsVehicleTrack(track)->mDifferentialRatio;
}

const JPH_VehicleTrack *JPH_TrackedVehicleController_GetTrack(const JPH_TrackedVehicleController *controller,
                                                              const JPH_TrackSide side)
{
    return ToVehicleTrack(&AsTrackedVehicleController(controller)->GetTracks()[static_cast<int>(side)]);
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

/* LinearCurve */
JPH_LinearCurve *JPH_LinearCurve_Create()
{
    JPH::LinearCurve *curve = new JPH::LinearCurve();
    return ToLinearCurve(curve);
}

void JPH_LinearCurve_Destroy(JPH_LinearCurve *curve)
{
    if (curve != nullptr)
    {
        delete AsLinearCurve(curve);
    }
}

void JPH_LinearCurve_Clear(JPH_LinearCurve *curve)
{
    JPH_ASSERT(curve);
    AsLinearCurve(curve)->Clear();
}

void JPH_LinearCurve_Reserve(JPH_LinearCurve *curve, uint32_t numPoints)
{
    JPH_ASSERT(curve);
    AsLinearCurve(curve)->Reserve(numPoints);
}

void JPH_LinearCurve_AddPoint(JPH_LinearCurve *curve, float x, float y)
{
    JPH_ASSERT(curve);
    AsLinearCurve(curve)->AddPoint(x, y);
}

void JPH_LinearCurve_Sort(JPH_LinearCurve *curve)
{
    JPH_ASSERT(curve);
    AsLinearCurve(curve)->Sort();
}

float JPH_LinearCurve_GetMinX(const JPH_LinearCurve *curve)
{
    JPH_ASSERT(curve);
    return AsLinearCurve(curve)->GetMinX();
}

float JPH_LinearCurve_GetMaxX(const JPH_LinearCurve *curve)
{
    JPH_ASSERT(curve);
    return AsLinearCurve(curve)->GetMaxX();
}

float JPH_LinearCurve_GetValue(const JPH_LinearCurve *curve, float x)
{
    JPH_ASSERT(curve);
    return AsLinearCurve(curve)->GetValue(x);
}

size_t JPH_LinearCurve_GetPointCount(const JPH_LinearCurve *curve)
{
    JPH_ASSERT(curve);
    return AsLinearCurve(curve)->mPoints.size();
}

void JPH_LinearCurve_GetPoint(const JPH_LinearCurve *curve, uint32_t index, JPH_Point *result)
{
    JPH_ASSERT(curve);
    JPH_ASSERT(result);
    FromJolt(AsLinearCurve(curve)->mPoints.at(index), result);
}

void JPH_LinearCurve_GetPoints(const JPH_LinearCurve *curve, JPH_Point *points, size_t *count)
{
    JPH_ASSERT(curve);
    const JPH::Array<JPH::LinearCurve::Point> &joltPoints = AsLinearCurve(curve)->mPoints;
    if (count != nullptr)
    {
        *count = joltPoints.size();
    }
    if (points != nullptr)
    {
        for (size_t i = 0; i < joltPoints.size(); ++i)
        {
            const JPH::LinearCurve::Point &point = joltPoints.at(i);
            points[i] = JPH_Point{
                point.mX,
                point.mY,
            };
        }
    }
}
