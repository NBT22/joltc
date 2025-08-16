// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include <joltc/constants.h>
#include <joltc/joltc.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Character/CharacterBase.h>
#include <Jolt/Physics/Character/CharacterID.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRendererSimple.h>
#endif // JPH_DEBUG_RENDERER

#define ENSURE_SIZE_ALIGN(type0, type1) \
    static_assert(sizeof(type0) == sizeof(type1)); \
    static_assert(alignof(type0) == alignof(type1))

// Ensure that we use 32-bit object layers
static_assert(sizeof(JPH::ObjectLayer) == 4);

static_assert(sizeof(JPH::ObjectLayer) == sizeof(JPH_ObjectLayer));
static_assert(sizeof(JPH::BroadPhaseLayer) == sizeof(JPH_BroadPhaseLayer));
static_assert(sizeof(JPH::BodyID) == sizeof(JPH_BodyId));
static_assert(sizeof(JPH::SubShapeID) == sizeof(JPH_SubShapeId));
static_assert(sizeof(JPH::CharacterID) == sizeof(JPH_CharacterId));
static_assert(sizeof(JPH::CollisionGroup::GroupID) == sizeof(JPH_CollisionGroupId));
static_assert(sizeof(JPH::CollisionGroup::SubGroupID) == sizeof(JPH_CollisionSubGroupId));

static_assert(JPH_DefaultCollisionTolerance == JPH::cDefaultCollisionTolerance);
static_assert(JPH_DefaultPenetrationTolerance == JPH::cDefaultPenetrationTolerance);
static_assert(JPH_DefaultConvexRadius == JPH::cDefaultConvexRadius);
static_assert(JPH_CapsuleProjectionSlop == JPH::cCapsuleProjectionSlop);
static_assert(JPH_MaxPhysicsJobs == JPH::cMaxPhysicsJobs);
static_assert(JPH_MaxPhysicsBarriers == JPH::cMaxPhysicsBarriers);
static_assert(JPH_CollisionGroup_InvalidGroup == JPH::CollisionGroup::cInvalidGroup);
static_assert(JPH_CollisionGroup_InvalidSubGroup == JPH::CollisionGroup::cInvalidSubGroup);

static_assert(JPH_BodyId_InvalidBodyID == JPH::BodyID::cInvalidBodyID);
static_assert(JPH_ObjectLayerInvalid == JPH::cObjectLayerInvalid);
static_assert(JPH::BroadPhaseLayer(JPH_BroadPhaseLayerInvalid) == JPH::cBroadPhaseLayerInvalid);


// EPhysicsUpdateError
static_assert(sizeof(JPH_PhysicsUpdateError) == sizeof(JPH::EPhysicsUpdateError));
static_assert(JPH_PhysicsUpdateError_None == static_cast<int>(JPH::EPhysicsUpdateError::None));
static_assert(JPH_PhysicsUpdateError_ManifoldCacheFull ==
              static_cast<int>(JPH::EPhysicsUpdateError::ManifoldCacheFull));
static_assert(JPH_PhysicsUpdateError_BodyPairCacheFull ==
              static_cast<int>(JPH::EPhysicsUpdateError::BodyPairCacheFull));
static_assert(JPH_PhysicsUpdateError_ContactConstraintsFull ==
              static_cast<int>(JPH::EPhysicsUpdateError::ContactConstraintsFull));

// EBodyType
static_assert(JPH_BodyType_Rigid == static_cast<int>(JPH::EBodyType::RigidBody));
static_assert(JPH_BodyType_Soft == static_cast<int>(JPH::EBodyType::SoftBody));

// EMotionType
static_assert(JPH_MotionType_Static == static_cast<int>(JPH::EMotionType::Static));
static_assert(JPH_MotionType_Kinematic == static_cast<int>(JPH::EMotionType::Kinematic));
static_assert(JPH_MotionType_Dynamic == static_cast<int>(JPH::EMotionType::Dynamic));

// EActivation
static_assert(sizeof(JPH::EActivation) == sizeof(JPH_Activation));
static_assert(JPH_Activation_Activate == static_cast<int>(JPH::EActivation::Activate));
static_assert(JPH_Activation_DontActivate == static_cast<int>(JPH::EActivation::DontActivate));

// EActivation
static_assert(sizeof(JPH::ValidateResult) == sizeof(JPH_ValidateResult));
static_assert(JPH_ValidateResult_AcceptAllContactsForThisBodyPair ==
              static_cast<int>(JPH::ValidateResult::AcceptAllContactsForThisBodyPair));
static_assert(JPH_ValidateResult_AcceptContact == static_cast<int>(JPH::ValidateResult::AcceptContact));
static_assert(JPH_ValidateResult_RejectContact == static_cast<int>(JPH::ValidateResult::RejectContact));
static_assert(JPH_ValidateResult_RejectAllContactsForThisBodyPair ==
              static_cast<int>(JPH::ValidateResult::RejectAllContactsForThisBodyPair));

// EShapeType
static_assert(JPH_ShapeType_Convex == static_cast<int>(JPH::EShapeType::Convex));
static_assert(JPH_ShapeType_Compound == static_cast<int>(JPH::EShapeType::Compound));
static_assert(JPH_ShapeType_Decorated == static_cast<int>(JPH::EShapeType::Decorated));
static_assert(JPH_ShapeType_Mesh == static_cast<int>(JPH::EShapeType::Mesh));
static_assert(JPH_ShapeType_HeightField == static_cast<int>(JPH::EShapeType::HeightField));
static_assert(JPH_ShapeType_SoftBody == static_cast<int>(JPH::EShapeType::SoftBody));
static_assert(JPH_ShapeType_User1 == static_cast<int>(JPH::EShapeType::User1));
static_assert(JPH_ShapeType_User2 == static_cast<int>(JPH::EShapeType::User2));
static_assert(JPH_ShapeType_User3 == static_cast<int>(JPH::EShapeType::User3));
static_assert(JPH_ShapeType_User4 == static_cast<int>(JPH::EShapeType::User4));

// EShapeSubType
static_assert(JPH_ShapeSubType_Sphere == static_cast<int>(JPH::EShapeSubType::Sphere));
static_assert(JPH_ShapeSubType_Box == static_cast<int>(JPH::EShapeSubType::Box));
static_assert(JPH_ShapeSubType_Triangle == static_cast<int>(JPH::EShapeSubType::Triangle));
static_assert(JPH_ShapeSubType_Capsule == static_cast<int>(JPH::EShapeSubType::Capsule));
static_assert(JPH_ShapeSubType_TaperedCapsule == static_cast<int>(JPH::EShapeSubType::TaperedCapsule));
static_assert(JPH_ShapeSubType_Cylinder == static_cast<int>(JPH::EShapeSubType::Cylinder));
static_assert(JPH_ShapeSubType_ConvexHull == static_cast<int>(JPH::EShapeSubType::ConvexHull));
static_assert(JPH_ShapeSubType_StaticCompound == static_cast<int>(JPH::EShapeSubType::StaticCompound));
static_assert(JPH_ShapeSubType_MutableCompound == static_cast<int>(JPH::EShapeSubType::MutableCompound));
static_assert(JPH_ShapeSubType_RotatedTranslated == static_cast<int>(JPH::EShapeSubType::RotatedTranslated));
static_assert(JPH_ShapeSubType_Scaled == static_cast<int>(JPH::EShapeSubType::Scaled));
static_assert(JPH_ShapeSubType_OffsetCenterOfMass == static_cast<int>(JPH::EShapeSubType::OffsetCenterOfMass));
static_assert(JPH_ShapeSubType_Mesh == static_cast<int>(JPH::EShapeSubType::Mesh));
static_assert(JPH_ShapeSubType_HeightField == static_cast<int>(JPH::EShapeSubType::HeightField));
static_assert(JPH_ShapeSubType_SoftBody == static_cast<int>(JPH::EShapeSubType::SoftBody));

// EConstraintType
static_assert(JPH_ConstraintType_Constraint == static_cast<int>(JPH::EConstraintType::Constraint));
static_assert(JPH_ConstraintType_TwoBodyConstraint == static_cast<int>(JPH::EConstraintType::TwoBodyConstraint));

// EConstraintSubType
static_assert(JPH_ConstraintSubType_Fixed == static_cast<int>(JPH::EConstraintSubType::Fixed));
static_assert(JPH_ConstraintSubType_Point == static_cast<int>(JPH::EConstraintSubType::Point));
static_assert(JPH_ConstraintSubType_Hinge == static_cast<int>(JPH::EConstraintSubType::Hinge));
static_assert(JPH_ConstraintSubType_Slider == static_cast<int>(JPH::EConstraintSubType::Slider));
static_assert(JPH_ConstraintSubType_Distance == static_cast<int>(JPH::EConstraintSubType::Distance));
static_assert(JPH_ConstraintSubType_Cone == static_cast<int>(JPH::EConstraintSubType::Cone));
static_assert(JPH_ConstraintSubType_SwingTwist == static_cast<int>(JPH::EConstraintSubType::SwingTwist));
static_assert(JPH_ConstraintSubType_SixDOF == static_cast<int>(JPH::EConstraintSubType::SixDOF));
static_assert(JPH_ConstraintSubType_Path == static_cast<int>(JPH::EConstraintSubType::Path));
static_assert(JPH_ConstraintSubType_Vehicle == static_cast<int>(JPH::EConstraintSubType::Vehicle));
static_assert(JPH_ConstraintSubType_RackAndPinion == static_cast<int>(JPH::EConstraintSubType::RackAndPinion));
static_assert(JPH_ConstraintSubType_Gear == static_cast<int>(JPH::EConstraintSubType::Gear));
static_assert(JPH_ConstraintSubType_Pulley == static_cast<int>(JPH::EConstraintSubType::Pulley));

static_assert(JPH_ConstraintSubType_User1 == static_cast<int>(JPH::EConstraintSubType::User1));
static_assert(JPH_ConstraintSubType_User2 == static_cast<int>(JPH::EConstraintSubType::User2));
static_assert(JPH_ConstraintSubType_User3 == static_cast<int>(JPH::EConstraintSubType::User3));
static_assert(JPH_ConstraintSubType_User4 == static_cast<int>(JPH::EConstraintSubType::User4));

// EActivation
static_assert(sizeof(JPH::EConstraintSpace) == sizeof(JPH_ConstraintSpace));
static_assert(JPH_ConstraintSpace_LocalToBodyCOM == static_cast<int>(JPH::EConstraintSpace::LocalToBodyCOM));
static_assert(JPH_ConstraintSpace_WorldSpace == static_cast<int>(JPH::EConstraintSpace::WorldSpace));

// EMotionQuality
static_assert(JPH_MotionQuality_Discrete == static_cast<int>(JPH::EMotionQuality::Discrete));
static_assert(JPH_MotionQuality_LinearCast == static_cast<int>(JPH::EMotionQuality::LinearCast));

// EOverrideMassProperties
static_assert(sizeof(JPH_OverrideMassProperties) == sizeof(uint32_t));
static_assert(sizeof(JPH::EOverrideMassProperties) == sizeof(uint8_t));
static_assert(JPH_OverrideMassProperties_CalculateMassAndInertia ==
              static_cast<int>(JPH::EOverrideMassProperties::CalculateMassAndInertia));
static_assert(JPH_OverrideMassProperties_CalculateInertia ==
              static_cast<int>(JPH::EOverrideMassProperties::CalculateInertia));
static_assert(JPH_OverrideMassProperties_MassAndInertiaProvided ==
              static_cast<int>(JPH::EOverrideMassProperties::MassAndInertiaProvided));

// EAllowedDOFs
static_assert(sizeof(JPH_AllowedDOFs) == sizeof(uint32_t));
static_assert(JPH_AllowedDOFs_All == static_cast<int>(JPH::EAllowedDOFs::All));
static_assert(JPH_AllowedDOFs_TranslationX == static_cast<int>(JPH::EAllowedDOFs::TranslationX));
static_assert(JPH_AllowedDOFs_TranslationY == static_cast<int>(JPH::EAllowedDOFs::TranslationY));
static_assert(JPH_AllowedDOFs_TranslationZ == static_cast<int>(JPH::EAllowedDOFs::TranslationZ));
static_assert(JPH_AllowedDOFs_RotationX == static_cast<int>(JPH::EAllowedDOFs::RotationX));
static_assert(JPH_AllowedDOFs_RotationY == static_cast<int>(JPH::EAllowedDOFs::RotationY));
static_assert(JPH_AllowedDOFs_RotationZ == static_cast<int>(JPH::EAllowedDOFs::RotationZ));
static_assert(JPH_AllowedDOFs_Plane2D == static_cast<int>(JPH::EAllowedDOFs::Plane2D));

// JPH_MotorState
static_assert(sizeof(JPH_MotorState) == sizeof(uint32_t));
static_assert(JPH_MotorState_Off == static_cast<int>(JPH::EMotorState::Off));
static_assert(JPH_MotorState_Velocity == static_cast<int>(JPH::EMotorState::Velocity));
static_assert(JPH_MotorState_Position == static_cast<int>(JPH::EMotorState::Position));

// JPH_SwingType
static_assert(sizeof(JPH_SwingType) == sizeof(uint32_t));
static_assert(JPH_SwingType_Cone == static_cast<int>(JPH::ESwingType::Cone));
static_assert(JPH_SwingType_Pyramid == static_cast<int>(JPH::ESwingType::Pyramid));

// JPH_SixDOFConstraintAxis
static_assert(sizeof(JPH_SixDOFConstraintAxis) == sizeof(uint32_t));
static_assert(JPH_SixDOFConstraintAxis_TranslationX ==
              static_cast<int>(JPH::SixDOFConstraintSettings::EAxis::TranslationX));
static_assert(JPH_SixDOFConstraintAxis_TranslationY ==
              static_cast<int>(JPH::SixDOFConstraintSettings::EAxis::TranslationY));
static_assert(JPH_SixDOFConstraintAxis_TranslationZ ==
              static_cast<int>(JPH::SixDOFConstraintSettings::EAxis::TranslationZ));
static_assert(JPH_SixDOFConstraintAxis_RotationX == static_cast<int>(JPH::SixDOFConstraintSettings::EAxis::RotationX));
static_assert(JPH_SixDOFConstraintAxis_RotationY == static_cast<int>(JPH::SixDOFConstraintSettings::EAxis::RotationY));
static_assert(JPH_SixDOFConstraintAxis_RotationZ == static_cast<int>(JPH::SixDOFConstraintSettings::EAxis::RotationZ));

// JPH_SpringMode
static_assert(sizeof(JPH_SpringMode) == sizeof(uint32_t));
static_assert(JPH_SpringMode_FrequencyAndDamping == static_cast<int>(JPH::ESpringMode::FrequencyAndDamping));
static_assert(JPH_SpringMode_StiffnessAndDamping == static_cast<int>(JPH::ESpringMode::StiffnessAndDamping));

// EGroundState
static_assert(sizeof(JPH::CharacterBase::EGroundState) == sizeof(JPH_GroundState));
static_assert(JPH_GroundState_OnGround == static_cast<int>(JPH::CharacterBase::EGroundState::OnGround));
static_assert(JPH_GroundState_OnSteepGround == static_cast<int>(JPH::CharacterBase::EGroundState::OnSteepGround));
static_assert(JPH_GroundState_NotSupported == static_cast<int>(JPH::CharacterBase::EGroundState::NotSupported));
static_assert(JPH_GroundState_InAir == static_cast<int>(JPH::CharacterBase::EGroundState::InAir));

// EBackFaceMode
static_assert(JPH_BackFaceMode_IgnoreBackFaces == static_cast<int>(JPH::EBackFaceMode::IgnoreBackFaces));
static_assert(JPH_BackFaceMode_CollideWithBackFaces == static_cast<int>(JPH::EBackFaceMode::CollideWithBackFaces));

// EActiveEdgeMode
static_assert(JPH_ActiveEdgeMode_CollideOnlyWithActive ==
              static_cast<int>(JPH::EActiveEdgeMode::CollideOnlyWithActive));
static_assert(JPH_ActiveEdgeMode_CollideWithAll == static_cast<int>(JPH::EActiveEdgeMode::CollideWithAll));

// ECollectFacesMode
static_assert(JPH_CollectFacesMode_CollectFaces == static_cast<int>(JPH::ECollectFacesMode::CollectFaces));
static_assert(JPH_CollectFacesMode_NoFaces == static_cast<int>(JPH::ECollectFacesMode::NoFaces));

static_assert(sizeof(JPH::SubShapeIDPair) == sizeof(JPH_SubShapeIDPair));
static_assert(alignof(JPH::SubShapeIDPair) == alignof(JPH_SubShapeIDPair));

#ifdef JPH_DEBUG_RENDERER

// ESoftBodyConstraintColor
static_assert(JPH_SoftBodyConstraintColor_ConstraintType ==
              static_cast<int>(JPH::ESoftBodyConstraintColor::ConstraintType));
static_assert(JPH_SoftBodyConstraintColor_ConstraintGroup ==
              static_cast<int>(JPH::ESoftBodyConstraintColor::ConstraintGroup));
static_assert(JPH_SoftBodyConstraintColor_ConstraintOrder ==
              static_cast<int>(JPH::ESoftBodyConstraintColor::ConstraintOrder));

// BodyManager::EShapeColor
static_assert(JPH_BodyManager_ShapeColor_InstanceColor ==
              static_cast<int>(JPH::BodyManager::EShapeColor::InstanceColor));
static_assert(JPH_BodyManager_ShapeColor_ShapeTypeColor ==
              static_cast<int>(JPH::BodyManager::EShapeColor::ShapeTypeColor));
static_assert(JPH_BodyManager_ShapeColor_MotionTypeColor ==
              static_cast<int>(JPH::BodyManager::EShapeColor::MotionTypeColor));
static_assert(JPH_BodyManager_ShapeColor_SleepColor == static_cast<int>(JPH::BodyManager::EShapeColor::SleepColor));
static_assert(JPH_BodyManager_ShapeColor_IslandColor == static_cast<int>(JPH::BodyManager::EShapeColor::IslandColor));
static_assert(JPH_BodyManager_ShapeColor_MaterialColor ==
              static_cast<int>(JPH::BodyManager::EShapeColor::MaterialColor));

// DebugRenderer::ECastShadow
static_assert(JPH_DebugRenderer_CastShadow_On == static_cast<int>(JPH::DebugRenderer::ECastShadow::On));
static_assert(JPH_DebugRenderer_CastShadow_Off == static_cast<int>(JPH::DebugRenderer::ECastShadow::Off));

// DebugRenderer::EDrawMode
static_assert(JPH_DebugRenderer_DrawMode_Solid == static_cast<int>(JPH::DebugRenderer::EDrawMode::Solid));
static_assert(JPH_DebugRenderer_DrawMode_Wireframe == static_cast<int>(JPH::DebugRenderer::EDrawMode::Wireframe));

// MeshShapeSettings::EBuildQuality
static_assert(JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance ==
              static_cast<int>(JPH::MeshShapeSettings::EBuildQuality::FavorRuntimePerformance));
static_assert(JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed ==
              static_cast<int>(JPH::MeshShapeSettings::EBuildQuality::FavorBuildSpeed));

// MeshShapeSettings::EBuildQuality
static_assert(JPH_TransmissionMode_Auto == static_cast<int>(JPH::ETransmissionMode::Auto));
static_assert(JPH_TransmissionMode_Manual == static_cast<int>(JPH::ETransmissionMode::Manual));


#endif
