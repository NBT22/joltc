//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_ENUMS_H
#define JOLTC_ENUMS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef enum JPH_PhysicsUpdateError : uint32_t
{
    JPH_PhysicsUpdateError_None = 0,
    JPH_PhysicsUpdateError_ManifoldCacheFull = 1 << 0,
    JPH_PhysicsUpdateError_BodyPairCacheFull = 1 << 1,
    JPH_PhysicsUpdateError_ContactConstraintsFull = 1 << 2,
} JPH_PhysicsUpdateError;

typedef enum JPH_BodyType : uint32_t
{
    JPH_BodyType_Rigid = 0,
    JPH_BodyType_Soft = 1,
} JPH_BodyType;

typedef enum JPH_MotionType : uint32_t
{
    JPH_MotionType_Static = 0,
    JPH_MotionType_Kinematic = 1,
    JPH_MotionType_Dynamic = 2,
} JPH_MotionType;

typedef enum JPH_Activation : uint32_t
{
    JPH_Activation_Activate = 0,
    JPH_Activation_DontActivate = 1,
} JPH_Activation;

typedef enum JPH_ValidateResult : uint32_t
{
    JPH_ValidateResult_AcceptAllContactsForThisBodyPair = 0,
    JPH_ValidateResult_AcceptContact = 1,
    JPH_ValidateResult_RejectContact = 2,
    JPH_ValidateResult_RejectAllContactsForThisBodyPair = 3,
} JPH_ValidateResult;

typedef enum JPH_ShapeType : uint32_t
{
    JPH_ShapeType_Convex = 0,
    JPH_ShapeType_Compound = 1,
    JPH_ShapeType_Decorated = 2,
    JPH_ShapeType_Mesh = 3,
    JPH_ShapeType_HeightField = 4,
    JPH_ShapeType_SoftBody = 5,

    JPH_ShapeType_User1 = 6,
    JPH_ShapeType_User2 = 7,
    JPH_ShapeType_User3 = 8,
    JPH_ShapeType_User4 = 9,
} JPH_ShapeType;

typedef enum JPH_ShapeSubType : uint32_t
{
    JPH_ShapeSubType_Sphere = 0,
    JPH_ShapeSubType_Box = 1,
    JPH_ShapeSubType_Triangle = 2,
    JPH_ShapeSubType_Capsule = 3,
    JPH_ShapeSubType_TaperedCapsule = 4,
    JPH_ShapeSubType_Cylinder = 5,
    JPH_ShapeSubType_ConvexHull = 6,
    JPH_ShapeSubType_StaticCompound = 7,
    JPH_ShapeSubType_MutableCompound = 8,
    JPH_ShapeSubType_RotatedTranslated = 9,
    JPH_ShapeSubType_Scaled = 10,
    JPH_ShapeSubType_OffsetCenterOfMass = 11,
    JPH_ShapeSubType_Mesh = 12,
    JPH_ShapeSubType_HeightField = 13,
    JPH_ShapeSubType_SoftBody = 14,
} JPH_ShapeSubType;

typedef enum JPH_ConstraintType : uint32_t
{
    JPH_ConstraintType_Constraint = 0,
    JPH_ConstraintType_TwoBodyConstraint = 1,
} JPH_ConstraintType;

typedef enum JPH_ConstraintSubType : uint32_t
{
    JPH_ConstraintSubType_Fixed = 0,
    JPH_ConstraintSubType_Point = 1,
    JPH_ConstraintSubType_Hinge = 2,
    JPH_ConstraintSubType_Slider = 3,
    JPH_ConstraintSubType_Distance = 4,
    JPH_ConstraintSubType_Cone = 5,
    JPH_ConstraintSubType_SwingTwist = 6,
    JPH_ConstraintSubType_SixDOF = 7,
    JPH_ConstraintSubType_Path = 8,
    JPH_ConstraintSubType_Vehicle = 9,
    JPH_ConstraintSubType_RackAndPinion = 10,
    JPH_ConstraintSubType_Gear = 11,
    JPH_ConstraintSubType_Pulley = 12,

    JPH_ConstraintSubType_User1 = 13,
    JPH_ConstraintSubType_User2 = 14,
    JPH_ConstraintSubType_User3 = 15,
    JPH_ConstraintSubType_User4 = 16,
} JPH_ConstraintSubType;

typedef enum JPH_ConstraintSpace : uint32_t
{
    JPH_ConstraintSpace_LocalToBodyCOM = 0,
    JPH_ConstraintSpace_WorldSpace = 1,
} JPH_ConstraintSpace;

typedef enum JPH_MotionQuality : uint32_t
{
    JPH_MotionQuality_Discrete = 0,
    JPH_MotionQuality_LinearCast = 1,
} JPH_MotionQuality;

typedef enum JPH_OverrideMassProperties : uint32_t
{
    JPH_OverrideMassProperties_CalculateMassAndInertia = 0,
    JPH_OverrideMassProperties_CalculateInertia = 1,
    JPH_OverrideMassProperties_MassAndInertiaProvided = 2,
} JPH_OverrideMassProperties;

typedef enum JPH_AllowedDOFs : uint32_t
{
    JPH_AllowedDOFs_All = 0b111111,
    JPH_AllowedDOFs_TranslationX = 0b000001,
    JPH_AllowedDOFs_TranslationY = 0b000010,
    JPH_AllowedDOFs_TranslationZ = 0b000100,
    JPH_AllowedDOFs_RotationX = 0b001000,
    JPH_AllowedDOFs_RotationY = 0b010000,
    JPH_AllowedDOFs_RotationZ = 0b100000,
    JPH_AllowedDOFs_Plane2D = JPH_AllowedDOFs_TranslationX | JPH_AllowedDOFs_TranslationY | JPH_AllowedDOFs_RotationZ,
} JPH_AllowedDOFs;

typedef enum JPH_GroundState : uint32_t
{
    JPH_GroundState_OnGround = 0,
    JPH_GroundState_OnSteepGround = 1,
    JPH_GroundState_NotSupported = 2,
    JPH_GroundState_InAir = 3,
} JPH_GroundState;

typedef enum JPH_BackFaceMode : uint32_t
{
    JPH_BackFaceMode_IgnoreBackFaces = 0,
    JPH_BackFaceMode_CollideWithBackFaces = 1,
} JPH_BackFaceMode;

typedef enum JPH_ActiveEdgeMode : uint32_t
{
    JPH_ActiveEdgeMode_CollideOnlyWithActive = 0,
    JPH_ActiveEdgeMode_CollideWithAll = 1,
} JPH_ActiveEdgeMode;

typedef enum JPH_CollectFacesMode : uint32_t
{
    JPH_CollectFacesMode_CollectFaces = 0,
    JPH_CollectFacesMode_NoFaces = 1,
} JPH_CollectFacesMode;

typedef enum JPH_MotorState : uint32_t
{
    JPH_MotorState_Off = 0,
    JPH_MotorState_Velocity = 1,
    JPH_MotorState_Position = 2,
} JPH_MotorState;

typedef enum JPH_CollisionCollectorType : uint32_t
{
    JPH_CollisionCollectorType_AllHit = 0,
    JPH_CollisionCollectorType_AllHitSorted = 1,
    JPH_CollisionCollectorType_ClosestHit = 2,
    JPH_CollisionCollectorType_AnyHit = 3,
} JPH_CollisionCollectorType;

typedef enum JPH_SwingType : uint32_t
{
    JPH_SwingType_Cone = 0,
    JPH_SwingType_Pyramid = 1,
} JPH_SwingType;

typedef enum JPH_SixDOFConstraintAxis : uint32_t
{
    JPH_SixDOFConstraintAxis_TranslationX = 0,
    JPH_SixDOFConstraintAxis_TranslationY = 1,
    JPH_SixDOFConstraintAxis_TranslationZ = 2,

    JPH_SixDOFConstraintAxis_RotationX = 3,
    JPH_SixDOFConstraintAxis_RotationY = 4,
    JPH_SixDOFConstraintAxis_RotationZ = 5,

    _JPH_SixDOFConstraintAxis_Num = 6,
    _JPH_SixDOFConstraintAxis_NumTranslation = JPH_SixDOFConstraintAxis_TranslationZ + 1,
} JPH_SixDOFConstraintAxis;

typedef enum JPH_SpringMode : uint32_t
{
    JPH_SpringMode_FrequencyAndDamping = 0,
    JPH_SpringMode_StiffnessAndDamping = 1,
} JPH_SpringMode;

/// Defines how to color soft body constraints
typedef enum JPH_SoftBodyConstraintColor : uint32_t
{
    JPH_SoftBodyConstraintColor_ConstraintType = 0, /// Draw different types of constraints in different colors
    JPH_SoftBodyConstraintColor_ConstraintGroup =
            1, /// Draw constraints in the same group in the same color, non-parallel group will be red
    JPH_SoftBodyConstraintColor_ConstraintOrder =
            2, /// Draw constraints in the same group in the same color, non-parallel group will be red, and order within each group will be indicated with gradient
} JPH_SoftBodyConstraintColor;

typedef enum JPH_BodyManager_ShapeColor : uint32_t
{
    JPH_BodyManager_ShapeColor_InstanceColor = 0, ///< Random color per instance
    JPH_BodyManager_ShapeColor_ShapeTypeColor = 1, ///< Convex = green, scaled = yellow, compound = orange, mesh = red
    JPH_BodyManager_ShapeColor_MotionTypeColor =
            2, ///< Static = grey, keyframed = green, dynamic = random color per instance
    JPH_BodyManager_ShapeColor_SleepColor = 3, ///< Static = grey, keyframed = green, dynamic = yellow, sleeping = red
    JPH_BodyManager_ShapeColor_IslandColor =
            4, ///< Static = grey, active = random color per island, sleeping = light grey
    JPH_BodyManager_ShapeColor_MaterialColor = 5, ///< Color as defined by the PhysicsMaterial of the shape
} JPH_BodyManager_ShapeColor;

typedef enum JPH_DebugRenderer_CastShadow : uint32_t
{
    JPH_DebugRenderer_CastShadow_On = 0, ///< This shape should cast a shadow
    JPH_DebugRenderer_CastShadow_Off = 1, ///< This shape should not cast a shadow
} JPH_DebugRenderer_CastShadow;

typedef enum JPH_DebugRenderer_DrawMode : uint32_t
{
    JPH_DebugRenderer_DrawMode_Solid = 0, ///< Draw as a solid shape
    JPH_DebugRenderer_DrawMode_Wireframe = 1, ///< Draw as wireframe
} JPH_DebugRenderer_DrawMode;

typedef enum JPH_Mesh_Shape_BuildQuality : uint32_t
{
    JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance = 0,
    JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed = 1,
} JPH_Mesh_Shape_BuildQuality;

typedef enum JPH_TransmissionMode : uint32_t
{
    JPH_TransmissionMode_Auto = 0,
    JPH_TransmissionMode_Manual = 1,
} JPH_TransmissionMode;

typedef enum JPH_TrackSide : uint32_t
{
    JPH_TrackSide_Left = 0,
    JPH_TrackSide_Right = 1,
} JPH_TrackSide;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_ENUMS_H
