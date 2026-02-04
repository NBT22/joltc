// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

// ReSharper disable CppVariableCanBeMadeConstexpr
#ifndef JOLTC_H
#define JOLTC_H

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
#include <joltc/Physics/Body/BodyFilter.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Body/BodyInterface.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/NarrowPhaseQuery.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <joltc/Physics/Collision/Shape/SubShapeIDPair.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>
#include <joltc/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Forward declarations */
typedef struct JPH_BroadPhaseLayerInterface JPH_BroadPhaseLayerInterface;
typedef struct JPH_ObjectVsBroadPhaseLayerFilter JPH_ObjectVsBroadPhaseLayerFilter;
typedef struct JPH_ObjectLayerPairFilter JPH_ObjectLayerPairFilter;

typedef struct JPH_SimShapeFilter JPH_SimShapeFilter;

typedef struct JPH_PhysicsStepListener JPH_PhysicsStepListener;
typedef struct JPH_PhysicsSystem JPH_PhysicsSystem;
typedef struct JPH_LinearCurve JPH_LinearCurve;

/* ShapeSettings */
typedef struct JPH_ConvexShapeSettings JPH_ConvexShapeSettings;
typedef struct JPH_SphereShapeSettings JPH_SphereShapeSettings;
typedef struct JPH_BoxShapeSettings JPH_BoxShapeSettings;
typedef struct JPH_PlaneShapeSettings JPH_PlaneShapeSettings;
typedef struct JPH_TriangleShapeSettings JPH_TriangleShapeSettings;
typedef struct JPH_CapsuleShapeSettings JPH_CapsuleShapeSettings;
typedef struct JPH_TaperedCapsuleShapeSettings JPH_TaperedCapsuleShapeSettings;
typedef struct JPH_CylinderShapeSettings JPH_CylinderShapeSettings;
typedef struct JPH_TaperedCylinderShapeSettings JPH_TaperedCylinderShapeSettings;
typedef struct JPH_ConvexHullShapeSettings JPH_ConvexHullShapeSettings;
typedef struct JPH_CompoundShapeSettings JPH_CompoundShapeSettings;
typedef struct JPH_StaticCompoundShapeSettings JPH_StaticCompoundShapeSettings;
typedef struct JPH_MutableCompoundShapeSettings JPH_MutableCompoundShapeSettings;
typedef struct JPH_MeshShapeSettings JPH_MeshShapeSettings;
typedef struct JPH_HeightFieldShapeSettings JPH_HeightFieldShapeSettings;
typedef struct JPH_RotatedTranslatedShapeSettings JPH_RotatedTranslatedShapeSettings;
typedef struct JPH_ScaledShapeSettings JPH_ScaledShapeSettings;
typedef struct JPH_OffsetCenterOfMassShapeSettings JPH_OffsetCenterOfMassShapeSettings;
typedef struct JPH_EmptyShapeSettings JPH_EmptyShapeSettings;

/* Shape */
typedef struct JPH_ConvexShape JPH_ConvexShape;
typedef struct JPH_SphereShape JPH_SphereShape;
typedef struct JPH_BoxShape JPH_BoxShape;
typedef struct JPH_PlaneShape JPH_PlaneShape;
typedef struct JPH_CapsuleShape JPH_CapsuleShape;
typedef struct JPH_CylinderShape JPH_CylinderShape;
typedef struct JPH_TaperedCylinderShape JPH_TaperedCylinderShape;
typedef struct JPH_TriangleShape JPH_TriangleShape;
typedef struct JPH_TaperedCapsuleShape JPH_TaperedCapsuleShape;
typedef struct JPH_ConvexHullShape JPH_ConvexHullShape;
typedef struct JPH_CompoundShape JPH_CompoundShape;
typedef struct JPH_StaticCompoundShape JPH_StaticCompoundShape;
typedef struct JPH_MutableCompoundShape JPH_MutableCompoundShape;
typedef struct JPH_MeshShape JPH_MeshShape;
typedef struct JPH_HeightFieldShape JPH_HeightFieldShape;
typedef struct JPH_DecoratedShape JPH_DecoratedShape;
typedef struct JPH_RotatedTranslatedShape JPH_RotatedTranslatedShape;
typedef struct JPH_ScaledShape JPH_ScaledShape;
typedef struct JPH_OffsetCenterOfMassShape JPH_OffsetCenterOfMassShape;
typedef struct JPH_EmptyShape JPH_EmptyShape;

typedef struct JPH_BodyLockInterface JPH_BodyLockInterface;

typedef struct JPH_ContactListener JPH_ContactListener;
typedef struct JPH_ContactManifold JPH_ContactManifold;

typedef struct JPH_GroupFilterTable JPH_GroupFilterTable; /* Inherits JPH_GroupFilter */

typedef struct JPH_Plane
{
        Vector3 normal;
        float distance;
} JPH_Plane;

typedef struct JPH_Point
{
        float x;
        float y;
} JPH_Point;

typedef struct JPH_Triangle
{
        Vector3 v1;
        Vector3 v2;
        Vector3 v3;
        uint32_t materialIndex;
} JPH_Triangle;

typedef struct JPH_IndexedTriangleNoMaterial
{
        uint32_t i1;
        uint32_t i2;
        uint32_t i3;
} JPH_IndexedTriangleNoMaterial;

typedef struct JPH_IndexedTriangle
{
        uint32_t i1;
        uint32_t i2;
        uint32_t i3;
        uint32_t materialIndex;
        uint32_t userData;
} JPH_IndexedTriangle;

typedef struct JPH_ContactSettings
{
        float combinedFriction;
        float combinedRestitution;
        float invMassScale1;
        float invInertiaScale1;
        float invMassScale2;
        float invInertiaScale2;
        bool isSensor;
        Vector3 relativeLinearSurfaceVelocity;
        Vector3 relativeAngularSurfaceVelocity;
} JPH_ContactSettings;

typedef struct JPH_SpringSettings
{
        JPH_SpringMode mode;
        float frequencyOrStiffness;
        float damping;
} JPH_SpringSettings;

typedef struct JPH_MotorSettings
{
        JPH_SpringSettings springSettings;
        float minForceLimit;
        float maxForceLimit;
        float minTorqueLimit;
        float maxTorqueLimit;
} JPH_MotorSettings;

typedef struct JPH_DrawSettings
{
        bool drawGetSupportFunction; ///< Draw the GetSupport() function, used for convex collision detection
        bool drawSupportDirection; ///< When drawing the support function, also draw which direction mapped to a specific support point
        bool drawGetSupportingFace; ///< Draw the faces that were found colliding during collision detection
        bool drawShape; ///< Draw the shapes of all bodies
        bool drawShapeWireframe; ///< When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
        JPH_BodyManager_ShapeColor drawShapeColor; ///< Coloring scheme to use for shapes
        bool drawBoundingBox; ///< Draw a bounding box per body
        bool drawCenterOfMassTransform; ///< Draw the center of mass for each body
        bool drawWorldTransform; ///< Draw the world transform (which may differ from its center of mass) of each body
        bool drawVelocity; ///< Draw the velocity vector for each body
        bool drawMassAndInertia; ///< Draw the mass and inertia (as the box equivalent) for each body
        bool drawSleepStats; ///< Draw stats regarding the sleeping algorithm of each body
        bool drawSoftBodyVertices; ///< Draw the vertices of soft bodies
        bool drawSoftBodyVertexVelocities; ///< Draw the velocities of the vertices of soft bodies
        bool drawSoftBodyEdgeConstraints; ///< Draw the edge constraints of soft bodies
        bool drawSoftBodyBendConstraints; ///< Draw the bend constraints of soft bodies
        bool drawSoftBodyVolumeConstraints; ///< Draw the volume constraints of soft bodies
        bool drawSoftBodySkinConstraints; ///< Draw the skin constraints of soft bodies
        bool drawSoftBodyLRAConstraints; ///< Draw the LRA constraints of soft bodies
        bool drawSoftBodyPredictedBounds; ///< Draw the predicted bounds of soft bodies
        JPH_SoftBodyConstraintColor drawSoftBodyConstraintColor; ///< Coloring scheme to use for soft body constraints
} JPH_DrawSettings;

typedef float JPH_CollideShapeCollectorCallback(void *context, const JPH_CollideShapeResult *result);

typedef struct JPH_CollisionEstimationResultImpulse
{
        float contactImpulse;
        float frictionImpulse1;
        float frictionImpulse2;
} JPH_CollisionEstimationResultImpulse;

typedef struct JPH_CollisionEstimationResult
{
        Vector3 linearVelocity1;
        Vector3 angularVelocity1;
        Vector3 linearVelocity2;
        Vector3 angularVelocity2;

        Vector3 tangent1;
        Vector3 tangent2;

        uint32_t impulseCount;
        JPH_CollisionEstimationResultImpulse *impulses;
} JPH_CollisionEstimationResult;

typedef struct JPH_BodyActivationListener JPH_BodyActivationListener;
typedef struct JPH_BodyDrawFilter JPH_BodyDrawFilter;

typedef struct JPH_SharedMutex JPH_SharedMutex;

typedef struct JPH_DebugRenderer JPH_DebugRenderer;

/* Constraint */
typedef struct JPH_Constraint JPH_Constraint;
typedef struct JPH_TwoBodyConstraint JPH_TwoBodyConstraint;
typedef struct JPH_FixedConstraint JPH_FixedConstraint;
typedef struct JPH_DistanceConstraint JPH_DistanceConstraint;
typedef struct JPH_PointConstraint JPH_PointConstraint;
typedef struct JPH_HingeConstraint JPH_HingeConstraint;
typedef struct JPH_SliderConstraint JPH_SliderConstraint;
typedef struct JPH_ConeConstraint JPH_ConeConstraint;
typedef struct JPH_SwingTwistConstraint JPH_SwingTwistConstraint;
typedef struct JPH_SixDOFConstraint JPH_SixDOFConstraint;
typedef struct JPH_GearConstraint JPH_GearConstraint;

/* Character, CharacterVirtual */
typedef struct JPH_CharacterBase JPH_CharacterBase;
typedef struct JPH_Character JPH_Character; /* Inherits JPH_CharacterBase */
typedef struct JPH_CharacterVirtual JPH_CharacterVirtual; /* Inherits JPH_CharacterBase */
typedef struct JPH_CharacterContactListener JPH_CharacterContactListener;
typedef struct JPH_CharacterVsCharacterCollision JPH_CharacterVsCharacterCollision;

/* Skeleton/Ragdoll */
typedef struct JPH_Skeleton JPH_Skeleton;
typedef struct JPH_SkeletonPose JPH_SkeletonPose;
typedef struct JPH_SkeletalAnimation JPH_SkeletalAnimation;
typedef struct JPH_SkeletonMapper JPH_SkeletonMapper;
typedef struct JPH_RagdollSettings JPH_RagdollSettings;
typedef struct JPH_Ragdoll JPH_Ragdoll;

typedef struct JPH_ConstraintSettings
{
        bool enabled;
        uint32_t constraintPriority;
        uint32_t numVelocityStepsOverride;
        uint32_t numPositionStepsOverride;
        float drawConstraintSize;
        uint64_t userData;
} JPH_ConstraintSettings;

typedef struct JPH_FixedConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        bool autoDetectPoint;
        JPH_RVec3 point1;
        Vector3 axisX1;
        Vector3 axisY1;
        JPH_RVec3 point2;
        Vector3 axisX2;
        Vector3 axisY2;
} JPH_FixedConstraintSettings;

typedef struct JPH_DistanceConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        JPH_RVec3 point1;
        JPH_RVec3 point2;
        float minDistance;
        float maxDistance;
        JPH_SpringSettings limitsSpringSettings;
} JPH_DistanceConstraintSettings;

typedef struct JPH_PointConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        JPH_RVec3 point1;
        JPH_RVec3 point2;
} JPH_PointConstraintSettings;

typedef struct JPH_HingeConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        JPH_RVec3 point1;
        Vector3 hingeAxis1;
        Vector3 normalAxis1;
        JPH_RVec3 point2;
        Vector3 hingeAxis2;
        Vector3 normalAxis2;
        float limitsMin;
        float limitsMax;
        JPH_SpringSettings limitsSpringSettings;
        float maxFrictionTorque;
        JPH_MotorSettings motorSettings;
} JPH_HingeConstraintSettings;

typedef struct JPH_SliderConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        bool autoDetectPoint;
        JPH_RVec3 point1;
        Vector3 sliderAxis1;
        Vector3 normalAxis1;
        JPH_RVec3 point2;
        Vector3 sliderAxis2;
        Vector3 normalAxis2;
        float limitsMin;
        float limitsMax;
        JPH_SpringSettings limitsSpringSettings;
        float maxFrictionForce;
        JPH_MotorSettings motorSettings;
} JPH_SliderConstraintSettings;

typedef struct JPH_ConeConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        JPH_RVec3 point1;
        Vector3 twistAxis1;
        JPH_RVec3 point2;
        Vector3 twistAxis2;
        float halfConeAngle;
} JPH_ConeConstraintSettings;

typedef struct JPH_SwingTwistConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        JPH_RVec3 position1;
        Vector3 twistAxis1;
        Vector3 planeAxis1;
        JPH_RVec3 position2;
        Vector3 twistAxis2;
        Vector3 planeAxis2;
        JPH_SwingType swingType;
        float normalHalfConeAngle;
        float planeHalfConeAngle;
        float twistMinAngle;
        float twistMaxAngle;
        float maxFrictionTorque;
        JPH_MotorSettings swingMotorSettings;
        JPH_MotorSettings twistMotorSettings;
} JPH_SwingTwistConstraintSettings;

typedef struct JPH_SixDOFConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        JPH_RVec3 position1;
        Vector3 axisX1;
        Vector3 axisY1;
        JPH_RVec3 position2;
        Vector3 axisX2;
        Vector3 axisY2;
        float maxFriction[_JPH_SixDOFConstraintAxis_Num];
        JPH_SwingType swingType;
        float limitMin[_JPH_SixDOFConstraintAxis_Num];
        float limitMax[_JPH_SixDOFConstraintAxis_Num];

        JPH_SpringSettings limitsSpringSettings[_JPH_SixDOFConstraintAxis_NumTranslation];
        JPH_MotorSettings motorSettings[_JPH_SixDOFConstraintAxis_Num];
} JPH_SixDOFConstraintSettings;

typedef struct JPH_GearConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        JPH_ConstraintSpace space;
        Vector3 hingeAxis1;
        Vector3 hingeAxis2;
        float ratio;
} JPH_GearConstraintSettings;

typedef struct JPH_BodyLockRead
{
        const JPH_BodyLockInterface *lockInterface;
        JPH_SharedMutex *mutex;
        const JPH_Body *body;
} JPH_BodyLockRead;

typedef struct JPH_BodyLockWrite
{
        const JPH_BodyLockInterface *lockInterface;
        JPH_SharedMutex *mutex;
        JPH_Body *body;
} JPH_BodyLockWrite;

typedef struct JPH_BodyLockMultiRead JPH_BodyLockMultiRead;
typedef struct JPH_BodyLockMultiWrite JPH_BodyLockMultiWrite;

typedef struct JPH_ExtendedUpdateSettings
{
        Vector3 stickToFloorStepDown;
        Vector3 walkStairsStepUp;
        float walkStairsMinStepForward;
        float walkStairsStepForwardTest;
        float walkStairsCosAngleForwardContact;
        Vector3 walkStairsStepDownExtra;
} JPH_ExtendedUpdateSettings;

typedef struct JPH_CharacterBaseSettings
{
        Vector3 up;
        JPH_Plane supportingVolume;
        float maxSlopeAngle;
        bool enhancedInternalEdgeRemoval;
        const JPH_Shape *shape;
} JPH_CharacterBaseSettings;

/* Character */
typedef struct JPH_CharacterSettings
{
        JPH_CharacterBaseSettings base; /* Inherits JPH_CharacterBaseSettings */
        JPH_ObjectLayer layer;
        float mass;
        float friction;
        float gravityFactor;
        JPH_AllowedDOFs allowedDOFs;
} JPH_CharacterSettings;

/* CharacterVirtual */
typedef struct JPH_CharacterVirtualSettings
{
        JPH_CharacterBaseSettings base; /* Inherits JPH_CharacterBaseSettings */
        JPH_CharacterID ID;
        float mass;
        float maxStrength;
        Vector3 shapeOffset;
        JPH_BackFaceMode backFaceMode;
        float predictiveContactDistance;
        uint32_t maxCollisionIterations;
        uint32_t maxConstraintIterations;
        float minTimeRemaining;
        float collisionTolerance;
        float characterPadding;
        uint32_t maxNumHits;
        float hitReductionCosMaxAngle;
        float penetrationRecoverySpeed;
        const JPH_Shape *innerBodyShape;
        JPH_BodyID innerBodyIDOverride;
        JPH_ObjectLayer innerBodyLayer;
} JPH_CharacterVirtualSettings;

typedef struct JPH_CharacterContactSettings
{
        bool canPushCharacter;
        bool canReceiveImpulses;
} JPH_CharacterContactSettings;

typedef struct JPH_CharacterVirtualContact
{
        uint64_t hash;
        JPH_BodyID bodyB;
        JPH_CharacterID characterIDB;
        JPH_SubShapeID subShapeIDB;
        JPH_RVec3 position;
        Vector3 linearVelocity;
        Vector3 contactNormal;
        Vector3 surfaceNormal;
        float distance;
        float fraction;
        JPH_MotionType motionTypeB;
        bool isSensorB;
        const JPH_CharacterVirtual *characterB;
        uint64_t userData;
        const JPH_PhysicsMaterial *material;
        bool hadCollision;
        bool wasDiscarded;
        bool canPushCharacter;
} JPH_CharacterVirtualContact;

typedef void(JPH_API_CALL *JPH_TraceFunc)(const char *message);
typedef bool(JPH_API_CALL *JPH_AssertFailureFunc)(const char *expression,
                                                  const char *message,
                                                  const char *file,
                                                  uint32_t line);

typedef void JPH_JobFunction(void *arg);
typedef void JPH_QueueJobCallback(void *context, JPH_JobFunction *job, void *arg);
typedef void JPH_QueueJobsCallback(void *context, JPH_JobFunction *job, void **args, uint32_t count);

typedef struct JPH_JobSystemThreadPoolConfig
{
        uint32_t maxJobs;
        uint32_t maxBarriers;
        int32_t numThreads;
} JPH_JobSystemThreadPoolConfig;

typedef struct JPH_JobSystemConfig
{
        void *context;
        JPH_QueueJobCallback *queueJob;
        JPH_QueueJobsCallback *queueJobs;
        int maxConcurrency;
        uint32_t maxBarriers;
} JPH_JobSystemConfig;

typedef struct JPH_JobSystem JPH_JobSystem;

/* Calculate max tire impulses by combining friction, slip, and suspension impulse. Note that the actual applied impulse may be lower (e.g. when the vehicle is stationary on a horizontal surface the actual impulse applied will be 0) */
typedef void(JPH_API_CALL *JPH_TireMaxImpulseCallback)(uint32_t inWheelIndex,
                                                       float *outLongitudinalImpulse,
                                                       float *outLateralImpulse,
                                                       float inSuspensionImpulse,
                                                       float inLongitudinalFriction,
                                                       float inLateralFriction,
                                                       float inLongitudinalSlip,
                                                       float inLateralSlip,
                                                       float inDeltaTime);

JPH_CAPI JPH_JobSystem *JPH_JobSystemThreadPool_Create(const JPH_JobSystemThreadPoolConfig *config);
JPH_CAPI JPH_JobSystem *JPH_JobSystemCallback_Create(const JPH_JobSystemConfig *config);
JPH_CAPI void JPH_JobSystem_Destroy(JPH_JobSystem *jobSystem);

JPH_CAPI void JPH_Init(void);
JPH_CAPI void JPH_Shutdown(void);
JPH_CAPI void JPH_SetTraceHandler(JPH_TraceFunc handler);
JPH_CAPI void JPH_SetAssertFailureHandler(JPH_AssertFailureFunc handler);

/* Structs free members */
JPH_CAPI void JPH_CollisionEstimationResult_FreeMembers(const JPH_CollisionEstimationResult *result);

/* JPH_BroadPhaseLayerInterface */
typedef struct JPH_BroadPhaseLayerInterface_Impl
{
        JPH_BroadPhaseLayer(JPH_API_CALL *GetBroadPhaseLayer)(JPH_ObjectLayer inLayer);
} JPH_BroadPhaseLayerInterface_Impl;
JPH_CAPI JPH_BroadPhaseLayerInterface *JPH_BroadPhaseLayerInterface_Create(uint32_t broadPhaseLayerCount,
                                                                           const JPH_BroadPhaseLayerInterface_Impl
                                                                                   *impl);
JPH_CAPI void JPH_BroadPhaseLayerInterface_Destroy(JPH_BroadPhaseLayerInterface *broadPhaseLayerInterface);
JPH_CAPI void JPH_BroadPhaseLayerInterface_SetImpl(JPH_BroadPhaseLayerInterface *broadPhaseLayerInterface,
                                                   const JPH_BroadPhaseLayerInterface_Impl *impl);

/* JPH_ObjectLayerPairFilter */
typedef struct JPH_ObjectLayerPairFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(JPH_ObjectLayer inLayer1, JPH_ObjectLayer inLayer2);
} JPH_ObjectLayerPairFilter_Impl;

JPH_CAPI JPH_ObjectLayerPairFilter *JPH_ObjectLayerPairFilter_Create(const JPH_ObjectLayerPairFilter_Impl *impl);
JPH_CAPI void JPH_ObjectLayerPairFilter_Destroy(JPH_ObjectLayerPairFilter *filter);
JPH_CAPI void JPH_ObjectLayerPairFilter_SetImpl(JPH_ObjectLayerPairFilter *filter,
                                                const JPH_ObjectLayerPairFilter_Impl *impl);

/* JPH_ObjectVsBroadPhaseLayerFilter */
typedef struct JPH_ObjectVsBroadPhaseLayerFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(JPH_ObjectLayer inObjectLayer, JPH_BroadPhaseLayer inBroadPhaseLayer);
} JPH_ObjectVsBroadPhaseLayerFilter_Impl;
JPH_CAPI JPH_ObjectVsBroadPhaseLayerFilter *JPH_ObjectVsBroadPhaseLayerFilter_Create(
        const JPH_ObjectVsBroadPhaseLayerFilter_Impl *impl);
JPH_CAPI void JPH_ObjectVsBroadPhaseLayerFilter_Destroy(JPH_ObjectVsBroadPhaseLayerFilter *filter);
JPH_CAPI void JPH_ObjectVsBroadPhaseLayerFilter_SetImpl(JPH_ObjectVsBroadPhaseLayerFilter *filter,
                                                        const JPH_ObjectVsBroadPhaseLayerFilter_Impl *impl);

/* JPH_PhysicsSystem */
typedef struct JPH_PhysicsSystemSettings
{
        uint32_t maxBodies; /* 10240 */
        uint32_t numBodyMutexes; /* 0 */
        uint32_t maxBodyPairs; /* 65536 */
        uint32_t maxContactConstraints; /* 10240 */
        uint32_t _padding;
        JPH_BroadPhaseLayerInterface *broadPhaseLayerInterface;
        JPH_ObjectLayerPairFilter *objectLayerPairFilter;
        JPH_ObjectVsBroadPhaseLayerFilter *objectVsBroadPhaseLayerFilter;
} JPH_PhysicsSystemSettings;

typedef struct JPH_PhysicsSettings
{
        int maxInFlightBodyPairs;
        int stepListenersBatchSize;
        int stepListenerBatchesPerJob;
        float baumgarte;
        float speculativeContactDistance;
        float penetrationSlop;
        float linearCastThreshold;
        float linearCastMaxPenetration;
        float manifoldTolerance;
        float maxPenetrationDistance;
        float bodyPairCacheMaxDeltaPositionSq;
        float bodyPairCacheCosMaxDeltaRotationDiv2;
        float contactNormalCosMaxDeltaRotation;
        float contactPointPreserveLambdaMaxDistSq;
        uint32_t numVelocitySteps;
        uint32_t numPositionSteps;
        float minVelocityForRestitution;
        float timeBeforeSleep;
        float pointVelocitySleepThreshold;
        bool deterministicSimulation;
        bool constraintWarmStart;
        bool useBodyPairContactCache;
        bool useManifoldReduction;
        bool useLargeIslandSplitter;
        bool allowSleeping;
        bool checkActiveEdges;
} JPH_PhysicsSettings;

JPH_CAPI JPH_PhysicsSystem *JPH_PhysicsSystem_Create(const JPH_PhysicsSystemSettings *settings);
JPH_CAPI void JPH_PhysicsSystem_Destroy(const JPH_PhysicsSystem *system);

JPH_CAPI void JPH_PhysicsSystem_SetPhysicsSettings(const JPH_PhysicsSystem *system,
                                                   const JPH_PhysicsSettings *settings);
JPH_CAPI void JPH_PhysicsSystem_GetPhysicsSettings(const JPH_PhysicsSystem *system, JPH_PhysicsSettings *result);

JPH_CAPI void JPH_PhysicsSystem_OptimizeBroadPhase(JPH_PhysicsSystem *system);
JPH_CAPI JPH_PhysicsUpdateError JPH_PhysicsSystem_Update(const JPH_PhysicsSystem *system,
                                                         float deltaTime,
                                                         int collisionSteps,
                                                         JPH_JobSystem *jobSystem);

JPH_CAPI JPH_BodyInterface *JPH_PhysicsSystem_GetBodyInterface(const JPH_PhysicsSystem *system);
JPH_CAPI JPH_BodyInterface *JPH_PhysicsSystem_GetBodyInterfaceNoLock(const JPH_PhysicsSystem *system);

JPH_CAPI const JPH_BodyLockInterface *JPH_PhysicsSystem_GetBodyLockInterface(const JPH_PhysicsSystem *system);
JPH_CAPI const JPH_BodyLockInterface *JPH_PhysicsSystem_GetBodyLockInterfaceNoLock(const JPH_PhysicsSystem *system);

JPH_CAPI const JPH_BroadPhaseQuery *JPH_PhysicsSystem_GetBroadPhaseQuery(const JPH_PhysicsSystem *system);

JPH_CAPI const JPH_NarrowPhaseQuery *JPH_PhysicsSystem_GetNarrowPhaseQuery(const JPH_PhysicsSystem *system);
JPH_CAPI const JPH_NarrowPhaseQuery *JPH_PhysicsSystem_GetNarrowPhaseQueryNoLock(const JPH_PhysicsSystem *system);

JPH_CAPI void JPH_PhysicsSystem_SetContactListener(JPH_PhysicsSystem *system, JPH_ContactListener *listener);
JPH_CAPI void JPH_PhysicsSystem_SetBodyActivationListener(JPH_PhysicsSystem *system,
                                                          JPH_BodyActivationListener *listener);
JPH_CAPI void JPH_PhysicsSystem_SetSimShapeFilter(JPH_PhysicsSystem *system, const JPH_SimShapeFilter *filter);

JPH_CAPI bool JPH_PhysicsSystem_WereBodiesInContact(const JPH_PhysicsSystem *system,
                                                    JPH_BodyID body1,
                                                    JPH_BodyID body2);

JPH_CAPI uint32_t JPH_PhysicsSystem_GetNumBodies(const JPH_PhysicsSystem *system);
JPH_CAPI uint32_t JPH_PhysicsSystem_GetNumActiveBodies(const JPH_PhysicsSystem *system, JPH_BodyType type);
JPH_CAPI uint32_t JPH_PhysicsSystem_GetMaxBodies(const JPH_PhysicsSystem *system);
JPH_CAPI uint32_t JPH_PhysicsSystem_GetNumConstraints(const JPH_PhysicsSystem *system);

JPH_CAPI void JPH_PhysicsSystem_SetGravity(JPH_PhysicsSystem *system, const Vector3 *value);
JPH_CAPI void JPH_PhysicsSystem_GetGravity(JPH_PhysicsSystem *system, Vector3 *result);

JPH_CAPI void JPH_PhysicsSystem_AddConstraint(JPH_PhysicsSystem *system, JPH_Constraint *constraint);
JPH_CAPI void JPH_PhysicsSystem_RemoveConstraint(JPH_PhysicsSystem *system, JPH_Constraint *constraint);

JPH_CAPI void JPH_PhysicsSystem_AddConstraints(JPH_PhysicsSystem *system, JPH_Constraint **constraints, int count);
JPH_CAPI void JPH_PhysicsSystem_RemoveConstraints(JPH_PhysicsSystem *system, JPH_Constraint **constraints, int count);

JPH_CAPI void JPH_PhysicsSystem_AddStepListener(JPH_PhysicsSystem *system, JPH_PhysicsStepListener *listener);
JPH_CAPI void JPH_PhysicsSystem_RemoveStepListener(JPH_PhysicsSystem *system, JPH_PhysicsStepListener *listener);

JPH_CAPI void JPH_PhysicsSystem_GetBodies(const JPH_PhysicsSystem *system, JPH_BodyID *ids, uint32_t count);
JPH_CAPI void JPH_PhysicsSystem_GetConstraints(const JPH_PhysicsSystem *system,
                                               const JPH_Constraint **constraints,
                                               uint32_t count);
JPH_CAPI void JPH_PhysicsSystem_ActivateBodiesInAABox(const JPH_PhysicsSystem *system,
                                                      const JPH_AABox *box,
                                                      JPH_ObjectLayer layer);

JPH_CAPI void JPH_PhysicsSystem_DrawBodies(const JPH_PhysicsSystem *system,
                                           const JPH_DrawSettings *settings,
                                           JPH_DebugRenderer *renderer,
                                           const JPH_BodyDrawFilter *bodyFilter /* = nullptr */);
JPH_CAPI void JPH_PhysicsSystem_DrawConstraints(const JPH_PhysicsSystem *system, JPH_DebugRenderer *renderer);
JPH_CAPI void JPH_PhysicsSystem_DrawConstraintLimits(const JPH_PhysicsSystem *system, JPH_DebugRenderer *renderer);
JPH_CAPI void JPH_PhysicsSystem_DrawConstraintReferenceFrame(const JPH_PhysicsSystem *system,
                                                             JPH_DebugRenderer *renderer);

/* PhysicsStepListener */
typedef struct JPH_PhysicsStepListenerContext
{
        float deltaTime;
        bool isFirstStep;
        bool isLastStep;
        JPH_PhysicsSystem *physicsSystem;
} JPH_PhysicsStepListenerContext;


typedef struct JPH_PhysicsStepListener_Impl
{
        void(JPH_API_CALL *OnStep)(void *userData, const JPH_PhysicsStepListenerContext *context);
} JPH_PhysicsStepListener_Impl;

JPH_CAPI void JPH_PhysicsStepListener_SetImpl(const JPH_PhysicsStepListener_Impl *impl);
JPH_CAPI JPH_PhysicsStepListener *JPH_PhysicsStepListener_Create(void *userData);
JPH_CAPI void JPH_PhysicsStepListener_Destroy(JPH_PhysicsStepListener *listener);

/* Math */
JPH_CAPI float JPH_Sin(float value);
JPH_CAPI float JPH_Cos(float value);

/* GroupFilter/GroupFilterTable */
JPH_CAPI JPH_GroupFilterTable *JPH_GroupFilterTable_Create(uint32_t numSubGroups /* = 0*/);
JPH_CAPI void JPH_GroupFilterTable_DisableCollision(JPH_GroupFilterTable *table,
                                                    JPH_CollisionSubGroupID subGroup1,
                                                    JPH_CollisionSubGroupID subGroup2);
JPH_CAPI void JPH_GroupFilterTable_EnableCollision(JPH_GroupFilterTable *table,
                                                   JPH_CollisionSubGroupID subGroup1,
                                                   JPH_CollisionSubGroupID subGroup2);
JPH_CAPI bool JPH_GroupFilterTable_IsCollisionEnabled(JPH_GroupFilterTable *table,
                                                      JPH_CollisionSubGroupID subGroup1,
                                                      JPH_CollisionSubGroupID subGroup2);

/* JPH_ConvexShape */
JPH_CAPI float JPH_ConvexShapeSettings_GetDensity(const JPH_ConvexShapeSettings *settings);
JPH_CAPI void JPH_ConvexShapeSettings_SetDensity(JPH_ConvexShapeSettings *settings, float value);
JPH_CAPI float JPH_ConvexShape_GetDensity(const JPH_ConvexShape *shape);
JPH_CAPI void JPH_ConvexShape_SetDensity(JPH_ConvexShape *shape, float density);

/* BoxShape */
JPH_CAPI JPH_BoxShapeSettings *JPH_BoxShapeSettings_Create(const Vector3 *halfExtent, float convexRadius);
JPH_CAPI JPH_BoxShape *JPH_BoxShapeSettings_CreateShape(const JPH_BoxShapeSettings *settings);

JPH_CAPI JPH_BoxShape *JPH_BoxShape_Create(const Vector3 *halfExtent, float convexRadius);
JPH_CAPI void JPH_BoxShape_GetHalfExtent(const JPH_BoxShape *shape, Vector3 *halfExtent);
JPH_CAPI float JPH_BoxShape_GetConvexRadius(const JPH_BoxShape *shape);

/* SphereShape */
JPH_CAPI JPH_SphereShapeSettings *JPH_SphereShapeSettings_Create(float radius);
JPH_CAPI JPH_SphereShape *JPH_SphereShapeSettings_CreateShape(const JPH_SphereShapeSettings *settings);

JPH_CAPI float JPH_SphereShapeSettings_GetRadius(const JPH_SphereShapeSettings *settings);
JPH_CAPI void JPH_SphereShapeSettings_SetRadius(JPH_SphereShapeSettings *settings, float radius);
JPH_CAPI JPH_SphereShape *JPH_SphereShape_Create(float radius);
JPH_CAPI float JPH_SphereShape_GetRadius(const JPH_SphereShape *shape);

/* PlaneShape */
JPH_CAPI JPH_PlaneShapeSettings *JPH_PlaneShapeSettings_Create(const JPH_Plane *plane,
                                                               const JPH_PhysicsMaterial *material,
                                                               float halfExtent);
JPH_CAPI JPH_PlaneShape *JPH_PlaneShapeSettings_CreateShape(const JPH_PlaneShapeSettings *settings);
JPH_CAPI JPH_PlaneShape *JPH_PlaneShape_Create(const JPH_Plane *plane,
                                               const JPH_PhysicsMaterial *material,
                                               float halfExtent);
JPH_CAPI void JPH_PlaneShape_GetPlane(const JPH_PlaneShape *shape, JPH_Plane *result);
JPH_CAPI float JPH_PlaneShape_GetHalfExtent(const JPH_PlaneShape *shape);

/* TriangleShape */
JPH_CAPI JPH_TriangleShapeSettings *JPH_TriangleShapeSettings_Create(const Vector3 *v1,
                                                                     const Vector3 *v2,
                                                                     const Vector3 *v3,
                                                                     float convexRadius);
JPH_CAPI JPH_TriangleShape *JPH_TriangleShapeSettings_CreateShape(const JPH_TriangleShapeSettings *settings);

JPH_CAPI JPH_TriangleShape *JPH_TriangleShape_Create(const Vector3 *v1,
                                                     const Vector3 *v2,
                                                     const Vector3 *v3,
                                                     float convexRadius);
JPH_CAPI float JPH_TriangleShape_GetConvexRadius(const JPH_TriangleShape *shape);
JPH_CAPI void JPH_TriangleShape_GetVertex1(const JPH_TriangleShape *shape, Vector3 *result);
JPH_CAPI void JPH_TriangleShape_GetVertex2(const JPH_TriangleShape *shape, Vector3 *result);
JPH_CAPI void JPH_TriangleShape_GetVertex3(const JPH_TriangleShape *shape, Vector3 *result);

/* CapsuleShape */
JPH_CAPI JPH_CapsuleShapeSettings *JPH_CapsuleShapeSettings_Create(float halfHeightOfCylinder, float radius);
JPH_CAPI JPH_CapsuleShape *JPH_CapsuleShapeSettings_CreateShape(const JPH_CapsuleShapeSettings *settings);
JPH_CAPI JPH_CapsuleShape *JPH_CapsuleShape_Create(float halfHeightOfCylinder, float radius);
JPH_CAPI float JPH_CapsuleShape_GetRadius(const JPH_CapsuleShape *shape);
JPH_CAPI float JPH_CapsuleShape_GetHalfHeightOfCylinder(const JPH_CapsuleShape *shape);

/* CylinderShape */
JPH_CAPI JPH_CylinderShapeSettings *JPH_CylinderShapeSettings_Create(float halfHeight,
                                                                     float radius,
                                                                     float convexRadius);
JPH_CAPI JPH_CylinderShape *JPH_CylinderShapeSettings_CreateShape(const JPH_CylinderShapeSettings *settings);

JPH_CAPI JPH_CylinderShape *JPH_CylinderShape_Create(float halfHeight, float radius);
JPH_CAPI float JPH_CylinderShape_GetRadius(const JPH_CylinderShape *shape);
JPH_CAPI float JPH_CylinderShape_GetHalfHeight(const JPH_CylinderShape *shape);

/* TaperedCylinderShape */
JPH_CAPI JPH_TaperedCylinderShapeSettings *JPH_TaperedCylinderShapeSettings_Create(
        float halfHeightOfTaperedCylinder,
        float topRadius,
        float bottomRadius,
        float convexRadius /* = cDefaultConvexRadius*/,
        const JPH_PhysicsMaterial *material /* = NULL*/);
JPH_CAPI JPH_TaperedCylinderShape *JPH_TaperedCylinderShapeSettings_CreateShape(const JPH_TaperedCylinderShapeSettings
                                                                                        *settings);
JPH_CAPI float JPH_TaperedCylinderShape_GetTopRadius(const JPH_TaperedCylinderShape *shape);
JPH_CAPI float JPH_TaperedCylinderShape_GetBottomRadius(const JPH_TaperedCylinderShape *shape);
JPH_CAPI float JPH_TaperedCylinderShape_GetConvexRadius(const JPH_TaperedCylinderShape *shape);
JPH_CAPI float JPH_TaperedCylinderShape_GetHalfHeight(const JPH_TaperedCylinderShape *shape);

/* ConvexHullShape */
JPH_CAPI JPH_ConvexHullShapeSettings *JPH_ConvexHullShapeSettings_Create(const Vector3 *points,
                                                                         uint32_t pointsCount,
                                                                         float maxConvexRadius);
JPH_CAPI JPH_ConvexHullShape *JPH_ConvexHullShapeSettings_CreateShape(const JPH_ConvexHullShapeSettings *settings);
JPH_CAPI JPH_ConvexHullShape *JPH_ConvexHullShape_Create(const Vector3 *points,
                                                         uint32_t pointsCount,
                                                         float maxConvexRadius);
JPH_CAPI uint32_t JPH_ConvexHullShape_GetNumPoints(const JPH_ConvexHullShape *shape);
JPH_CAPI void JPH_ConvexHullShape_GetPoint(const JPH_ConvexHullShape *shape, uint32_t index, Vector3 *result);
JPH_CAPI uint32_t JPH_ConvexHullShape_GetNumFaces(const JPH_ConvexHullShape *shape);
JPH_CAPI uint32_t JPH_ConvexHullShape_GetNumVerticesInFace(const JPH_ConvexHullShape *shape, uint32_t faceIndex);
JPH_CAPI uint32_t JPH_ConvexHullShape_GetFaceVertices(const JPH_ConvexHullShape *shape,
                                                      uint32_t faceIndex,
                                                      uint32_t maxVertices,
                                                      uint32_t *vertices);

/* MeshShape */
JPH_CAPI JPH_MeshShapeSettings *JPH_MeshShapeSettings_Create(const JPH_Triangle *triangles, uint32_t triangleCount);
JPH_CAPI JPH_MeshShapeSettings *JPH_MeshShapeSettings_Create2(const Vector3 *vertices,
                                                              uint32_t verticesCount,
                                                              const JPH_IndexedTriangle *triangles,
                                                              uint32_t triangleCount);
JPH_CAPI uint32_t JPH_MeshShapeSettings_GetMaxTrianglesPerLeaf(const JPH_MeshShapeSettings *settings);
JPH_CAPI void JPH_MeshShapeSettings_SetMaxTrianglesPerLeaf(JPH_MeshShapeSettings *settings, uint32_t value);
JPH_CAPI float JPH_MeshShapeSettings_GetActiveEdgeCosThresholdAngle(const JPH_MeshShapeSettings *settings);
JPH_CAPI void JPH_MeshShapeSettings_SetActiveEdgeCosThresholdAngle(JPH_MeshShapeSettings *settings, float value);
JPH_CAPI bool JPH_MeshShapeSettings_GetPerTriangleUserData(const JPH_MeshShapeSettings *settings);
JPH_CAPI void JPH_MeshShapeSettings_SetPerTriangleUserData(JPH_MeshShapeSettings *settings, bool value);
JPH_CAPI JPH_Mesh_Shape_BuildQuality JPH_MeshShapeSettings_GetBuildQuality(const JPH_MeshShapeSettings *settings);
JPH_CAPI void JPH_MeshShapeSettings_SetBuildQuality(JPH_MeshShapeSettings *settings, JPH_Mesh_Shape_BuildQuality value);

JPH_CAPI void JPH_MeshShapeSettings_Sanitize(JPH_MeshShapeSettings *settings);
JPH_CAPI JPH_MeshShape *JPH_MeshShapeSettings_CreateShape(const JPH_MeshShapeSettings *settings);
JPH_CAPI uint32_t JPH_MeshShape_GetTriangleUserData(const JPH_MeshShape *shape, JPH_SubShapeID id);

/* HeightFieldShape */
JPH_CAPI JPH_HeightFieldShapeSettings *JPH_HeightFieldShapeSettings_Create(const float *samples,
                                                                           const Vector3 *offset,
                                                                           const Vector3 *scale,
                                                                           uint32_t sampleCount,
                                                                           const uint8_t *materialIndices);
JPH_CAPI JPH_HeightFieldShape *JPH_HeightFieldShapeSettings_CreateShape(JPH_HeightFieldShapeSettings *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_DetermineMinAndMaxSample(const JPH_HeightFieldShapeSettings *settings,
                                                                    float *pOutMinValue,
                                                                    float *pOutMaxValue,
                                                                    float *pOutQuantizationScale);
JPH_CAPI uint32_t
JPH_HeightFieldShapeSettings_CalculateBitsPerSampleForError(const JPH_HeightFieldShapeSettings *settings,
                                                            float maxError);
JPH_CAPI void JPH_HeightFieldShapeSettings_GetOffset(const JPH_HeightFieldShapeSettings *settings, Vector3 *result);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetOffset(JPH_HeightFieldShapeSettings *settings, const Vector3 *value);
JPH_CAPI void JPH_HeightFieldShapeSettings_GetScale(const JPH_HeightFieldShapeSettings *settings, Vector3 *result);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetScale(JPH_HeightFieldShapeSettings *settings, const Vector3 *value);
JPH_CAPI uint32_t JPH_HeightFieldShapeSettings_GetSampleCount(const JPH_HeightFieldShapeSettings *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetSampleCount(JPH_HeightFieldShapeSettings *settings, uint32_t value);
JPH_CAPI float JPH_HeightFieldShapeSettings_GetMinHeightValue(const JPH_HeightFieldShapeSettings *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetMinHeightValue(JPH_HeightFieldShapeSettings *settings, float value);
JPH_CAPI float JPH_HeightFieldShapeSettings_GetMaxHeightValue(const JPH_HeightFieldShapeSettings *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetMaxHeightValue(JPH_HeightFieldShapeSettings *settings, float value);
JPH_CAPI uint32_t JPH_HeightFieldShapeSettings_GetBlockSize(const JPH_HeightFieldShapeSettings *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetBlockSize(JPH_HeightFieldShapeSettings *settings, uint32_t value);
JPH_CAPI uint32_t JPH_HeightFieldShapeSettings_GetBitsPerSample(const JPH_HeightFieldShapeSettings *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetBitsPerSample(JPH_HeightFieldShapeSettings *settings, uint32_t value);
JPH_CAPI float JPH_HeightFieldShapeSettings_GetActiveEdgeCosThresholdAngle(const JPH_HeightFieldShapeSettings
                                                                                   *settings);
JPH_CAPI void JPH_HeightFieldShapeSettings_SetActiveEdgeCosThresholdAngle(JPH_HeightFieldShapeSettings *settings,
                                                                          float value);

JPH_CAPI uint32_t JPH_HeightFieldShape_GetSampleCount(const JPH_HeightFieldShape *shape);
JPH_CAPI uint32_t JPH_HeightFieldShape_GetBlockSize(const JPH_HeightFieldShape *shape);
JPH_CAPI const JPH_PhysicsMaterial *JPH_HeightFieldShape_GetMaterial(const JPH_HeightFieldShape *shape,
                                                                     uint32_t x,
                                                                     uint32_t y);
JPH_CAPI void JPH_HeightFieldShape_GetPosition(const JPH_HeightFieldShape *shape,
                                               uint32_t x,
                                               uint32_t y,
                                               Vector3 *result);
JPH_CAPI bool JPH_HeightFieldShape_IsNoCollision(const JPH_HeightFieldShape *shape, uint32_t x, uint32_t y);
JPH_CAPI bool JPH_HeightFieldShape_ProjectOntoSurface(const JPH_HeightFieldShape *shape,
                                                      const Vector3 *localPosition,
                                                      Vector3 *outSurfacePosition,
                                                      JPH_SubShapeID *outSubShapeID);
JPH_CAPI float JPH_HeightFieldShape_GetMinHeightValue(const JPH_HeightFieldShape *shape);
JPH_CAPI float JPH_HeightFieldShape_GetMaxHeightValue(const JPH_HeightFieldShape *shape);

/* TaperedCapsuleShape */
JPH_CAPI JPH_TaperedCapsuleShapeSettings *JPH_TaperedCapsuleShapeSettings_Create(float halfHeightOfTaperedCylinder,
                                                                                 float topRadius,
                                                                                 float bottomRadius);
JPH_CAPI JPH_TaperedCapsuleShape *JPH_TaperedCapsuleShapeSettings_CreateShape(const JPH_TaperedCapsuleShapeSettings
                                                                                      *settings);

JPH_CAPI float JPH_TaperedCapsuleShape_GetTopRadius(const JPH_TaperedCapsuleShape *shape);
JPH_CAPI float JPH_TaperedCapsuleShape_GetBottomRadius(const JPH_TaperedCapsuleShape *shape);
JPH_CAPI float JPH_TaperedCapsuleShape_GetHalfHeight(const JPH_TaperedCapsuleShape *shape);

/* CompoundShape */
JPH_CAPI void JPH_CompoundShapeSettings_AddShape(JPH_CompoundShapeSettings *settings,
                                                 const Vector3 *position,
                                                 const JPH_Quat *rotation,
                                                 const JPH_ShapeSettings *shapeSettings,
                                                 uint32_t userData);
JPH_CAPI void JPH_CompoundShapeSettings_AddShape2(JPH_CompoundShapeSettings *settings,
                                                  const Vector3 *position,
                                                  const JPH_Quat *rotation,
                                                  const JPH_Shape *shape,
                                                  uint32_t userData);
JPH_CAPI uint32_t JPH_CompoundShape_GetNumSubShapes(const JPH_CompoundShape *shape);
JPH_CAPI void JPH_CompoundShape_GetSubShape(const JPH_CompoundShape *shape,
                                            uint32_t index,
                                            const JPH_Shape **subShape,
                                            Vector3 *positionCOM,
                                            JPH_Quat *rotation,
                                            uint32_t *userData);
JPH_CAPI uint32_t JPH_CompoundShape_GetSubShapeIndexFromID(const JPH_CompoundShape *shape,
                                                           JPH_SubShapeID id,
                                                           JPH_SubShapeID *remainder);

/* StaticCompoundShape */
JPH_CAPI JPH_StaticCompoundShapeSettings *JPH_StaticCompoundShapeSettings_Create(void);
JPH_CAPI JPH_StaticCompoundShape *JPH_StaticCompoundShape_Create(const JPH_StaticCompoundShapeSettings *settings);

/* MutableCompoundShape */
JPH_CAPI JPH_MutableCompoundShapeSettings *JPH_MutableCompoundShapeSettings_Create(void);
JPH_CAPI JPH_MutableCompoundShape *JPH_MutableCompoundShape_Create(const JPH_MutableCompoundShapeSettings *settings);

JPH_CAPI uint32_t JPH_MutableCompoundShape_AddShape(JPH_MutableCompoundShape *shape,
                                                    const Vector3 *position,
                                                    const JPH_Quat *rotation,
                                                    const JPH_Shape *child,
                                                    uint32_t userData /* = 0 */,
                                                    uint32_t index /* = UINT32_MAX */);
JPH_CAPI void JPH_MutableCompoundShape_RemoveShape(JPH_MutableCompoundShape *shape, uint32_t index);
JPH_CAPI void JPH_MutableCompoundShape_ModifyShape(JPH_MutableCompoundShape *shape,
                                                   uint32_t index,
                                                   const Vector3 *position,
                                                   const JPH_Quat *rotation);
JPH_CAPI void JPH_MutableCompoundShape_ModifyShape2(JPH_MutableCompoundShape *shape,
                                                    uint32_t index,
                                                    const Vector3 *position,
                                                    const JPH_Quat *rotation,
                                                    const JPH_Shape *newShape);
JPH_CAPI void JPH_MutableCompoundShape_AdjustCenterOfMass(JPH_MutableCompoundShape *shape);

/* DecoratedShape */
JPH_CAPI const JPH_Shape *JPH_DecoratedShape_GetInnerShape(const JPH_DecoratedShape *shape);

/* RotatedTranslatedShape */
JPH_CAPI JPH_RotatedTranslatedShapeSettings *JPH_RotatedTranslatedShapeSettings_Create(const Vector3 *position,
                                                                                       const JPH_Quat *rotation,
                                                                                       const JPH_ShapeSettings
                                                                                               *shapeSettings);
JPH_CAPI JPH_RotatedTranslatedShapeSettings *JPH_RotatedTranslatedShapeSettings_Create2(const Vector3 *position,
                                                                                        const JPH_Quat *rotation,
                                                                                        const JPH_Shape *shape);
JPH_CAPI JPH_RotatedTranslatedShape *JPH_RotatedTranslatedShapeSettings_CreateShape(
        const JPH_RotatedTranslatedShapeSettings *settings);
JPH_CAPI JPH_RotatedTranslatedShape *JPH_RotatedTranslatedShape_Create(const Vector3 *position,
                                                                       const JPH_Quat *rotation,
                                                                       const JPH_Shape *shape);
JPH_CAPI void JPH_RotatedTranslatedShape_GetPosition(const JPH_RotatedTranslatedShape *shape, Vector3 *position);
JPH_CAPI void JPH_RotatedTranslatedShape_GetRotation(const JPH_RotatedTranslatedShape *shape, JPH_Quat *rotation);

/* ScaledShape */
JPH_CAPI JPH_ScaledShapeSettings *JPH_ScaledShapeSettings_Create(const JPH_ShapeSettings *shapeSettings,
                                                                 const Vector3 *scale);
JPH_CAPI JPH_ScaledShapeSettings *JPH_ScaledShapeSettings_Create2(const JPH_Shape *shape, const Vector3 *scale);
JPH_CAPI JPH_ScaledShape *JPH_ScaledShapeSettings_CreateShape(const JPH_ScaledShapeSettings *settings);
JPH_CAPI JPH_ScaledShape *JPH_ScaledShape_Create(const JPH_Shape *shape, const Vector3 *scale);
JPH_CAPI void JPH_ScaledShape_GetScale(const JPH_ScaledShape *shape, Vector3 *result);

/* OffsetCenterOfMassShape */
JPH_CAPI JPH_OffsetCenterOfMassShapeSettings *JPH_OffsetCenterOfMassShapeSettings_Create(const Vector3 *offset,
                                                                                         const JPH_ShapeSettings
                                                                                                 *shapeSettings);
JPH_CAPI JPH_OffsetCenterOfMassShapeSettings *JPH_OffsetCenterOfMassShapeSettings_Create2(const Vector3 *offset,
                                                                                          const JPH_Shape *shape);
JPH_CAPI JPH_OffsetCenterOfMassShape *JPH_OffsetCenterOfMassShapeSettings_CreateShape(
        const JPH_OffsetCenterOfMassShapeSettings *settings);

JPH_CAPI JPH_OffsetCenterOfMassShape *JPH_OffsetCenterOfMassShape_Create(const Vector3 *offset, const JPH_Shape *shape);
JPH_CAPI void JPH_OffsetCenterOfMassShape_GetOffset(const JPH_OffsetCenterOfMassShape *shape, Vector3 *result);

/* EmptyShape */
JPH_CAPI JPH_EmptyShapeSettings *JPH_EmptyShapeSettings_Create(const Vector3 *centerOfMass);
JPH_CAPI JPH_EmptyShape *JPH_EmptyShapeSettings_CreateShape(const JPH_EmptyShapeSettings *settings);

/* JPH_Constraint */
JPH_CAPI void JPH_Constraint_Destroy(JPH_Constraint *constraint);
JPH_CAPI JPH_ConstraintType JPH_Constraint_GetType(const JPH_Constraint *constraint);
JPH_CAPI JPH_ConstraintSubType JPH_Constraint_GetSubType(const JPH_Constraint *constraint);
JPH_CAPI uint32_t JPH_Constraint_GetConstraintPriority(const JPH_Constraint *constraint);
JPH_CAPI void JPH_Constraint_SetConstraintPriority(JPH_Constraint *constraint, uint32_t priority);
JPH_CAPI uint32_t JPH_Constraint_GetNumVelocityStepsOverride(const JPH_Constraint *constraint);
JPH_CAPI void JPH_Constraint_SetNumVelocityStepsOverride(JPH_Constraint *constraint, uint32_t value);
JPH_CAPI uint32_t JPH_Constraint_GetNumPositionStepsOverride(const JPH_Constraint *constraint);
JPH_CAPI void JPH_Constraint_SetNumPositionStepsOverride(JPH_Constraint *constraint, uint32_t value);
JPH_CAPI bool JPH_Constraint_GetEnabled(const JPH_Constraint *constraint);
JPH_CAPI void JPH_Constraint_SetEnabled(JPH_Constraint *constraint, bool enabled);
JPH_CAPI uint64_t JPH_Constraint_GetUserData(const JPH_Constraint *constraint);
JPH_CAPI void JPH_Constraint_SetUserData(JPH_Constraint *constraint, uint64_t userData);
JPH_CAPI void JPH_Constraint_NotifyShapeChanged(JPH_Constraint *constraint, JPH_BodyID bodyID, const Vector3 *deltaCOM);
JPH_CAPI void JPH_Constraint_ResetWarmStart(JPH_Constraint *constraint);
JPH_CAPI bool JPH_Constraint_IsActive(const JPH_Constraint *constraint);
JPH_CAPI void JPH_Constraint_SetupVelocityConstraint(JPH_Constraint *constraint, float deltaTime);
JPH_CAPI void JPH_Constraint_WarmStartVelocityConstraint(JPH_Constraint *constraint, float warmStartImpulseRatio);
JPH_CAPI bool JPH_Constraint_SolveVelocityConstraint(JPH_Constraint *constraint, float deltaTime);
JPH_CAPI bool JPH_Constraint_SolvePositionConstraint(JPH_Constraint *constraint, float deltaTime, float baumgarte);

/* JPH_TwoBodyConstraint */
JPH_CAPI JPH_Body *JPH_TwoBodyConstraint_GetBody1(const JPH_TwoBodyConstraint *constraint);
JPH_CAPI JPH_Body *JPH_TwoBodyConstraint_GetBody2(const JPH_TwoBodyConstraint *constraint);
JPH_CAPI void JPH_TwoBodyConstraint_GetConstraintToBody1Matrix(const JPH_TwoBodyConstraint *constraint,
                                                               JPH_Mat44 *result);
JPH_CAPI void JPH_TwoBodyConstraint_GetConstraintToBody2Matrix(const JPH_TwoBodyConstraint *constraint,
                                                               JPH_Mat44 *result);

/* JPH_FixedConstraint */
JPH_CAPI void JPH_FixedConstraintSettings_Init(JPH_FixedConstraintSettings *settings);
JPH_CAPI JPH_FixedConstraint *JPH_FixedConstraint_Create(const JPH_FixedConstraintSettings *settings,
                                                         JPH_Body *body1,
                                                         JPH_Body *body2);
JPH_CAPI void JPH_FixedConstraint_GetSettings(const JPH_FixedConstraint *constraint,
                                              JPH_FixedConstraintSettings *settings);
JPH_CAPI void JPH_FixedConstraint_GetTotalLambdaPosition(const JPH_FixedConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_FixedConstraint_GetTotalLambdaRotation(const JPH_FixedConstraint *constraint, Vector3 *result);

/* JPH_DistanceConstraint */
JPH_CAPI void JPH_DistanceConstraintSettings_Init(JPH_DistanceConstraintSettings *settings);
JPH_CAPI JPH_DistanceConstraint *JPH_DistanceConstraint_Create(const JPH_DistanceConstraintSettings *settings,
                                                               JPH_Body *body1,
                                                               JPH_Body *body2);
JPH_CAPI void JPH_DistanceConstraint_GetSettings(const JPH_DistanceConstraint *constraint,
                                                 JPH_DistanceConstraintSettings *settings);
JPH_CAPI void JPH_DistanceConstraint_SetDistance(JPH_DistanceConstraint *constraint,
                                                 float minDistance,
                                                 float maxDistance);
JPH_CAPI float JPH_DistanceConstraint_GetMinDistance(JPH_DistanceConstraint *constraint);
JPH_CAPI float JPH_DistanceConstraint_GetMaxDistance(JPH_DistanceConstraint *constraint);
JPH_CAPI void JPH_DistanceConstraint_GetLimitsSpringSettings(JPH_DistanceConstraint *constraint,
                                                             JPH_SpringSettings *result);
JPH_CAPI void JPH_DistanceConstraint_SetLimitsSpringSettings(JPH_DistanceConstraint *constraint,
                                                             const JPH_SpringSettings *settings);
JPH_CAPI float JPH_DistanceConstraint_GetTotalLambdaPosition(const JPH_DistanceConstraint *constraint);

/* JPH_PointConstraint */
JPH_CAPI void JPH_PointConstraintSettings_Init(JPH_PointConstraintSettings *settings);
JPH_CAPI JPH_PointConstraint *JPH_PointConstraint_Create(const JPH_PointConstraintSettings *settings,
                                                         JPH_Body *body1,
                                                         JPH_Body *body2);
JPH_CAPI void JPH_PointConstraint_GetSettings(const JPH_PointConstraint *constraint,
                                              JPH_PointConstraintSettings *settings);
JPH_CAPI void JPH_PointConstraint_SetPoint1(JPH_PointConstraint *constraint,
                                            JPH_ConstraintSpace space,
                                            const JPH_RVec3 *value);
JPH_CAPI void JPH_PointConstraint_SetPoint2(JPH_PointConstraint *constraint,
                                            JPH_ConstraintSpace space,
                                            const JPH_RVec3 *value);
JPH_CAPI void JPH_PointConstraint_GetLocalSpacePoint1(const JPH_PointConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_PointConstraint_GetLocalSpacePoint2(const JPH_PointConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_PointConstraint_GetTotalLambdaPosition(const JPH_PointConstraint *constraint, Vector3 *result);

/* JPH_HingeConstraint */
JPH_CAPI void JPH_HingeConstraintSettings_Init(JPH_HingeConstraintSettings *settings);
JPH_CAPI JPH_HingeConstraint *JPH_HingeConstraint_Create(const JPH_HingeConstraintSettings *settings,
                                                         JPH_Body *body1,
                                                         JPH_Body *body2);
JPH_CAPI void JPH_HingeConstraint_GetSettings(JPH_HingeConstraint *constraint, JPH_HingeConstraintSettings *settings);
JPH_CAPI void JPH_HingeConstraint_GetLocalSpacePoint1(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_HingeConstraint_GetLocalSpacePoint2(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_HingeConstraint_GetLocalSpaceHingeAxis1(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_HingeConstraint_GetLocalSpaceHingeAxis2(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_HingeConstraint_GetLocalSpaceNormalAxis1(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_HingeConstraint_GetLocalSpaceNormalAxis2(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI float JPH_HingeConstraint_GetCurrentAngle(JPH_HingeConstraint *constraint);
JPH_CAPI void JPH_HingeConstraint_SetMaxFrictionTorque(JPH_HingeConstraint *constraint, float frictionTorque);
JPH_CAPI float JPH_HingeConstraint_GetMaxFrictionTorque(JPH_HingeConstraint *constraint);
JPH_CAPI void JPH_HingeConstraint_SetMotorSettings(JPH_HingeConstraint *constraint, const JPH_MotorSettings *settings);
JPH_CAPI void JPH_HingeConstraint_GetMotorSettings(JPH_HingeConstraint *constraint, JPH_MotorSettings *result);
JPH_CAPI void JPH_HingeConstraint_SetMotorState(JPH_HingeConstraint *constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_HingeConstraint_GetMotorState(JPH_HingeConstraint *constraint);
JPH_CAPI void JPH_HingeConstraint_SetTargetAngularVelocity(JPH_HingeConstraint *constraint, float angularVelocity);
JPH_CAPI float JPH_HingeConstraint_GetTargetAngularVelocity(JPH_HingeConstraint *constraint);
JPH_CAPI void JPH_HingeConstraint_SetTargetAngle(JPH_HingeConstraint *constraint, float angle);
JPH_CAPI float JPH_HingeConstraint_GetTargetAngle(JPH_HingeConstraint *constraint);
JPH_CAPI void JPH_HingeConstraint_SetLimits(JPH_HingeConstraint *constraint, float inLimitsMin, float inLimitsMax);
JPH_CAPI float JPH_HingeConstraint_GetLimitsMin(JPH_HingeConstraint *constraint);
JPH_CAPI float JPH_HingeConstraint_GetLimitsMax(JPH_HingeConstraint *constraint);
JPH_CAPI bool JPH_HingeConstraint_HasLimits(JPH_HingeConstraint *constraint);
JPH_CAPI void JPH_HingeConstraint_GetLimitsSpringSettings(JPH_HingeConstraint *constraint, JPH_SpringSettings *result);
JPH_CAPI void JPH_HingeConstraint_SetLimitsSpringSettings(JPH_HingeConstraint *constraint,
                                                          const JPH_SpringSettings *settings);
JPH_CAPI void JPH_HingeConstraint_GetTotalLambdaPosition(const JPH_HingeConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_HingeConstraint_GetTotalLambdaRotation(const JPH_HingeConstraint *constraint, float rotation[2]);
JPH_CAPI float JPH_HingeConstraint_GetTotalLambdaRotationLimits(const JPH_HingeConstraint *constraint);
JPH_CAPI float JPH_HingeConstraint_GetTotalLambdaMotor(const JPH_HingeConstraint *constraint);

/* JPH_SliderConstraint */
JPH_CAPI void JPH_SliderConstraintSettings_Init(JPH_SliderConstraintSettings *settings);
JPH_CAPI void JPH_SliderConstraintSettings_SetSliderAxis(JPH_SliderConstraintSettings *settings, const Vector3 *axis);

JPH_CAPI JPH_SliderConstraint *JPH_SliderConstraint_Create(const JPH_SliderConstraintSettings *settings,
                                                           JPH_Body *body1,
                                                           JPH_Body *body2);
JPH_CAPI void JPH_SliderConstraint_GetSettings(JPH_SliderConstraint *constraint,
                                               JPH_SliderConstraintSettings *settings);
JPH_CAPI float JPH_SliderConstraint_GetCurrentPosition(JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_SetMaxFrictionForce(JPH_SliderConstraint *constraint, float frictionForce);
JPH_CAPI float JPH_SliderConstraint_GetMaxFrictionForce(JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_SetMotorSettings(JPH_SliderConstraint *constraint,
                                                    const JPH_MotorSettings *settings);
JPH_CAPI void JPH_SliderConstraint_GetMotorSettings(const JPH_SliderConstraint *constraint, JPH_MotorSettings *result);
JPH_CAPI void JPH_SliderConstraint_SetMotorState(JPH_SliderConstraint *constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_SliderConstraint_GetMotorState(JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_SetTargetVelocity(JPH_SliderConstraint *constraint, float velocity);
JPH_CAPI float JPH_SliderConstraint_GetTargetVelocity(JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_SetTargetPosition(JPH_SliderConstraint *constraint, float position);
JPH_CAPI float JPH_SliderConstraint_GetTargetPosition(JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_SetLimits(JPH_SliderConstraint *constraint, float inLimitsMin, float inLimitsMax);
JPH_CAPI float JPH_SliderConstraint_GetLimitsMin(JPH_SliderConstraint *constraint);
JPH_CAPI float JPH_SliderConstraint_GetLimitsMax(JPH_SliderConstraint *constraint);
JPH_CAPI bool JPH_SliderConstraint_HasLimits(JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_GetLimitsSpringSettings(JPH_SliderConstraint *constraint,
                                                           JPH_SpringSettings *result);
JPH_CAPI void JPH_SliderConstraint_SetLimitsSpringSettings(JPH_SliderConstraint *constraint,
                                                           const JPH_SpringSettings *settings);
JPH_CAPI void JPH_SliderConstraint_GetTotalLambdaPosition(const JPH_SliderConstraint *constraint, float position[2]);
JPH_CAPI float JPH_SliderConstraint_GetTotalLambdaPositionLimits(const JPH_SliderConstraint *constraint);
JPH_CAPI void JPH_SliderConstraint_GetTotalLambdaRotation(const JPH_SliderConstraint *constraint, Vector3 *result);
JPH_CAPI float JPH_SliderConstraint_GetTotalLambdaMotor(const JPH_SliderConstraint *constraint);

/* JPH_ConeConstraint */
JPH_CAPI void JPH_ConeConstraintSettings_Init(JPH_ConeConstraintSettings *settings);
JPH_CAPI JPH_ConeConstraint *JPH_ConeConstraint_Create(const JPH_ConeConstraintSettings *settings,
                                                       JPH_Body *body1,
                                                       JPH_Body *body2);
JPH_CAPI void JPH_ConeConstraint_GetSettings(JPH_ConeConstraint *constraint, JPH_ConeConstraintSettings *settings);
JPH_CAPI void JPH_ConeConstraint_SetHalfConeAngle(JPH_ConeConstraint *constraint, float halfConeAngle);
JPH_CAPI float JPH_ConeConstraint_GetCosHalfConeAngle(const JPH_ConeConstraint *constraint);
JPH_CAPI void JPH_ConeConstraint_GetTotalLambdaPosition(const JPH_ConeConstraint *constraint, Vector3 *result);
JPH_CAPI float JPH_ConeConstraint_GetTotalLambdaRotation(const JPH_ConeConstraint *constraint);

/* JPH_SwingTwistConstraint */
JPH_CAPI void JPH_SwingTwistConstraintSettings_Init(JPH_SwingTwistConstraintSettings *settings);
JPH_CAPI JPH_SwingTwistConstraint *JPH_SwingTwistConstraint_Create(const JPH_SwingTwistConstraintSettings *settings,
                                                                   JPH_Body *body1,
                                                                   JPH_Body *body2);
JPH_CAPI void JPH_SwingTwistConstraint_GetSettings(JPH_SwingTwistConstraint *constraint,
                                                   JPH_SwingTwistConstraintSettings *settings);
JPH_CAPI float JPH_SwingTwistConstraint_GetNormalHalfConeAngle(JPH_SwingTwistConstraint *constraint);
JPH_CAPI void JPH_SwingTwistConstraint_GetTotalLambdaPosition(const JPH_SwingTwistConstraint *constraint,
                                                              Vector3 *result);
JPH_CAPI float JPH_SwingTwistConstraint_GetTotalLambdaTwist(const JPH_SwingTwistConstraint *constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetTotalLambdaSwingY(const JPH_SwingTwistConstraint *constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetTotalLambdaSwingZ(const JPH_SwingTwistConstraint *constraint);
JPH_CAPI void JPH_SwingTwistConstraint_GetTotalLambdaMotor(const JPH_SwingTwistConstraint *constraint, Vector3 *result);

/* JPH_SixDOFConstraint */
JPH_CAPI void JPH_SixDOFConstraintSettings_Init(JPH_SixDOFConstraintSettings *settings);
JPH_CAPI void JPH_SixDOFConstraintSettings_MakeFreeAxis(JPH_SixDOFConstraintSettings *settings,
                                                        JPH_SixDOFConstraintAxis axis);
JPH_CAPI bool JPH_SixDOFConstraintSettings_IsFreeAxis(const JPH_SixDOFConstraintSettings *settings,
                                                      JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraintSettings_MakeFixedAxis(JPH_SixDOFConstraintSettings *settings,
                                                         JPH_SixDOFConstraintAxis axis);
JPH_CAPI bool JPH_SixDOFConstraintSettings_IsFixedAxis(const JPH_SixDOFConstraintSettings *settings,
                                                       JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraintSettings_SetLimitedAxis(JPH_SixDOFConstraintSettings *settings,
                                                          JPH_SixDOFConstraintAxis axis,
                                                          float min,
                                                          float max);

JPH_CAPI JPH_SixDOFConstraint *JPH_SixDOFConstraint_Create(const JPH_SixDOFConstraintSettings *settings,
                                                           JPH_Body *body1,
                                                           JPH_Body *body2);
JPH_CAPI void JPH_SixDOFConstraint_GetSettings(JPH_SixDOFConstraint *constraint,
                                               JPH_SixDOFConstraintSettings *settings);
JPH_CAPI float JPH_SixDOFConstraint_GetLimitsMin(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis);
JPH_CAPI float JPH_SixDOFConstraint_GetLimitsMax(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraint_GetTotalLambdaPosition(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetTotalLambdaRotation(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetTotalLambdaMotorTranslation(const JPH_SixDOFConstraint *constraint,
                                                                  Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetTotalLambdaMotorRotation(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetTranslationLimitsMin(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetTranslationLimitsMax(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetRotationLimitsMin(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_GetRotationLimitsMax(const JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI bool JPH_SixDOFConstraint_IsFixedAxis(const JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis);
JPH_CAPI bool JPH_SixDOFConstraint_IsFreeAxis(const JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraint_GetLimitsSpringSettings(JPH_SixDOFConstraint *constraint,
                                                           JPH_SpringSettings *result,
                                                           JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraint_SetLimitsSpringSettings(JPH_SixDOFConstraint *constraint,
                                                           const JPH_SpringSettings *settings,
                                                           JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraint_SetMaxFriction(JPH_SixDOFConstraint *constraint,
                                                  JPH_SixDOFConstraintAxis axis,
                                                  float inFriction);
JPH_CAPI float JPH_SixDOFConstraint_GetMaxFriction(JPH_SixDOFConstraint *constraint, JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraint_GetRotationInConstraintSpace(JPH_SixDOFConstraint *constraint, JPH_Quat *result);
JPH_CAPI void JPH_SixDOFConstraint_GetMotorSettings(JPH_SixDOFConstraint *constraint,
                                                    JPH_SixDOFConstraintAxis axis,
                                                    JPH_MotorSettings *result);
JPH_CAPI void JPH_SixDOFConstraint_SetMotorState(JPH_SixDOFConstraint *constraint,
                                                 JPH_SixDOFConstraintAxis axis,
                                                 JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_SixDOFConstraint_GetMotorState(JPH_SixDOFConstraint *constraint,
                                                           JPH_SixDOFConstraintAxis axis);
JPH_CAPI void JPH_SixDOFConstraint_SetTargetVelocityCS(JPH_SixDOFConstraint *constraint, const Vector3 *inVelocity);
JPH_CAPI void JPH_SixDOFConstraint_GetTargetVelocityCS(JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_SetTargetAngularVelocityCS(JPH_SixDOFConstraint *constraint,
                                                              const Vector3 *inAngularVelocity);
JPH_CAPI void JPH_SixDOFConstraint_GetTargetAngularVelocityCS(JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_SetTargetPositionCS(JPH_SixDOFConstraint *constraint, const Vector3 *inPosition);
JPH_CAPI void JPH_SixDOFConstraint_GetTargetPositionCS(JPH_SixDOFConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_SixDOFConstraint_SetTargetOrientationCS(JPH_SixDOFConstraint *constraint,
                                                          const JPH_Quat *inOrientation);
JPH_CAPI void JPH_SixDOFConstraint_GetTargetOrientationCS(JPH_SixDOFConstraint *constraint, JPH_Quat *result);
JPH_CAPI void JPH_SixDOFConstraint_SetTargetOrientationBS(JPH_SixDOFConstraint *constraint,
                                                          const JPH_Quat *inOrientation);

/* JPH_GearConstraint */
JPH_CAPI void JPH_GearConstraintSettings_Init(JPH_GearConstraintSettings *settings);
JPH_CAPI JPH_GearConstraint *JPH_GearConstraint_Create(const JPH_GearConstraintSettings *settings,
                                                       JPH_Body *body1,
                                                       JPH_Body *body2);
JPH_CAPI void JPH_GearConstraint_GetSettings(JPH_GearConstraint *constraint, JPH_GearConstraintSettings *settings);
JPH_CAPI void JPH_GearConstraint_SetConstraints(JPH_GearConstraint *constraint,
                                                const JPH_Constraint *gear1,
                                                const JPH_Constraint *gear2);
JPH_CAPI float JPH_GearConstraint_GetTotalLambda(const JPH_GearConstraint *constraint);

//--------------------------------------------------------------------------------------------------
// JPH_BodyLockInterface
//--------------------------------------------------------------------------------------------------
JPH_CAPI void JPH_BodyLockInterface_LockRead(const JPH_BodyLockInterface *lockInterface,
                                             JPH_BodyID bodyID,
                                             JPH_BodyLockRead *outLock);
JPH_CAPI void JPH_BodyLockInterface_UnlockRead(const JPH_BodyLockInterface *lockInterface, JPH_BodyLockRead *ioLock);

JPH_CAPI void JPH_BodyLockInterface_LockWrite(const JPH_BodyLockInterface *lockInterface,
                                              JPH_BodyID bodyID,
                                              JPH_BodyLockWrite *outLock);
JPH_CAPI void JPH_BodyLockInterface_UnlockWrite(const JPH_BodyLockInterface *lockInterface, JPH_BodyLockWrite *ioLock);

JPH_CAPI JPH_BodyLockMultiRead *JPH_BodyLockInterface_LockMultiRead(const JPH_BodyLockInterface *lockInterface,
                                                                    const JPH_BodyID *bodyIDs,
                                                                    int count);
JPH_CAPI void JPH_BodyLockMultiRead_Destroy(JPH_BodyLockMultiRead *ioLock);
JPH_CAPI const JPH_Body *JPH_BodyLockMultiRead_GetBody(JPH_BodyLockMultiRead *ioLock, int bodyIndex);

JPH_CAPI JPH_BodyLockMultiWrite *JPH_BodyLockInterface_LockMultiWrite(const JPH_BodyLockInterface *lockInterface,
                                                                      const JPH_BodyID *bodyIDs,
                                                                      int count);
JPH_CAPI void JPH_BodyLockMultiWrite_Destroy(JPH_BodyLockMultiWrite *ioLock);
JPH_CAPI JPH_Body *JPH_BodyLockMultiWrite_GetBody(JPH_BodyLockMultiWrite *ioLock, int bodyIndex);

//--------------------------------------------------------------------------------------------------
// JPH_RayCast
//--------------------------------------------------------------------------------------------------
JPH_CAPI void JPH_RayCast_GetPointOnRay(const Vector3 *origin,
                                        const Vector3 *direction,
                                        float fraction,
                                        Vector3 *result);
JPH_CAPI void JPH_RRayCast_GetPointOnRay(const JPH_RVec3 *origin,
                                         const Vector3 *direction,
                                         float fraction,
                                         JPH_RVec3 *result);

/* JPH_SimShapeFilter */
typedef struct JPH_SimShapeFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(void *userData,
                                          const JPH_Body *body1,
                                          const JPH_Shape *shape1,
                                          const JPH_SubShapeID *subShapeIDOfShape1,
                                          const JPH_Body *body2,
                                          const JPH_Shape *shape2,
                                          const JPH_SubShapeID *subShapeIDOfShape2);
} JPH_SimShapeFilter_Impl;

JPH_CAPI void JPH_SimShapeFilter_SetImpl(const JPH_SimShapeFilter_Impl *impl);
JPH_CAPI JPH_SimShapeFilter *JPH_SimShapeFilter_Create(void *userData);
JPH_CAPI void JPH_SimShapeFilter_Destroy(JPH_SimShapeFilter *filter);

/* Contact listener */
typedef struct JPH_ContactListener_Impl
{
        JPH_ValidateResult(JPH_API_CALL *OnContactValidate)(const JPH_Body *body1,
                                                            const JPH_Body *body2,
                                                            const JPH_RVec3 *baseOffset,
                                                            const JPH_CollideShapeResult *collisionResult);

        void(JPH_API_CALL *OnContactAdded)(const JPH_Body *body1,
                                           const JPH_Body *body2,
                                           const JPH_ContactManifold *manifold,
                                           JPH_ContactSettings *settings);

        void(JPH_API_CALL *OnContactPersisted)(const JPH_Body *body1,
                                               const JPH_Body *body2,
                                               const JPH_ContactManifold *manifold,
                                               JPH_ContactSettings *settings);

        void(JPH_API_CALL *OnContactRemoved)(const JPH_SubShapeIDPair *subShapePair);
} JPH_ContactListener_Impl;

JPH_CAPI void JPH_ContactListener_SetImpl(JPH_ContactListener *listener, const JPH_ContactListener_Impl *impl);
JPH_CAPI JPH_ContactListener *JPH_ContactListener_Create(void);
JPH_CAPI void JPH_ContactListener_Destroy(JPH_ContactListener *listener);

/* BodyActivationListener */
typedef struct JPH_BodyActivationListener_Impl
{
        void(JPH_API_CALL *OnBodyActivated)(void *userData, JPH_BodyID bodyID, uint64_t bodyUserData);
        void(JPH_API_CALL *OnBodyDeactivated)(void *userData, JPH_BodyID bodyID, uint64_t bodyUserData);
} JPH_BodyActivationListener_Impl;

JPH_CAPI void JPH_BodyActivationListener_SetImpl(const JPH_BodyActivationListener_Impl *impl);
JPH_CAPI JPH_BodyActivationListener *JPH_BodyActivationListener_Create(void *userData);
JPH_CAPI void JPH_BodyActivationListener_Destroy(JPH_BodyActivationListener *listener);

/* JPH_BodyDrawFilter */
typedef struct JPH_BodyDrawFilter_Impl
{
        bool(JPH_API_CALL *ShouldDraw)(void *userData, const JPH_Body *body);
} JPH_BodyDrawFilter_Impl;

JPH_CAPI void JPH_BodyDrawFilter_SetImpl(const JPH_BodyDrawFilter_Impl *impl);
JPH_CAPI JPH_BodyDrawFilter *JPH_BodyDrawFilter_Create(void *userData);
JPH_CAPI void JPH_BodyDrawFilter_Destroy(JPH_BodyDrawFilter *filter);

/* ContactManifold */
JPH_CAPI void JPH_ContactManifold_GetWorldSpaceNormal(const JPH_ContactManifold *manifold, Vector3 *result);
JPH_CAPI float JPH_ContactManifold_GetPenetrationDepth(const JPH_ContactManifold *manifold);
JPH_CAPI JPH_SubShapeID JPH_ContactManifold_GetSubShapeID1(const JPH_ContactManifold *manifold);
JPH_CAPI JPH_SubShapeID JPH_ContactManifold_GetSubShapeID2(const JPH_ContactManifold *manifold);
JPH_CAPI uint32_t JPH_ContactManifold_GetPointCount(const JPH_ContactManifold *manifold);
JPH_CAPI void JPH_ContactManifold_GetWorldSpaceContactPointOn1(const JPH_ContactManifold *manifold,
                                                               uint32_t index,
                                                               JPH_RVec3 *result);
JPH_CAPI void JPH_ContactManifold_GetWorldSpaceContactPointOn2(const JPH_ContactManifold *manifold,
                                                               uint32_t index,
                                                               JPH_RVec3 *result);

/* CharacterBase */
JPH_CAPI void JPH_CharacterBase_Destroy(JPH_CharacterBase *character);
JPH_CAPI float JPH_CharacterBase_GetCosMaxSlopeAngle(JPH_CharacterBase *character);
JPH_CAPI void JPH_CharacterBase_SetMaxSlopeAngle(JPH_CharacterBase *character, float maxSlopeAngle);
JPH_CAPI void JPH_CharacterBase_GetUp(JPH_CharacterBase *character, Vector3 *result);
JPH_CAPI void JPH_CharacterBase_SetUp(JPH_CharacterBase *character, const Vector3 *value);
JPH_CAPI bool JPH_CharacterBase_IsSlopeTooSteep(JPH_CharacterBase *character, const Vector3 *value);
JPH_CAPI const JPH_Shape *JPH_CharacterBase_GetShape(JPH_CharacterBase *character);

JPH_CAPI JPH_GroundState JPH_CharacterBase_GetGroundState(JPH_CharacterBase *character);
JPH_CAPI bool JPH_CharacterBase_IsSupported(JPH_CharacterBase *character);
JPH_CAPI void JPH_CharacterBase_GetGroundPosition(JPH_CharacterBase *character, JPH_RVec3 *position);
JPH_CAPI void JPH_CharacterBase_GetGroundNormal(JPH_CharacterBase *character, Vector3 *normal);
JPH_CAPI void JPH_CharacterBase_GetGroundVelocity(JPH_CharacterBase *character, Vector3 *velocity);
JPH_CAPI const JPH_PhysicsMaterial *JPH_CharacterBase_GetGroundMaterial(JPH_CharacterBase *character);
JPH_CAPI JPH_BodyID JPH_CharacterBase_GetGroundBodyId(JPH_CharacterBase *character);
JPH_CAPI JPH_SubShapeID JPH_CharacterBase_GetGroundSubShapeId(JPH_CharacterBase *character);
JPH_CAPI uint64_t JPH_CharacterBase_GetGroundUserData(JPH_CharacterBase *character);

/* CharacterSettings */
JPH_CAPI void JPH_CharacterSettings_Init(JPH_CharacterSettings *settings);

/* Character */
JPH_CAPI JPH_Character *JPH_Character_Create(const JPH_CharacterSettings *settings,
                                             const JPH_RVec3 *position,
                                             const JPH_Quat *rotation,
                                             uint64_t userData,
                                             JPH_PhysicsSystem *system);

JPH_CAPI void JPH_Character_AddToPhysicsSystem(JPH_Character *character,
                                               JPH_Activation activationMode /*= JPH_ActivationActivate */,
                                               bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_RemoveFromPhysicsSystem(JPH_Character *character, bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_Activate(JPH_Character *character, bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_PostSimulation(JPH_Character *character,
                                           float maxSeparationDistance,
                                           bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_SetLinearAndAngularVelocity(JPH_Character *character,
                                                        const Vector3 *linearVelocity,
                                                        const Vector3 *angularVelocity,
                                                        bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_GetLinearVelocity(JPH_Character *character, Vector3 *result);
JPH_CAPI void JPH_Character_SetLinearVelocity(JPH_Character *character,
                                              const Vector3 *value,
                                              bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_AddLinearVelocity(JPH_Character *character,
                                              const Vector3 *value,
                                              bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_AddImpulse(JPH_Character *character, const Vector3 *value, bool lockBodies /* = true */);
JPH_CAPI JPH_BodyID JPH_Character_GetBodyID(const JPH_Character *character);

JPH_CAPI void JPH_Character_GetPositionAndRotation(JPH_Character *character,
                                                   JPH_RVec3 *position,
                                                   JPH_Quat *rotation,
                                                   bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_SetPositionAndRotation(JPH_Character *character,
                                                   const JPH_RVec3 *position,
                                                   const JPH_Quat *rotation,
                                                   JPH_Activation activationMode,
                                                   bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_GetPosition(JPH_Character *character, JPH_RVec3 *position, bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_SetPosition(JPH_Character *character,
                                        const JPH_RVec3 *position,
                                        JPH_Activation activationMode,
                                        bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_GetRotation(JPH_Character *character, JPH_Quat *rotation, bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_SetRotation(JPH_Character *character,
                                        const JPH_Quat *rotation,
                                        JPH_Activation activationMode,
                                        bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_GetCenterOfMassPosition(JPH_Character *character,
                                                    JPH_RVec3 *result,
                                                    bool lockBodies /* = true */);
JPH_CAPI void JPH_Character_GetWorldTransform(JPH_Character *character,
                                              JPH_RMat44 *result,
                                              bool lockBodies /* = true */);
JPH_CAPI JPH_ObjectLayer JPH_Character_GetLayer(const JPH_Character *character);
JPH_CAPI void JPH_Character_SetLayer(JPH_Character *character, JPH_ObjectLayer value, bool lockBodies /*= true*/);
JPH_CAPI bool JPH_Character_SetShape(JPH_Character *character,
                                     const JPH_Shape *shape,
                                     float maxPenetrationDepth,
                                     bool lockBodies /*= true*/);

/* CharacterVirtualSettings */
JPH_CAPI void JPH_CharacterVirtualSettings_Init(JPH_CharacterVirtualSettings *settings);

/* CharacterVirtual */
JPH_CAPI JPH_CharacterVirtual *JPH_CharacterVirtual_Create(const JPH_CharacterVirtualSettings *settings,
                                                           const JPH_RVec3 *position,
                                                           const JPH_Quat *rotation,
                                                           uint64_t userData,
                                                           JPH_PhysicsSystem *system);
JPH_CAPI void JPH_CharacterVirtual_Destroy(JPH_CharacterVirtual *characterVirtual);

JPH_CAPI JPH_CharacterID JPH_CharacterVirtual_GetID(const JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetListener(JPH_CharacterVirtual *character, JPH_CharacterContactListener *listener);
JPH_CAPI void JPH_CharacterVirtual_SetCharacterVsCharacterCollision(JPH_CharacterVirtual *character,
                                                                    JPH_CharacterVsCharacterCollision
                                                                            *characterVsCharacterCollision);

JPH_CAPI void JPH_CharacterVirtual_GetLinearVelocity(JPH_CharacterVirtual *character, Vector3 *velocity);
JPH_CAPI void JPH_CharacterVirtual_SetLinearVelocity(JPH_CharacterVirtual *character, const Vector3 *velocity);
JPH_CAPI void JPH_CharacterVirtual_GetPosition(JPH_CharacterVirtual *character, JPH_RVec3 *position);
JPH_CAPI void JPH_CharacterVirtual_SetPosition(JPH_CharacterVirtual *character, const JPH_RVec3 *position);
JPH_CAPI void JPH_CharacterVirtual_GetRotation(JPH_CharacterVirtual *character, JPH_Quat *rotation);
JPH_CAPI void JPH_CharacterVirtual_SetRotation(JPH_CharacterVirtual *character, const JPH_Quat *rotation);
JPH_CAPI void JPH_CharacterVirtual_GetWorldTransform(JPH_CharacterVirtual *character, JPH_RMat44 *result);
JPH_CAPI void JPH_CharacterVirtual_GetCenterOfMassTransform(JPH_CharacterVirtual *character, JPH_RMat44 *result);
JPH_CAPI float JPH_CharacterVirtual_GetMass(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetMass(JPH_CharacterVirtual *character, float value);
JPH_CAPI float JPH_CharacterVirtual_GetMaxStrength(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetMaxStrength(JPH_CharacterVirtual *character, float value);

JPH_CAPI float JPH_CharacterVirtual_GetPenetrationRecoverySpeed(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetPenetrationRecoverySpeed(JPH_CharacterVirtual *character, float value);
JPH_CAPI bool JPH_CharacterVirtual_GetEnhancedInternalEdgeRemoval(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetEnhancedInternalEdgeRemoval(JPH_CharacterVirtual *character, bool value);
JPH_CAPI float JPH_CharacterVirtual_GetCharacterPadding(JPH_CharacterVirtual *character);
JPH_CAPI uint32_t JPH_CharacterVirtual_GetMaxNumHits(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetMaxNumHits(JPH_CharacterVirtual *character, uint32_t value);
JPH_CAPI float JPH_CharacterVirtual_GetHitReductionCosMaxAngle(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetHitReductionCosMaxAngle(JPH_CharacterVirtual *character, float value);
JPH_CAPI bool JPH_CharacterVirtual_GetMaxHitsExceeded(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_GetShapeOffset(JPH_CharacterVirtual *character, Vector3 *result);
JPH_CAPI void JPH_CharacterVirtual_SetShapeOffset(JPH_CharacterVirtual *character, const Vector3 *value);
JPH_CAPI uint64_t JPH_CharacterVirtual_GetUserData(const JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_SetUserData(JPH_CharacterVirtual *character, uint64_t value);
JPH_CAPI JPH_BodyID JPH_CharacterVirtual_GetInnerBodyID(const JPH_CharacterVirtual *character);

JPH_CAPI void JPH_CharacterVirtual_CancelVelocityTowardsSteepSlopes(JPH_CharacterVirtual *character,
                                                                    const Vector3 *desiredVelocity,
                                                                    Vector3 *velocity);
JPH_CAPI void JPH_CharacterVirtual_StartTrackingContactChanges(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_FinishTrackingContactChanges(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_Update(JPH_CharacterVirtual *character,
                                          float deltaTime,
                                          JPH_ObjectLayer layer,
                                          const JPH_PhysicsSystem *system,
                                          const JPH_BodyFilter *bodyFilter,
                                          const JPH_ShapeFilter *shapeFilter);

JPH_CAPI void JPH_CharacterVirtual_ExtendedUpdate(JPH_CharacterVirtual *character,
                                                  float deltaTime,
                                                  const JPH_ExtendedUpdateSettings *settings,
                                                  JPH_ObjectLayer layer,
                                                  const JPH_PhysicsSystem *system,
                                                  const JPH_BodyFilter *bodyFilter,
                                                  const JPH_ShapeFilter *shapeFilter);
JPH_CAPI void JPH_CharacterVirtual_RefreshContacts(JPH_CharacterVirtual *character,
                                                   JPH_ObjectLayer layer,
                                                   const JPH_PhysicsSystem *system,
                                                   const JPH_BodyFilter *bodyFilter,
                                                   const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_CharacterVirtual_CanWalkStairs(JPH_CharacterVirtual *character, const Vector3 *linearVelocity);
JPH_CAPI bool JPH_CharacterVirtual_WalkStairs(JPH_CharacterVirtual *character,
                                              float deltaTime,
                                              const Vector3 *stepUp,
                                              const Vector3 *stepForward,
                                              const Vector3 *stepForwardTest,
                                              const Vector3 *stepDownExtra,
                                              JPH_ObjectLayer layer,
                                              const JPH_PhysicsSystem *system,
                                              const JPH_BodyFilter *bodyFilter,
                                              const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_CharacterVirtual_StickToFloor(JPH_CharacterVirtual *character,
                                                const Vector3 *stepDown,
                                                JPH_ObjectLayer layer,
                                                const JPH_PhysicsSystem *system,
                                                const JPH_BodyFilter *bodyFilter,
                                                const JPH_ShapeFilter *shapeFilter);

JPH_CAPI void JPH_CharacterVirtual_UpdateGroundVelocity(JPH_CharacterVirtual *character);
JPH_CAPI bool JPH_CharacterVirtual_SetShape(JPH_CharacterVirtual *character,
                                            const JPH_Shape *shape,
                                            float maxPenetrationDepth,
                                            JPH_ObjectLayer layer,
                                            const JPH_PhysicsSystem *system,
                                            const JPH_BodyFilter *bodyFilter,
                                            const JPH_ShapeFilter *shapeFilter);
JPH_CAPI void JPH_CharacterVirtual_SetInnerBodyShape(JPH_CharacterVirtual *character, const JPH_Shape *shape);

JPH_CAPI uint32_t JPH_CharacterVirtual_GetNumActiveContacts(JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVirtual_GetActiveContact(JPH_CharacterVirtual *character,
                                                    uint32_t index,
                                                    JPH_CharacterVirtualContact *result);

JPH_CAPI bool JPH_CharacterVirtual_HasCollidedWithBody(JPH_CharacterVirtual *character, JPH_BodyID body);
JPH_CAPI bool JPH_CharacterVirtual_HasCollidedWith(JPH_CharacterVirtual *character, JPH_CharacterID other);
JPH_CAPI bool JPH_CharacterVirtual_HasCollidedWithCharacter(JPH_CharacterVirtual *character,
                                                            const JPH_CharacterVirtual *other);

/* CharacterContactListener */
typedef struct JPH_CharacterContactListener_Impl
{
        void(JPH_API_CALL *OnAdjustBodyVelocity)(const JPH_CharacterVirtual *character,
                                                 const JPH_Body *body2,
                                                 Vector3 *ioLinearVelocity,
                                                 Vector3 *ioAngularVelocity);

        bool(JPH_API_CALL *OnContactValidate)(const JPH_CharacterVirtual *character,
                                              JPH_BodyID bodyID2,
                                              JPH_SubShapeID subShapeID2);

        bool(JPH_API_CALL *OnCharacterContactValidate)(const JPH_CharacterVirtual *character,
                                                       const JPH_CharacterVirtual *otherCharacter,
                                                       JPH_SubShapeID subShapeID2);

        void(JPH_API_CALL *OnContactAdded)(const JPH_CharacterVirtual *character,
                                           JPH_BodyID bodyID2,
                                           JPH_SubShapeID subShapeID2,
                                           const JPH_RVec3 *contactPosition,
                                           const Vector3 *contactNormal,
                                           JPH_CharacterContactSettings *ioSettings);

        void(JPH_API_CALL *OnContactPersisted)(const JPH_CharacterVirtual *character,
                                               JPH_BodyID bodyID2,
                                               JPH_SubShapeID subShapeID2,
                                               const JPH_RVec3 *contactPosition,
                                               const Vector3 *contactNormal,
                                               JPH_CharacterContactSettings *ioSettings);

        void(JPH_API_CALL *OnContactRemoved)(const JPH_CharacterVirtual *character,
                                             JPH_BodyID bodyID2,
                                             JPH_SubShapeID subShapeID2);

        void(JPH_API_CALL *OnCharacterContactAdded)(const JPH_CharacterVirtual *character,
                                                    const JPH_CharacterVirtual *otherCharacter,
                                                    JPH_SubShapeID subShapeID2,
                                                    const JPH_RVec3 *contactPosition,
                                                    const Vector3 *contactNormal,
                                                    JPH_CharacterContactSettings *ioSettings);

        void(JPH_API_CALL *OnCharacterContactPersisted)(const JPH_CharacterVirtual *character,
                                                        const JPH_CharacterVirtual *otherCharacter,
                                                        JPH_SubShapeID subShapeID2,
                                                        const JPH_RVec3 *contactPosition,
                                                        const Vector3 *contactNormal,
                                                        JPH_CharacterContactSettings *ioSettings);

        void(JPH_API_CALL *OnCharacterContactRemoved)(const JPH_CharacterVirtual *character,
                                                      JPH_CharacterID otherCharacterID,
                                                      JPH_SubShapeID subShapeID2);

        void(JPH_API_CALL *OnContactSolve)(const JPH_CharacterVirtual *character,
                                           JPH_BodyID bodyID2,
                                           JPH_SubShapeID subShapeID2,
                                           const JPH_RVec3 *contactPosition,
                                           const Vector3 *contactNormal,
                                           const Vector3 *contactVelocity,
                                           const JPH_PhysicsMaterial *contactMaterial,
                                           const Vector3 *characterVelocity,
                                           Vector3 *newCharacterVelocity);

        void(JPH_API_CALL *OnCharacterContactSolve)(const JPH_CharacterVirtual *character,
                                                    const JPH_CharacterVirtual *otherCharacter,
                                                    JPH_SubShapeID subShapeID2,
                                                    const JPH_RVec3 *contactPosition,
                                                    const Vector3 *contactNormal,
                                                    const Vector3 *contactVelocity,
                                                    const JPH_PhysicsMaterial *contactMaterial,
                                                    const Vector3 *characterVelocity,
                                                    Vector3 *newCharacterVelocity);
} JPH_CharacterContactListener_Impl;

JPH_CAPI JPH_CharacterContactListener *JPH_CharacterContactListener_Create(const JPH_CharacterContactListener_Impl
                                                                                   *impl);
JPH_CAPI void JPH_CharacterContactListener_Destroy(JPH_CharacterContactListener *listener);
JPH_CAPI void JPH_CharacterContactListener_SetImpl(JPH_CharacterContactListener *listener,
                                                   const JPH_CharacterContactListener_Impl *impl);

/* JPH_CharacterVsCharacterCollision */
typedef struct JPH_CharacterVsCharacterCollision_Impl
{
        void(JPH_API_CALL *CollideCharacter)(void *userData,
                                             const JPH_CharacterVirtual *character,
                                             const JPH_RMat44 *centerOfMassTransform,
                                             const JPH_CollideShapeSettings *collideShapeSettings,
                                             const JPH_RVec3 *baseOffset);

        void(JPH_API_CALL *CastCharacter)(void *userData,
                                          const JPH_CharacterVirtual *character,
                                          const JPH_RMat44 *centerOfMassTransform,
                                          const Vector3 *direction,
                                          const JPH_ShapeCastSettings *shapeCastSettings,
                                          const JPH_RVec3 *baseOffset);
} JPH_CharacterVsCharacterCollision_Impl;

JPH_CAPI void JPH_CharacterVsCharacterCollision_SetImpl(const JPH_CharacterVsCharacterCollision_Impl *impl);
JPH_CAPI JPH_CharacterVsCharacterCollision *JPH_CharacterVsCharacterCollision_Create(void *userData);
JPH_CAPI JPH_CharacterVsCharacterCollision *JPH_CharacterVsCharacterCollision_CreateSimple(void);
JPH_CAPI void JPH_CharacterVsCharacterCollisionSimple_AddCharacter(JPH_CharacterVsCharacterCollision
                                                                           *characterVsCharacter,
                                                                   JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter(JPH_CharacterVsCharacterCollision
                                                                              *characterVsCharacter,
                                                                      JPH_CharacterVirtual *character);
JPH_CAPI void JPH_CharacterVsCharacterCollision_Destroy(JPH_CharacterVsCharacterCollision *listener);

/* DebugRenderer */
typedef struct JPH_DebugRenderer_Impl
{
        void(JPH_API_CALL *DrawLine)(void *userData, const JPH_RVec3 *from, const JPH_RVec3 *to, JPH_Color color);
        void(JPH_API_CALL *DrawTriangle)(void *userData,
                                         const JPH_RVec3 *v1,
                                         const JPH_RVec3 *v2,
                                         const JPH_RVec3 *v3,
                                         JPH_Color color,
                                         JPH_DebugRenderer_CastShadow castShadow);
        void(JPH_API_CALL *DrawText3D)(void *userData,
                                       const JPH_RVec3 *position,
                                       const char *str,
                                       JPH_Color color,
                                       float height);
} JPH_DebugRenderer_Impl;

JPH_CAPI void JPH_DebugRenderer_SetImpl(const JPH_DebugRenderer_Impl *impl);
JPH_CAPI JPH_DebugRenderer *JPH_DebugRenderer_Create(void *userData);
JPH_CAPI void JPH_DebugRenderer_Destroy(JPH_DebugRenderer *renderer);
JPH_CAPI void JPH_DebugRenderer_NextFrame(JPH_DebugRenderer *renderer);
JPH_CAPI void JPH_DebugRenderer_SetCameraPos(JPH_DebugRenderer *renderer, const JPH_RVec3 *position);

JPH_CAPI void JPH_DebugRenderer_DrawLine(JPH_DebugRenderer *renderer,
                                         const JPH_RVec3 *from,
                                         const JPH_RVec3 *to,
                                         JPH_Color color);
JPH_CAPI void JPH_DebugRenderer_DrawWireBox(JPH_DebugRenderer *renderer, const JPH_AABox *box, JPH_Color color);
JPH_CAPI void JPH_DebugRenderer_DrawWireBox2(JPH_DebugRenderer *renderer,
                                             const JPH_RMat44 *matrix,
                                             const JPH_AABox *box,
                                             JPH_Color color);
JPH_CAPI void JPH_DebugRenderer_DrawMarker(JPH_DebugRenderer *renderer,
                                           const JPH_RVec3 *position,
                                           JPH_Color color,
                                           float size);
JPH_CAPI void JPH_DebugRenderer_DrawArrow(JPH_DebugRenderer *renderer,
                                          const JPH_RVec3 *from,
                                          const JPH_RVec3 *to,
                                          JPH_Color color,
                                          float size);
JPH_CAPI void JPH_DebugRenderer_DrawCoordinateSystem(JPH_DebugRenderer *renderer, const JPH_RMat44 *matrix, float size);
JPH_CAPI void JPH_DebugRenderer_DrawPlane(JPH_DebugRenderer *renderer,
                                          const JPH_RVec3 *point,
                                          const Vector3 *normal,
                                          JPH_Color color,
                                          float size);
JPH_CAPI void JPH_DebugRenderer_DrawWireTriangle(JPH_DebugRenderer *renderer,
                                                 const JPH_RVec3 *v1,
                                                 const JPH_RVec3 *v2,
                                                 const JPH_RVec3 *v3,
                                                 JPH_Color color);
JPH_CAPI void JPH_DebugRenderer_DrawWireSphere(JPH_DebugRenderer *renderer,
                                               const JPH_RVec3 *center,
                                               float radius,
                                               JPH_Color color,
                                               int level);
JPH_CAPI void JPH_DebugRenderer_DrawWireUnitSphere(JPH_DebugRenderer *renderer,
                                                   const JPH_RMat44 *matrix,
                                                   JPH_Color color,
                                                   int level);
JPH_CAPI void JPH_DebugRenderer_DrawTriangle(JPH_DebugRenderer *renderer,
                                             const JPH_RVec3 *v1,
                                             const JPH_RVec3 *v2,
                                             const JPH_RVec3 *v3,
                                             JPH_Color color,
                                             JPH_DebugRenderer_CastShadow castShadow);
JPH_CAPI void JPH_DebugRenderer_DrawBox(JPH_DebugRenderer *renderer,
                                        const JPH_AABox *box,
                                        JPH_Color color,
                                        JPH_DebugRenderer_CastShadow castShadow,
                                        JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawBox2(JPH_DebugRenderer *renderer,
                                         const JPH_RMat44 *matrix,
                                         const JPH_AABox *box,
                                         JPH_Color color,
                                         JPH_DebugRenderer_CastShadow castShadow,
                                         JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawSphere(JPH_DebugRenderer *renderer,
                                           const JPH_RVec3 *center,
                                           float radius,
                                           JPH_Color color,
                                           JPH_DebugRenderer_CastShadow castShadow,
                                           JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawUnitSphere(JPH_DebugRenderer *renderer,
                                               const JPH_RMat44 *matrix,
                                               JPH_Color color,
                                               JPH_DebugRenderer_CastShadow castShadow,
                                               JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawCapsule(JPH_DebugRenderer *renderer,
                                            const JPH_RMat44 *matrix,
                                            float halfHeightOfCylinder,
                                            float radius,
                                            JPH_Color color,
                                            JPH_DebugRenderer_CastShadow castShadow,
                                            JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawCylinder(JPH_DebugRenderer *renderer,
                                             const JPH_RMat44 *matrix,
                                             float halfHeight,
                                             float radius,
                                             JPH_Color color,
                                             JPH_DebugRenderer_CastShadow castShadow,
                                             JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawOpenCone(JPH_DebugRenderer *renderer,
                                             const JPH_RVec3 *top,
                                             const Vector3 *axis,
                                             const Vector3 *perpendicular,
                                             float halfAngle,
                                             float length,
                                             JPH_Color color,
                                             JPH_DebugRenderer_CastShadow castShadow,
                                             JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawSwingConeLimits(JPH_DebugRenderer *renderer,
                                                    const JPH_RMat44 *matrix,
                                                    float swingYHalfAngle,
                                                    float swingZHalfAngle,
                                                    float edgeLength,
                                                    JPH_Color color,
                                                    JPH_DebugRenderer_CastShadow castShadow,
                                                    JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawSwingPyramidLimits(JPH_DebugRenderer *renderer,
                                                       const JPH_RMat44 *matrix,
                                                       float minSwingYAngle,
                                                       float maxSwingYAngle,
                                                       float minSwingZAngle,
                                                       float maxSwingZAngle,
                                                       float edgeLength,
                                                       JPH_Color color,
                                                       JPH_DebugRenderer_CastShadow castShadow,
                                                       JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawPie(JPH_DebugRenderer *renderer,
                                        const JPH_RVec3 *center,
                                        float radius,
                                        const Vector3 *normal,
                                        const Vector3 *axis,
                                        float minAngle,
                                        float maxAngle,
                                        JPH_Color color,
                                        JPH_DebugRenderer_CastShadow castShadow,
                                        JPH_DebugRenderer_DrawMode drawMode);
JPH_CAPI void JPH_DebugRenderer_DrawTaperedCylinder(JPH_DebugRenderer *renderer,
                                                    const JPH_RMat44 *inMatrix,
                                                    float top,
                                                    float bottom,
                                                    float topRadius,
                                                    float bottomRadius,
                                                    JPH_Color color,
                                                    JPH_DebugRenderer_CastShadow castShadow,
                                                    JPH_DebugRenderer_DrawMode drawMode);


/* Skeleton */
typedef struct JPH_SkeletonJoint
{
        const char *name;
        const char *parentName;
        int parentJointIndex;
} JPH_SkeletonJoint;

JPH_CAPI JPH_Skeleton *JPH_Skeleton_Create(void);
JPH_CAPI void JPH_Skeleton_Destroy(JPH_Skeleton *skeleton);

JPH_CAPI uint32_t JPH_Skeleton_AddJoint(JPH_Skeleton *skeleton, const char *name);
JPH_CAPI uint32_t JPH_Skeleton_AddJoint2(JPH_Skeleton *skeleton, const char *name, int parentIndex);
JPH_CAPI uint32_t JPH_Skeleton_AddJoint3(JPH_Skeleton *skeleton, const char *name, const char *parentName);
JPH_CAPI int JPH_Skeleton_GetJointCount(const JPH_Skeleton *skeleton);
JPH_CAPI void JPH_Skeleton_GetJoint(const JPH_Skeleton *skeleton, int index, JPH_SkeletonJoint *joint);
JPH_CAPI int JPH_Skeleton_GetJointIndex(const JPH_Skeleton *skeleton, const char *name);
JPH_CAPI void JPH_Skeleton_CalculateParentJointIndices(JPH_Skeleton *skeleton);
JPH_CAPI bool JPH_Skeleton_AreJointsCorrectlyOrdered(const JPH_Skeleton *skeleton);

/* SkeletonPose */
JPH_CAPI JPH_SkeletonPose *JPH_SkeletonPose_Create(void);
JPH_CAPI void JPH_SkeletonPose_Destroy(JPH_SkeletonPose *pose);
JPH_CAPI void JPH_SkeletonPose_SetSkeleton(JPH_SkeletonPose *pose, const JPH_Skeleton *skeleton);
JPH_CAPI const JPH_Skeleton *JPH_SkeletonPose_GetSkeleton(const JPH_SkeletonPose *pose);
JPH_CAPI void JPH_SkeletonPose_SetRootOffset(JPH_SkeletonPose *pose, const JPH_RVec3 *offset);
JPH_CAPI void JPH_SkeletonPose_GetRootOffset(const JPH_SkeletonPose *pose, JPH_RVec3 *result);
JPH_CAPI size_t JPH_SkeletonPose_GetJointCount(const JPH_SkeletonPose *pose);
JPH_CAPI void JPH_SkeletonPose_GetJointState(const JPH_SkeletonPose *pose,
                                             int index,
                                             Vector3 *outTranslation,
                                             JPH_Quat *outRotation);
JPH_CAPI void JPH_SkeletonPose_SetJointState(JPH_SkeletonPose *pose,
                                             int index,
                                             const Vector3 *translation,
                                             const JPH_Quat *rotation);
JPH_CAPI void JPH_SkeletonPose_GetJointMatrix(const JPH_SkeletonPose *pose, int index, JPH_Mat44 *result);
JPH_CAPI void JPH_SkeletonPose_SetJointMatrix(JPH_SkeletonPose *pose, int index, const JPH_Mat44 *matrix);
JPH_CAPI void JPH_SkeletonPose_GetJointMatrices(const JPH_SkeletonPose *pose, JPH_Mat44 *outMatrices, size_t count);
JPH_CAPI void JPH_SkeletonPose_SetJointMatrices(JPH_SkeletonPose *pose, const JPH_Mat44 *matrices, size_t count);
JPH_CAPI void JPH_SkeletonPose_CalculateJointMatrices(JPH_SkeletonPose *pose);
JPH_CAPI void JPH_SkeletonPose_CalculateJointStates(JPH_SkeletonPose *pose);
JPH_CAPI void JPH_SkeletonPose_CalculateLocalSpaceJointMatrices(const JPH_SkeletonPose *pose, JPH_Mat44 *outMatrices);

/* SkeletalAnimation */
JPH_CAPI JPH_SkeletalAnimation *JPH_SkeletalAnimation_Create(void);
JPH_CAPI void JPH_SkeletalAnimation_Destroy(JPH_SkeletalAnimation *animation);
JPH_CAPI float JPH_SkeletalAnimation_GetDuration(const JPH_SkeletalAnimation *animation);
JPH_CAPI bool JPH_SkeletalAnimation_IsLooping(const JPH_SkeletalAnimation *animation);
JPH_CAPI void JPH_SkeletalAnimation_SetIsLooping(JPH_SkeletalAnimation *animation, bool looping);
JPH_CAPI void JPH_SkeletalAnimation_ScaleJoints(JPH_SkeletalAnimation *animation, float scale);
JPH_CAPI void JPH_SkeletalAnimation_Sample(const JPH_SkeletalAnimation *animation, float time, JPH_SkeletonPose *pose);
JPH_CAPI size_t JPH_SkeletalAnimation_GetAnimatedJointCount(const JPH_SkeletalAnimation *animation);
JPH_CAPI void JPH_SkeletalAnimation_AddAnimatedJoint(JPH_SkeletalAnimation *animation, const char *jointName);
JPH_CAPI void JPH_SkeletalAnimation_AddKeyframe(JPH_SkeletalAnimation *animation,
                                                size_t jointIndex,
                                                float time,
                                                const Vector3 *translation,
                                                const JPH_Quat *rotation);

/* SkeletonMapper */
JPH_CAPI JPH_SkeletonMapper *JPH_SkeletonMapper_Create(void);
JPH_CAPI void JPH_SkeletonMapper_Destroy(JPH_SkeletonMapper *mapper);
JPH_CAPI void JPH_SkeletonMapper_Initialize(JPH_SkeletonMapper *mapper,
                                            const JPH_Skeleton *skeleton1,
                                            const JPH_Mat44 *neutralPose1,
                                            const JPH_Skeleton *skeleton2,
                                            const JPH_Mat44 *neutralPose2);
JPH_CAPI void JPH_SkeletonMapper_LockAllTranslations(JPH_SkeletonMapper *mapper,
                                                     const JPH_Skeleton *skeleton2,
                                                     const JPH_Mat44 *neutralPose2);
JPH_CAPI void JPH_SkeletonMapper_LockTranslations(JPH_SkeletonMapper *mapper,
                                                  const JPH_Skeleton *skeleton2,
                                                  const bool *lockedTranslations,
                                                  const JPH_Mat44 *neutralPose2);
JPH_CAPI void JPH_SkeletonMapper_Map(const JPH_SkeletonMapper *mapper,
                                     const JPH_Mat44 *pose1ModelSpace,
                                     const JPH_Mat44 *pose2LocalSpace,
                                     JPH_Mat44 *outPose2ModelSpace);
JPH_CAPI void JPH_SkeletonMapper_MapReverse(const JPH_SkeletonMapper *mapper,
                                            const JPH_Mat44 *pose2ModelSpace,
                                            JPH_Mat44 *outPose1ModelSpace);
JPH_CAPI int JPH_SkeletonMapper_GetMappedJointIndex(const JPH_SkeletonMapper *mapper, int joint1Index);
JPH_CAPI bool JPH_SkeletonMapper_IsJointTranslationLocked(const JPH_SkeletonMapper *mapper, int joint2Index);

/* RagdollSettings */
JPH_CAPI JPH_RagdollSettings *JPH_RagdollSettings_Create(void);
JPH_CAPI void JPH_RagdollSettings_Destroy(JPH_RagdollSettings *settings);

JPH_CAPI const JPH_Skeleton *JPH_RagdollSettings_GetSkeleton(const JPH_RagdollSettings *character);
JPH_CAPI void JPH_RagdollSettings_SetSkeleton(JPH_RagdollSettings *character, JPH_Skeleton *skeleton);
JPH_CAPI bool JPH_RagdollSettings_Stabilize(JPH_RagdollSettings *settings);
JPH_CAPI void JPH_RagdollSettings_DisableParentChildCollisions(JPH_RagdollSettings *settings,
                                                               const JPH_Mat44 *jointMatrices /*=nullptr*/,
                                                               float minSeparationDistance /* = 0.0f*/);
JPH_CAPI void JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex(JPH_RagdollSettings *settings);
JPH_CAPI int JPH_RagdollSettings_GetConstraintIndexForBodyIndex(JPH_RagdollSettings *settings, int bodyIndex);
JPH_CAPI void JPH_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(JPH_RagdollSettings *settings);

JPH_CAPI JPH_Ragdoll *JPH_RagdollSettings_CreateRagdoll(JPH_RagdollSettings *settings,
                                                        const JPH_PhysicsSystem *system,
                                                        JPH_CollisionGroupID collisionGroup /*=0*/,
                                                        uint64_t userData /* = 0*/);

/* Ragdoll */
JPH_CAPI void JPH_Ragdoll_Destroy(JPH_Ragdoll *ragdoll);
JPH_CAPI void JPH_Ragdoll_AddToPhysicsSystem(JPH_Ragdoll *ragdoll,
                                             JPH_Activation activationMode /*= JPH_ActivationActivate */,
                                             bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_RemoveFromPhysicsSystem(JPH_Ragdoll *ragdoll, bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_Activate(JPH_Ragdoll *ragdoll, bool lockBodies /* = true */);
JPH_CAPI bool JPH_Ragdoll_IsActive(const JPH_Ragdoll *ragdoll, bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_ResetWarmStart(JPH_Ragdoll *ragdoll);
JPH_CAPI void JPH_Ragdoll_SetPose(JPH_Ragdoll *ragdoll, const JPH_SkeletonPose *pose, bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_SetPose2(JPH_Ragdoll *ragdoll,
                                   const JPH_RVec3 *rootOffset,
                                   const JPH_Mat44 *jointMatrices,
                                   bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_GetPose(JPH_Ragdoll *ragdoll, JPH_SkeletonPose *outPose, bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_GetPose2(JPH_Ragdoll *ragdoll,
                                   JPH_RVec3 *outRootOffset,
                                   JPH_Mat44 *outJointMatrices,
                                   bool lockBodies /* = true */);
JPH_CAPI void JPH_Ragdoll_DriveToPoseUsingMotors(JPH_Ragdoll *ragdoll, const JPH_SkeletonPose *pose);
JPH_CAPI void JPH_Ragdoll_DriveToPoseUsingKinematics(JPH_Ragdoll *ragdoll,
                                                     const JPH_SkeletonPose *pose,
                                                     float deltaTime,
                                                     bool lockBodies /* = true */);
JPH_CAPI size_t JPH_Ragdoll_GetBodyCount(const JPH_Ragdoll *ragdoll);
JPH_CAPI JPH_BodyID JPH_Ragdoll_GetBodyID(const JPH_Ragdoll *ragdoll, int bodyIndex);
JPH_CAPI size_t JPH_Ragdoll_GetConstraintCount(const JPH_Ragdoll *ragdoll);
JPH_CAPI JPH_TwoBodyConstraint *JPH_Ragdoll_GetConstraint(JPH_Ragdoll *ragdoll, int constraintIndex);
JPH_CAPI void JPH_Ragdoll_GetRootTransform(const JPH_Ragdoll *ragdoll,
                                           JPH_RVec3 *outPosition,
                                           JPH_Quat *outRotation,
                                           bool lockBodies /* = true */);
JPH_CAPI const JPH_RagdollSettings *JPH_Ragdoll_GetRagdollSettings(const JPH_Ragdoll *ragdoll);

/* JPH_EstimateCollisionResponse */
JPH_CAPI void JPH_EstimateCollisionResponse(const JPH_Body *body1,
                                            const JPH_Body *body2,
                                            const JPH_ContactManifold *manifold,
                                            float combinedFriction,
                                            float combinedRestitution,
                                            float minVelocityForRestitution,
                                            uint32_t numIterations,
                                            JPH_CollisionEstimationResult *result);

/* Vehicle */
typedef struct JPH_WheelSettings JPH_WheelSettings;
typedef struct JPH_WheelSettingsWV JPH_WheelSettingsWV; /* Inherits JPH_WheelSettings */
typedef struct JPH_WheelSettingsTV JPH_WheelSettingsTV; /* Inherits JPH_WheelSettings */

typedef struct JPH_Wheel JPH_Wheel;
typedef struct JPH_WheelWV JPH_WheelWV; /* Inherits JPH_Wheel */
typedef struct JPH_WheelTV JPH_WheelTV; /* Inherits JPH_Wheel */

typedef struct JPH_VehicleEngine JPH_VehicleEngine;
typedef struct JPH_VehicleTransmission JPH_VehicleTransmission;
typedef struct JPH_VehicleTransmissionSettings JPH_VehicleTransmissionSettings;
typedef struct JPH_VehicleCollisionTester JPH_VehicleCollisionTester;
typedef struct JPH_VehicleCollisionTesterRay JPH_VehicleCollisionTesterRay; /* Inherits JPH_VehicleCollisionTester */
typedef struct JPH_VehicleCollisionTesterCastSphere
        JPH_VehicleCollisionTesterCastSphere; /* Inherits JPH_VehicleCollisionTester */
typedef struct JPH_VehicleCollisionTesterCastCylinder
        JPH_VehicleCollisionTesterCastCylinder; /* Inherits JPH_VehicleCollisionTester */
typedef struct JPH_VehicleConstraint JPH_VehicleConstraint; /* Inherits JPH_Constraint */

typedef struct JPH_VehicleControllerSettings JPH_VehicleControllerSettings;
typedef struct JPH_WheeledVehicleControllerSettings
        JPH_WheeledVehicleControllerSettings; /* Inherits JPH_VehicleControllerSettings */
typedef struct JPH_MotorcycleControllerSettings
        JPH_MotorcycleControllerSettings; /* Inherits JPH_WheeledVehicleControllerSettings */
typedef struct JPH_TrackedVehicleControllerSettings
        JPH_TrackedVehicleControllerSettings; /* Inherits JPH_VehicleControllerSettings */

typedef struct JPH_WheeledVehicleController JPH_WheeledVehicleController; /* Inherits JPH_VehicleController */
typedef struct JPH_MotorcycleController JPH_MotorcycleController; /* Inherits JPH_WheeledVehicleController */
typedef struct JPH_TrackedVehicleController JPH_TrackedVehicleController; /* Inherits JPH_VehicleController */

typedef struct JPH_VehicleController JPH_VehicleController;

typedef struct JPH_VehicleAntiRollBar
{
        int leftWheel;
        int rightWheel;
        float stiffness;
} JPH_VehicleAntiRollBar;

typedef struct JPH_VehicleConstraintSettings
{
        JPH_ConstraintSettings base; /* Inherits JPH_ConstraintSettings */

        Vector3 up;
        Vector3 forward;
        float maxPitchRollAngle;
        uint32_t wheelsCount;
        JPH_WheelSettings **wheels;
        uint32_t antiRollBarsCount;
        const JPH_VehicleAntiRollBar *antiRollBars;
        JPH_VehicleControllerSettings *controller;
} JPH_VehicleConstraintSettings;

typedef struct JPH_VehicleEngineSettings
{
        float maxTorque;
        float minRPM;
        float maxRPM;
        const JPH_LinearCurve *normalizedTorque;
        float inertia;
        float angularDamping;
} JPH_VehicleEngineSettings;

typedef struct JPH_VehicleDifferentialSettings
{
        int leftWheel;
        int rightWheel;
        float differentialRatio;
        float leftRightSplit;
        float limitedSlipRatio;
        float engineTorqueRatio;
} JPH_VehicleDifferentialSettings;

JPH_CAPI void JPH_VehicleConstraintSettings_Init(JPH_VehicleConstraintSettings *settings);

JPH_CAPI JPH_VehicleConstraint *JPH_VehicleConstraint_Create(JPH_Body *body,
                                                             const JPH_VehicleConstraintSettings *settings);
JPH_CAPI JPH_PhysicsStepListener *JPH_VehicleConstraint_AsPhysicsStepListener(JPH_VehicleConstraint *constraint);

JPH_CAPI void JPH_VehicleConstraint_SetMaxPitchRollAngle(JPH_VehicleConstraint *constraint, float maxPitchRollAngle);
JPH_CAPI void JPH_VehicleConstraint_SetVehicleCollisionTester(JPH_VehicleConstraint *constraint,
                                                              const JPH_VehicleCollisionTester *tester);

JPH_CAPI void JPH_VehicleConstraint_OverrideGravity(JPH_VehicleConstraint *constraint, const Vector3 *value);
JPH_CAPI bool JPH_VehicleConstraint_IsGravityOverridden(const JPH_VehicleConstraint *constraint);
JPH_CAPI void JPH_VehicleConstraint_GetGravityOverride(const JPH_VehicleConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_VehicleConstraint_ResetGravityOverride(JPH_VehicleConstraint *constraint);

JPH_CAPI void JPH_VehicleConstraint_GetLocalForward(const JPH_VehicleConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_VehicleConstraint_GetLocalUp(const JPH_VehicleConstraint *constraint, Vector3 *result);
JPH_CAPI void JPH_VehicleConstraint_GetWorldUp(const JPH_VehicleConstraint *constraint, Vector3 *result);

JPH_CAPI const JPH_Body *JPH_VehicleConstraint_GetVehicleBody(const JPH_VehicleConstraint *constraint);
JPH_CAPI JPH_VehicleController *JPH_VehicleConstraint_GetController(JPH_VehicleConstraint *constraint);
JPH_CAPI uint32_t JPH_VehicleConstraint_GetWheelsCount(JPH_VehicleConstraint *constraint);
JPH_CAPI JPH_Wheel *JPH_VehicleConstraint_GetWheel(JPH_VehicleConstraint *constraint, uint32_t index);
JPH_CAPI void JPH_VehicleConstraint_GetWheelLocalBasis(JPH_VehicleConstraint *constraint,
                                                       const JPH_Wheel *wheel,
                                                       Vector3 *outForward,
                                                       Vector3 *outUp,
                                                       Vector3 *outRight);
JPH_CAPI void JPH_VehicleConstraint_GetWheelLocalTransform(JPH_VehicleConstraint *constraint,
                                                           uint32_t wheelIndex,
                                                           const Vector3 *wheelRight,
                                                           const Vector3 *wheelUp,
                                                           JPH_Mat44 *result);
JPH_CAPI void JPH_VehicleConstraint_GetWheelWorldTransform(JPH_VehicleConstraint *constraint,
                                                           uint32_t wheelIndex,
                                                           const Vector3 *wheelRight,
                                                           const Vector3 *wheelUp,
                                                           JPH_RMat44 *result);

/* Wheel */
JPH_CAPI JPH_WheelSettings *JPH_WheelSettings_Create(void);
JPH_CAPI void JPH_WheelSettings_Destroy(JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_GetPosition(const JPH_WheelSettings *settings, Vector3 *result);
JPH_CAPI void JPH_WheelSettings_SetPosition(JPH_WheelSettings *settings, const Vector3 *value);
JPH_CAPI void JPH_WheelSettings_GetSuspensionForcePoint(const JPH_WheelSettings *settings, Vector3 *result);
JPH_CAPI void JPH_WheelSettings_SetSuspensionForcePoint(JPH_WheelSettings *settings, const Vector3 *value);
JPH_CAPI void JPH_WheelSettings_GetSuspensionDirection(const JPH_WheelSettings *settings, Vector3 *result);
JPH_CAPI void JPH_WheelSettings_SetSuspensionDirection(JPH_WheelSettings *settings, const Vector3 *value);
JPH_CAPI void JPH_WheelSettings_GetSteeringAxis(const JPH_WheelSettings *settings, Vector3 *result);
JPH_CAPI void JPH_WheelSettings_SetSteeringAxis(JPH_WheelSettings *settings, const Vector3 *value);
JPH_CAPI void JPH_WheelSettings_GetWheelUp(const JPH_WheelSettings *settings, Vector3 *result);
JPH_CAPI void JPH_WheelSettings_SetWheelUp(JPH_WheelSettings *settings, const Vector3 *value);
JPH_CAPI void JPH_WheelSettings_GetWheelForward(const JPH_WheelSettings *settings, Vector3 *result);
JPH_CAPI void JPH_WheelSettings_SetWheelForward(JPH_WheelSettings *settings, const Vector3 *value);
JPH_CAPI float JPH_WheelSettings_GetSuspensionMinLength(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_SetSuspensionMinLength(JPH_WheelSettings *settings, float value);
JPH_CAPI float JPH_WheelSettings_GetSuspensionMaxLength(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_SetSuspensionMaxLength(JPH_WheelSettings *settings, float value);
JPH_CAPI float JPH_WheelSettings_GetSuspensionPreloadLength(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_SetSuspensionPreloadLength(JPH_WheelSettings *settings, float value);
JPH_CAPI void JPH_WheelSettings_GetSuspensionSpring(const JPH_WheelSettings *settings, JPH_SpringSettings *result);
JPH_CAPI void JPH_WheelSettings_SetSuspensionSpring(JPH_WheelSettings *settings,
                                                    const JPH_SpringSettings *springSettings);
JPH_CAPI float JPH_WheelSettings_GetRadius(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_SetRadius(JPH_WheelSettings *settings, float value);
JPH_CAPI float JPH_WheelSettings_GetWidth(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_SetWidth(JPH_WheelSettings *settings, float value);
JPH_CAPI bool JPH_WheelSettings_GetEnableSuspensionForcePoint(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_WheelSettings_SetEnableSuspensionForcePoint(JPH_WheelSettings *settings, bool value);

JPH_CAPI JPH_Wheel *JPH_Wheel_Create(const JPH_WheelSettings *settings);
JPH_CAPI void JPH_Wheel_Destroy(JPH_Wheel *wheel);
JPH_CAPI const JPH_WheelSettings *JPH_Wheel_GetSettings(const JPH_Wheel *wheel);
JPH_CAPI float JPH_Wheel_GetAngularVelocity(const JPH_Wheel *wheel);
JPH_CAPI void JPH_Wheel_SetAngularVelocity(JPH_Wheel *wheel, float value);
JPH_CAPI float JPH_Wheel_GetRotationAngle(const JPH_Wheel *wheel);
JPH_CAPI void JPH_Wheel_SetRotationAngle(JPH_Wheel *wheel, float value);
JPH_CAPI float JPH_Wheel_GetSteerAngle(const JPH_Wheel *wheel);
JPH_CAPI void JPH_Wheel_SetSteerAngle(JPH_Wheel *wheel, float value);
JPH_CAPI bool JPH_Wheel_HasContact(const JPH_Wheel *wheel);
JPH_CAPI JPH_BodyID JPH_Wheel_GetContactBodyID(const JPH_Wheel *wheel);
JPH_CAPI JPH_SubShapeID JPH_Wheel_GetContactSubShapeID(const JPH_Wheel *wheel);
JPH_CAPI void JPH_Wheel_GetContactPosition(const JPH_Wheel *wheel, JPH_RVec3 *result);
JPH_CAPI void JPH_Wheel_GetContactPointVelocity(const JPH_Wheel *wheel, Vector3 *result);
JPH_CAPI void JPH_Wheel_GetContactNormal(const JPH_Wheel *wheel, Vector3 *result);
JPH_CAPI void JPH_Wheel_GetContactLongitudinal(const JPH_Wheel *wheel, Vector3 *result);
JPH_CAPI void JPH_Wheel_GetContactLateral(const JPH_Wheel *wheel, Vector3 *result);
JPH_CAPI float JPH_Wheel_GetSuspensionLength(const JPH_Wheel *wheel);
JPH_CAPI float JPH_Wheel_GetSuspensionLambda(const JPH_Wheel *wheel);
JPH_CAPI float JPH_Wheel_GetLongitudinalLambda(const JPH_Wheel *wheel);
JPH_CAPI float JPH_Wheel_GetLateralLambda(const JPH_Wheel *wheel);
JPH_CAPI bool JPH_Wheel_HasHitHardPoint(const JPH_Wheel *wheel);

/* VehicleAntiRollBar */
JPH_CAPI void JPH_VehicleAntiRollBar_Init(JPH_VehicleAntiRollBar *antiRollBar);

/* VehicleEngineSettings */
JPH_CAPI void JPH_VehicleEngineSettings_Init(JPH_VehicleEngineSettings *settings);

/* VehicleEngine */
JPH_CAPI void JPH_VehicleEngine_ClampRPM(JPH_VehicleEngine *engine);
JPH_CAPI float JPH_VehicleEngine_GetCurrentRPM(const JPH_VehicleEngine *engine);
JPH_CAPI void JPH_VehicleEngine_SetCurrentRPM(JPH_VehicleEngine *engine, float rpm);
JPH_CAPI float JPH_VehicleEngine_GetAngularVelocity(const JPH_VehicleEngine *engine);
JPH_CAPI float JPH_VehicleEngine_GetTorque(const JPH_VehicleEngine *engine, float acceleration);
JPH_CAPI void JPH_VehicleEngine_ApplyTorque(JPH_VehicleEngine *engine, float torque, float deltaTime);
JPH_CAPI void JPH_VehicleEngine_ApplyDamping(JPH_VehicleEngine *engine, float deltaTime);
JPH_CAPI bool JPH_VehicleEngine_AllowSleep(const JPH_VehicleEngine *engine);

/* VehicleDifferentialSettings */
JPH_CAPI void JPH_VehicleDifferentialSettings_Init(JPH_VehicleDifferentialSettings *settings);

/* VehicleTransmissionSettings */
JPH_CAPI JPH_VehicleTransmissionSettings *JPH_VehicleTransmissionSettings_Create(void);
JPH_CAPI void JPH_VehicleTransmissionSettings_Destroy(JPH_VehicleTransmissionSettings *settings);

JPH_CAPI JPH_TransmissionMode JPH_VehicleTransmissionSettings_GetMode(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetMode(JPH_VehicleTransmissionSettings *settings,
                                                      JPH_TransmissionMode value);

JPH_CAPI uint32_t JPH_VehicleTransmissionSettings_GetGearRatioCount(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetGearRatio(const JPH_VehicleTransmissionSettings *settings,
                                                            uint32_t index);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetGearRatio(JPH_VehicleTransmissionSettings *settings,
                                                           uint32_t index,
                                                           float value);
JPH_CAPI const float *JPH_VehicleTransmissionSettings_GetGearRatios(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetGearRatios(JPH_VehicleTransmissionSettings *settings,
                                                            const float *values,
                                                            uint32_t count);

JPH_CAPI uint32_t
JPH_VehicleTransmissionSettings_GetReverseGearRatioCount(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetReverseGearRatio(const JPH_VehicleTransmissionSettings *settings,
                                                                   uint32_t index);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetReverseGearRatio(JPH_VehicleTransmissionSettings *settings,
                                                                  uint32_t index,
                                                                  float value);
JPH_CAPI const float *JPH_VehicleTransmissionSettings_GetReverseGearRatios(const JPH_VehicleTransmissionSettings
                                                                                   *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetReverseGearRatios(JPH_VehicleTransmissionSettings *settings,
                                                                   const float *values,
                                                                   uint32_t count);

JPH_CAPI float JPH_VehicleTransmissionSettings_GetSwitchTime(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetSwitchTime(JPH_VehicleTransmissionSettings *settings, float value);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetClutchReleaseTime(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetClutchReleaseTime(JPH_VehicleTransmissionSettings *settings,
                                                                   float value);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetSwitchLatency(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetSwitchLatency(JPH_VehicleTransmissionSettings *settings, float value);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetShiftUpRPM(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetShiftUpRPM(JPH_VehicleTransmissionSettings *settings, float value);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetShiftDownRPM(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetShiftDownRPM(JPH_VehicleTransmissionSettings *settings, float value);
JPH_CAPI float JPH_VehicleTransmissionSettings_GetClutchStrength(const JPH_VehicleTransmissionSettings *settings);
JPH_CAPI void JPH_VehicleTransmissionSettings_SetClutchStrength(JPH_VehicleTransmissionSettings *settings, float value);

/* VehicleTransmission */
JPH_CAPI void JPH_VehicleTransmission_Set(JPH_VehicleTransmission *transmission, int currentGear, float clutchFriction);
JPH_CAPI void JPH_VehicleTransmission_Update(JPH_VehicleTransmission *transmission,
                                             float deltaTime,
                                             float currentRPM,
                                             float forwardInput,
                                             bool canShiftUp);
JPH_CAPI int JPH_VehicleTransmission_GetCurrentGear(const JPH_VehicleTransmission *transmission);
JPH_CAPI float JPH_VehicleTransmission_GetClutchFriction(const JPH_VehicleTransmission *transmission);
JPH_CAPI bool JPH_VehicleTransmission_IsSwitchingGear(const JPH_VehicleTransmission *transmission);
JPH_CAPI float JPH_VehicleTransmission_GetCurrentRatio(const JPH_VehicleTransmission *transmission);
JPH_CAPI bool JPH_VehicleTransmission_AllowSleep(const JPH_VehicleTransmission *transmission);

/* VehicleCollisionTester */
JPH_CAPI void JPH_VehicleCollisionTester_Destroy(const JPH_VehicleCollisionTester *tester);
JPH_CAPI JPH_ObjectLayer JPH_VehicleCollisionTester_GetObjectLayer(const JPH_VehicleCollisionTester *tester);
JPH_CAPI void JPH_VehicleCollisionTester_SetObjectLayer(JPH_VehicleCollisionTester *tester, JPH_ObjectLayer value);

JPH_CAPI JPH_VehicleCollisionTesterRay *JPH_VehicleCollisionTesterRay_Create(JPH_ObjectLayer layer,
                                                                             const Vector3 *up,
                                                                             float maxSlopeAngle);
JPH_CAPI JPH_VehicleCollisionTesterCastSphere *JPH_VehicleCollisionTesterCastSphere_Create(JPH_ObjectLayer layer,
                                                                                           float radius,
                                                                                           const Vector3 *up,
                                                                                           float maxSlopeAngle);
JPH_CAPI JPH_VehicleCollisionTesterCastCylinder *JPH_VehicleCollisionTesterCastCylinder_Create(
        JPH_ObjectLayer layer,
        float convexRadiusFraction);

/* VehicleControllerSettings/VehicleController */
JPH_CAPI void JPH_VehicleControllerSettings_Destroy(JPH_VehicleControllerSettings *settings);
JPH_CAPI const JPH_VehicleConstraint *JPH_VehicleController_GetConstraint(JPH_VehicleController *controller);

/* ---- WheelSettingsWV - WheelWV - WheeledVehicleController ---- */

JPH_CAPI JPH_WheelSettingsWV *JPH_WheelSettingsWV_Create(void);
JPH_CAPI float JPH_WheelSettingsWV_GetInertia(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetInertia(JPH_WheelSettingsWV *settings, float value);
JPH_CAPI float JPH_WheelSettingsWV_GetAngularDamping(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetAngularDamping(JPH_WheelSettingsWV *settings, float value);
JPH_CAPI float JPH_WheelSettingsWV_GetMaxSteerAngle(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetMaxSteerAngle(JPH_WheelSettingsWV *settings, float value);
JPH_CAPI const JPH_LinearCurve *JPH_WheelSettingsWV_GetLongitudinalFriction(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetLongitudinalFriction(JPH_WheelSettingsWV *settings, const JPH_LinearCurve *value);
JPH_CAPI const JPH_LinearCurve *JPH_WheelSettingsWV_GetLateralFriction(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetLateralFriction(JPH_WheelSettingsWV *settings, const JPH_LinearCurve *value);
JPH_CAPI float JPH_WheelSettingsWV_GetMaxBrakeTorque(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetMaxBrakeTorque(JPH_WheelSettingsWV *settings, float value);
JPH_CAPI float JPH_WheelSettingsWV_GetMaxHandBrakeTorque(const JPH_WheelSettingsWV *settings);
JPH_CAPI void JPH_WheelSettingsWV_SetMaxHandBrakeTorque(JPH_WheelSettingsWV *settings, float value);

JPH_CAPI JPH_WheelWV *JPH_WheelWV_Create(const JPH_WheelSettingsWV *settings);
JPH_CAPI const JPH_WheelSettingsWV *JPH_WheelWV_GetSettings(const JPH_WheelWV *wheel);
JPH_CAPI void JPH_WheelWV_ApplyTorque(JPH_WheelWV *wheel, float torque, float deltaTime);

JPH_CAPI JPH_WheeledVehicleControllerSettings *JPH_WheeledVehicleControllerSettings_Create(void);

JPH_CAPI void JPH_WheeledVehicleControllerSettings_GetEngine(const JPH_WheeledVehicleControllerSettings *settings,
                                                             JPH_VehicleEngineSettings *result);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_SetEngine(JPH_WheeledVehicleControllerSettings *settings,
                                                             const JPH_VehicleEngineSettings *value);
JPH_CAPI const JPH_VehicleTransmissionSettings *JPH_WheeledVehicleControllerSettings_GetTransmission(
        const JPH_WheeledVehicleControllerSettings *settings);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_SetTransmission(JPH_WheeledVehicleControllerSettings *settings,
                                                                   const JPH_VehicleTransmissionSettings *value);

JPH_CAPI uint32_t
JPH_WheeledVehicleControllerSettings_GetDifferentialsCount(const JPH_WheeledVehicleControllerSettings *settings);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_SetDifferentialsCount(JPH_WheeledVehicleControllerSettings *settings,
                                                                         uint32_t count);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_GetDifferential(const JPH_WheeledVehicleControllerSettings *settings,
                                                                   uint32_t index,
                                                                   JPH_VehicleDifferentialSettings *result);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_SetDifferential(JPH_WheeledVehicleControllerSettings *settings,
                                                                   uint32_t index,
                                                                   const JPH_VehicleDifferentialSettings *value);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_SetDifferentials(JPH_WheeledVehicleControllerSettings *settings,
                                                                    const JPH_VehicleDifferentialSettings *values,
                                                                    uint32_t count);

JPH_CAPI float JPH_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(
        const JPH_WheeledVehicleControllerSettings *settings);
JPH_CAPI void JPH_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(JPH_WheeledVehicleControllerSettings
                                                                                           *settings,
                                                                                   float value);

JPH_CAPI void JPH_WheeledVehicleController_SetDriverInput(JPH_WheeledVehicleController *controller,
                                                          float forward,
                                                          float right,
                                                          float brake,
                                                          float handBrake);
JPH_CAPI void JPH_WheeledVehicleController_SetForwardInput(JPH_WheeledVehicleController *controller, float forward);
JPH_CAPI float JPH_WheeledVehicleController_GetForwardInput(const JPH_WheeledVehicleController *controller);
JPH_CAPI void JPH_WheeledVehicleController_SetRightInput(JPH_WheeledVehicleController *controller, float right);
JPH_CAPI float JPH_WheeledVehicleController_GetRightInput(const JPH_WheeledVehicleController *controller);
JPH_CAPI void JPH_WheeledVehicleController_SetBrakeInput(JPH_WheeledVehicleController *controller, float brakeInput);
JPH_CAPI float JPH_WheeledVehicleController_GetBrakeInput(const JPH_WheeledVehicleController *controller);
JPH_CAPI void JPH_WheeledVehicleController_SetHandBrakeInput(JPH_WheeledVehicleController *controller,
                                                             float handBrakeInput);
JPH_CAPI float JPH_WheeledVehicleController_GetHandBrakeInput(const JPH_WheeledVehicleController *controller);
JPH_CAPI float JPH_WheeledVehicleController_GetWheelSpeedAtClutch(const JPH_WheeledVehicleController *controller);
JPH_CAPI void JPH_WheeledVehicleController_SetTireMaxImpulseCallback(JPH_WheeledVehicleController *controller,
                                                                     JPH_TireMaxImpulseCallback tireMaxImpulseCallback);
JPH_CAPI const JPH_VehicleEngine *JPH_WheeledVehicleController_GetEngine(const JPH_WheeledVehicleController
                                                                                 *controller);
JPH_CAPI const JPH_VehicleTransmission *JPH_WheeledVehicleController_GetTransmission(const JPH_WheeledVehicleController
                                                                                             *controller);

/* VehicleTrack */
typedef struct JPH_VehicleTrackSettings JPH_VehicleTrackSettings;
typedef struct JPH_VehicleTrack JPH_VehicleTrack;

typedef struct JPH_VehicleTrackSettings
{
        uint32_t drivenWheel;
        const uint32_t *wheels;
        uint32_t wheelsCount;
        float inertia;
        float angularDamping;
        float maxBrakeTorque;
        float differentialRatio;
} JPH_VehicleTrackSettings;

JPH_CAPI void JPH_VehicleTrackSettings_Init(JPH_VehicleTrackSettings *settings);

JPH_CAPI float JPH_VehicleTrack_GetAngularVelocity(const JPH_VehicleTrack *track);
JPH_CAPI void JPH_VehicleTrack_SetAngularVelocity(JPH_VehicleTrack *track, float velocity);
JPH_CAPI uint32_t JPH_VehicleTrack_GetDrivenWheel(const JPH_VehicleTrack *track);
JPH_CAPI float JPH_VehicleTrack_GetInertia(const JPH_VehicleTrack *track);
JPH_CAPI float JPH_VehicleTrack_GetAngularDamping(const JPH_VehicleTrack *track);
JPH_CAPI float JPH_VehicleTrack_GetMaxBrakeTorque(const JPH_VehicleTrack *track);
JPH_CAPI float JPH_VehicleTrack_GetDifferentialRatio(const JPH_VehicleTrack *track);

JPH_CAPI const JPH_VehicleTrack *JPH_TrackedVehicleController_GetTrack(const JPH_TrackedVehicleController *controller,
                                                                       JPH_TrackSide side);

/* WheelSettingsTV */
JPH_CAPI JPH_WheelSettingsTV *JPH_WheelSettingsTV_Create(void);
JPH_CAPI float JPH_WheelSettingsTV_GetLongitudinalFriction(const JPH_WheelSettingsTV *settings);
JPH_CAPI void JPH_WheelSettingsTV_SetLongitudinalFriction(JPH_WheelSettingsTV *settings, float value);
JPH_CAPI float JPH_WheelSettingsTV_GetLateralFriction(const JPH_WheelSettingsTV *settings);
JPH_CAPI void JPH_WheelSettingsTV_SetLateralFriction(JPH_WheelSettingsTV *settings, float value);

JPH_CAPI JPH_WheelTV *JPH_WheelTV_Create(const JPH_WheelSettingsTV *settings);
JPH_CAPI const JPH_WheelSettingsTV *JPH_WheelTV_GetSettings(const JPH_WheelTV *wheel);

JPH_CAPI JPH_TrackedVehicleControllerSettings *JPH_TrackedVehicleControllerSettings_Create(void);

JPH_CAPI void JPH_TrackedVehicleControllerSettings_GetEngine(const JPH_TrackedVehicleControllerSettings *settings,
                                                             JPH_VehicleEngineSettings *result);
JPH_CAPI void JPH_TrackedVehicleControllerSettings_SetEngine(JPH_TrackedVehicleControllerSettings *settings,
                                                             const JPH_VehicleEngineSettings *value);
JPH_CAPI const JPH_VehicleTransmissionSettings *JPH_TrackedVehicleControllerSettings_GetTransmission(
        const JPH_TrackedVehicleControllerSettings *settings);
JPH_CAPI void JPH_TrackedVehicleControllerSettings_SetTransmission(JPH_TrackedVehicleControllerSettings *settings,
                                                                   const JPH_VehicleTransmissionSettings *value);

JPH_CAPI void JPH_TrackedVehicleController_SetDriverInput(JPH_TrackedVehicleController *controller,
                                                          float forward,
                                                          float leftRatio,
                                                          float rightRatio,
                                                          float brake);
JPH_CAPI float JPH_TrackedVehicleController_GetForwardInput(const JPH_TrackedVehicleController *controller);
JPH_CAPI void JPH_TrackedVehicleController_SetForwardInput(JPH_TrackedVehicleController *controller, float value);
JPH_CAPI float JPH_TrackedVehicleController_GetLeftRatio(const JPH_TrackedVehicleController *controller);
JPH_CAPI void JPH_TrackedVehicleController_SetLeftRatio(JPH_TrackedVehicleController *controller, float value);
JPH_CAPI float JPH_TrackedVehicleController_GetRightRatio(const JPH_TrackedVehicleController *controller);
JPH_CAPI void JPH_TrackedVehicleController_SetRightRatio(JPH_TrackedVehicleController *controller, float value);
JPH_CAPI float JPH_TrackedVehicleController_GetBrakeInput(const JPH_TrackedVehicleController *controller);
JPH_CAPI void JPH_TrackedVehicleController_SetBrakeInput(JPH_TrackedVehicleController *controller, float value);
JPH_CAPI const JPH_VehicleEngine *JPH_TrackedVehicleController_GetEngine(const JPH_TrackedVehicleController
                                                                                 *controller);
JPH_CAPI const JPH_VehicleTransmission *JPH_TrackedVehicleController_GetTransmission(const JPH_TrackedVehicleController
                                                                                             *controller);

/* MotorcycleController */
JPH_CAPI JPH_MotorcycleControllerSettings *JPH_MotorcycleControllerSettings_Create(void);
JPH_CAPI float JPH_MotorcycleControllerSettings_GetMaxLeanAngle(const JPH_MotorcycleControllerSettings *settings);
JPH_CAPI void JPH_MotorcycleControllerSettings_SetMaxLeanAngle(JPH_MotorcycleControllerSettings *settings, float value);
JPH_CAPI float JPH_MotorcycleControllerSettings_GetLeanSpringConstant(const JPH_MotorcycleControllerSettings *settings);
JPH_CAPI void JPH_MotorcycleControllerSettings_SetLeanSpringConstant(JPH_MotorcycleControllerSettings *settings,
                                                                     float value);
JPH_CAPI float JPH_MotorcycleControllerSettings_GetLeanSpringDamping(const JPH_MotorcycleControllerSettings *settings);
JPH_CAPI void JPH_MotorcycleControllerSettings_SetLeanSpringDamping(JPH_MotorcycleControllerSettings *settings,
                                                                    float value);
JPH_CAPI float JPH_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficient(
        const JPH_MotorcycleControllerSettings *settings);
JPH_CAPI void JPH_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficient(JPH_MotorcycleControllerSettings
                                                                                           *settings,
                                                                                   float value);
JPH_CAPI float JPH_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficientDecay(
        const JPH_MotorcycleControllerSettings *settings);
JPH_CAPI void JPH_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficientDecay(JPH_MotorcycleControllerSettings
                                                                                                *settings,
                                                                                        float value);
JPH_CAPI float JPH_MotorcycleControllerSettings_GetLeanSmoothingFactor(const JPH_MotorcycleControllerSettings
                                                                               *settings);
JPH_CAPI void JPH_MotorcycleControllerSettings_SetLeanSmoothingFactor(JPH_MotorcycleControllerSettings *settings,
                                                                      float value);

JPH_CAPI float JPH_MotorcycleController_GetWheelBase(const JPH_MotorcycleController *controller);
JPH_CAPI bool JPH_MotorcycleController_IsLeanControllerEnabled(const JPH_MotorcycleController *controller);
JPH_CAPI void JPH_MotorcycleController_EnableLeanController(JPH_MotorcycleController *controller, bool value);
JPH_CAPI bool JPH_MotorcycleController_IsLeanSteeringLimitEnabled(const JPH_MotorcycleController *controller);
JPH_CAPI void JPH_MotorcycleController_EnableLeanSteeringLimit(JPH_MotorcycleController *controller, bool value);
JPH_CAPI float JPH_MotorcycleController_GetLeanSpringConstant(const JPH_MotorcycleController *controller);
JPH_CAPI void JPH_MotorcycleController_SetLeanSpringConstant(JPH_MotorcycleController *controller, float value);
JPH_CAPI float JPH_MotorcycleController_GetLeanSpringDamping(const JPH_MotorcycleController *controller);
JPH_CAPI void JPH_MotorcycleController_SetLeanSpringDamping(JPH_MotorcycleController *controller, float value);
JPH_CAPI float JPH_MotorcycleController_GetLeanSpringIntegrationCoefficient(const JPH_MotorcycleController *controller);
JPH_CAPI void JPH_MotorcycleController_SetLeanSpringIntegrationCoefficient(JPH_MotorcycleController *controller,
                                                                           float value);
JPH_CAPI float JPH_MotorcycleController_GetLeanSpringIntegrationCoefficientDecay(const JPH_MotorcycleController
                                                                                         *controller);
JPH_CAPI void JPH_MotorcycleController_SetLeanSpringIntegrationCoefficientDecay(JPH_MotorcycleController *controller,
                                                                                float value);
JPH_CAPI float JPH_MotorcycleController_GetLeanSmoothingFactor(const JPH_MotorcycleController *controller);
JPH_CAPI void JPH_MotorcycleController_SetLeanSmoothingFactor(JPH_MotorcycleController *controller, float value);

/* LinearCurve */
JPH_CAPI JPH_LinearCurve *JPH_LinearCurve_Create(void);
JPH_CAPI void JPH_LinearCurve_Destroy(JPH_LinearCurve *curve);
JPH_CAPI void JPH_LinearCurve_Clear(JPH_LinearCurve *curve);
JPH_CAPI void JPH_LinearCurve_Reserve(JPH_LinearCurve *curve, uint32_t numPoints);
JPH_CAPI void JPH_LinearCurve_AddPoint(JPH_LinearCurve *curve, float x, float y);
JPH_CAPI void JPH_LinearCurve_Sort(JPH_LinearCurve *curve);
JPH_CAPI float JPH_LinearCurve_GetMinX(const JPH_LinearCurve *curve);
JPH_CAPI float JPH_LinearCurve_GetMaxX(const JPH_LinearCurve *curve);
JPH_CAPI float JPH_LinearCurve_GetValue(const JPH_LinearCurve *curve, float x);
JPH_CAPI size_t JPH_LinearCurve_GetPointCount(const JPH_LinearCurve *curve);
JPH_CAPI void JPH_LinearCurve_GetPoint(const JPH_LinearCurve *curve, uint32_t index, JPH_Point *result);
JPH_CAPI void JPH_LinearCurve_GetPoints(const JPH_LinearCurve *curve, JPH_Point *points, size_t *count);

#ifdef __cplusplus
}
#endif

#endif /* JOLTC_H */
