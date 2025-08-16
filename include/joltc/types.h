//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_TYPES_H
#define JOLTC_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Vector3.h>
#include <stdint.h>

typedef uint32_t JPH_Bool;
typedef uint32_t JPH_Color;
typedef uint32_t JPH_BodyId;
typedef uint32_t JPH_SubShapeId;
typedef uint32_t JPH_ObjectLayer;
typedef uint8_t JPH_BroadPhaseLayer;
typedef uint32_t JPH_CollisionGroupId;
typedef uint32_t JPH_CollisionSubGroupId;
typedef uint32_t JPH_CharacterId;

typedef struct JPH_SubShapeIDPair
{
        JPH_BodyId Body1ID;
        JPH_SubShapeId subShapeID1;
        JPH_BodyId Body2ID;
        JPH_SubShapeId subShapeID2;
} JPH_SubShapeIDPair;

typedef struct JPH_BroadPhaseCastResult
{
        JPH_BodyId bodyID;
        float fraction;
} JPH_BroadPhaseCastResult;

typedef struct JPH_RayCastResult
{
        JPH_BodyId bodyID;
        float fraction;
        JPH_SubShapeId subShapeID2;
} JPH_RayCastResult;

typedef struct JPH_CollidePointResult
{
        JPH_BodyId bodyID;
        JPH_SubShapeId subShapeID2;
} JPH_CollidePointResult;

typedef struct JPH_ShapeCastResult
{
        Vector3 contactPointOn1;
        Vector3 contactPointOn2;
        Vector3 penetrationAxis;
        float penetrationDepth;
        JPH_SubShapeId subShapeID1;
        JPH_SubShapeId subShapeID2;
        JPH_BodyId bodyID2;
        float fraction;
        bool isBackFaceHit;
} JPH_ShapeCastResult;

typedef struct JPH_SupportingFace
{
        uint32_t count;
        Vector3 vertices[32];
} JPH_SupportingFace;

typedef void JPH_CastRayResultCallback(void *context, const JPH_RayCastResult *result);
typedef void JPH_RayCastBodyResultCallback(void *context, const JPH_BroadPhaseCastResult *result);
typedef void JPH_CollideShapeBodyResultCallback(void *context, JPH_BodyId result);
typedef void JPH_CollidePointResultCallback(void *context, const JPH_CollidePointResult *result);
// typedef void JPH_CollideShapeResultCallback(void *context, const JPH_CollideShapeResult *result);
typedef void JPH_CastShapeResultCallback(void *context, const JPH_ShapeCastResult *result);

typedef float JPH_CastRayCollectorCallback(void *context, const JPH_RayCastResult *result);
typedef float JPH_RayCastBodyCollectorCallback(void *context, const JPH_BroadPhaseCastResult *result);
typedef float JPH_CollideShapeBodyCollectorCallback(void *context, JPH_BodyId result);
typedef float JPH_CollidePointCollectorCallback(void *context, const JPH_CollidePointResult *result);
// typedef float JPH_CollideShapeCollectorCallback(void *context, const JPH_CollideShapeResult *result);
typedef float JPH_CastShapeCollectorCallback(void *context, const JPH_ShapeCastResult *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_TYPES_H
