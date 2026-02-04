//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_CASTRESULT_H
#define JOLTC_CASTRESULT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>

typedef struct JPH_BroadPhaseCastResult
{
        JPH_BodyID bodyID;
        float fraction;
} JPH_BroadPhaseCastResult;

typedef struct JPH_RayCastResult
{
        JPH_BodyID bodyID;
        float fraction;
        JPH_SubShapeID subShapeID2;
} JPH_RayCastResult;

typedef void JPH_RayCastBodyResultCallback(void *context, const JPH_BroadPhaseCastResult *result);
typedef void JPH_CastRayResultCallback(void *context, const JPH_RayCastResult *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_CASTRESULT_H
