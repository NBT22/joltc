//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_COLLIDEPOINTRESULT_H
#define JOLTC_COLLIDEPOINTRESULT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>

typedef struct JPH_CollidePointResult
{
        JPH_BodyID bodyID;
        JPH_SubShapeID subShapeID2;
} JPH_CollidePointResult;

typedef void JPH_CollidePointResultCallback(void *context, const JPH_CollidePointResult *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_COLLIDEPOINTRESULT_H
