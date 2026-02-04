//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_SUBSHAPEIDPAIR_H
#define JOLTC_SUBSHAPEIDPAIR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>

typedef struct JPH_SubShapeIDPair
{
        JPH_BodyID Body1ID;
        JPH_SubShapeID subShapeID1;
        JPH_BodyID Body2ID;
        JPH_SubShapeID subShapeID2;
} JPH_SubShapeIDPair;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_SUBSHAPEIDPAIR_H
