//
// Created by NBT22 on 8/14/25.
//

#ifndef JOLTC_TRANSFORM_H
#define JOLTC_TRANSFORM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Quat.h>
#include <joltc/Math/Vector3.h>

typedef struct Transform
{
        Vector3 position;
        JPH_Quat rotation;
} Transform;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_TRANSFORM_H
