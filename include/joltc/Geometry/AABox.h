//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_AABOX_H
#define JOLTC_AABOX_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Vector3.h>

typedef struct JPH_AABox
{
        Vector3 min;
        Vector3 max;
} JPH_AABox;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_AABOX_H
