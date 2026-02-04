//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_BODYID_H
#define JOLTC_BODYID_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef uint32_t JPH_BodyID;
static constexpr JPH_BodyID JPH_BodyId_InvalidBodyID = 0xffffffff;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BODYID_H
