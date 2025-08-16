//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_PHYSICSMATERIAL_H
#define JOLTC_PHYSICSMATERIAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct JPH_PhysicsMaterial JPH_PhysicsMaterial;

JPH_CAPI JPH_PhysicsMaterial *JPH_PhysicsMaterial_Create(const char *name, uint32_t color);
JPH_CAPI void JPH_PhysicsMaterial_Destroy(JPH_PhysicsMaterial *material);
JPH_CAPI const char *JPH_PhysicsMaterial_GetDebugName(const JPH_PhysicsMaterial *material);
JPH_CAPI uint32_t JPH_PhysicsMaterial_GetDebugColor(const JPH_PhysicsMaterial *material);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_PHYSICSMATERIAL_H
