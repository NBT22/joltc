//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_BROADPHASELAYER_H
#define JOLTC_BROADPHASELAYER_H

#ifdef __cplusplus
extern "C"
{
#endif

#if !defined(__cpp_constexpr) && __STDC_VERSION__ < 202311L
#define constexpr const
#endif

#include <stdint.h>

typedef uint8_t JPH_BroadPhaseLayer;
typedef struct JPH_BroadPhaseLayerFilter JPH_BroadPhaseLayerFilter;

typedef struct JPH_BroadPhaseLayerFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(JPH_BroadPhaseLayer layer);
} JPH_BroadPhaseLayerFilter_Impl;

static constexpr JPH_BroadPhaseLayer JPH_BroadPhaseLayerInvalid = 0xff;

JPH_CAPI JPH_BroadPhaseLayerFilter *JPH_BroadPhaseLayerFilter_Create(const JPH_BroadPhaseLayerFilter_Impl *impl);
JPH_CAPI void JPH_BroadPhaseLayerFilter_Destroy(JPH_BroadPhaseLayerFilter *filter);
JPH_CAPI void JPH_BroadPhaseLayerFilter_SetImpl(JPH_BroadPhaseLayerFilter *filter,
                                                const JPH_BroadPhaseLayerFilter_Impl *impl);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BROADPHASELAYER_H
