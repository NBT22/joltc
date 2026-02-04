//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_OBJECTLAYER_H
#define JOLTC_OBJECTLAYER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef uint32_t JPH_ObjectLayer;
typedef struct JPH_ObjectLayerFilter JPH_ObjectLayerFilter;

typedef struct JPH_ObjectLayerFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(JPH_ObjectLayer layer);
} JPH_ObjectLayerFilter_Impl;

static constexpr JPH_ObjectLayer JPH_ObjectLayerInvalid = ~0u;

JPH_CAPI JPH_ObjectLayerFilter *JPH_ObjectLayerFilter_Create(const JPH_ObjectLayerFilter_Impl *impl);
JPH_CAPI void JPH_ObjectLayerFilter_Destroy(JPH_ObjectLayerFilter *filter);
JPH_CAPI void JPH_ObjectLayerFilter_SetImpl(JPH_ObjectLayerFilter *filter, const JPH_ObjectLayerFilter_Impl *impl);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_OBJECTLAYER_H
