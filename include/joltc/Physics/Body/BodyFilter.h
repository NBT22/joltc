//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_BODYFILTER_H
#define JOLTC_BODYFILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Body/Body.h>
#include <joltc/Physics/Body/BodyID.h>

typedef struct JPH_BodyFilter JPH_BodyFilter;

typedef struct JPH_BodyFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(JPH_BodyID bodyID);
        bool(JPH_API_CALL *ShouldCollideLocked)(const JPH_Body *body);
} JPH_BodyFilter_Impl;

JPH_CAPI JPH_BodyFilter *JPH_BodyFilter_Create(const JPH_BodyFilter_Impl *impl);
JPH_CAPI void JPH_BodyFilter_Destroy(JPH_BodyFilter *filter);
JPH_CAPI void JPH_BodyFilter_SetImpl(JPH_BodyFilter *filter, const JPH_BodyFilter_Impl *impl);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_BODYFILTER_H
