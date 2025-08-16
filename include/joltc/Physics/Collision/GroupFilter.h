//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_GROUPFILTER_H
#define JOLTC_GROUPFILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Collision/CollisionGroup.h>

typedef struct JPH_GroupFilter JPH_GroupFilter;

JPH_CAPI void JPH_GroupFilter_Destroy(JPH_GroupFilter *groupFilter);
JPH_CAPI bool JPH_GroupFilter_CanCollide(JPH_GroupFilter *groupFilter,
                                         const JPH_CollisionGroup *group1,
                                         const JPH_CollisionGroup *group2);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_GROUPFILTER_H
