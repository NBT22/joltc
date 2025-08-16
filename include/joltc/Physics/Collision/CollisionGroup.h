//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_COLLISIONGROUP_H
#define JOLTC_COLLISIONGROUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/types.h>
#include <joltc/Physics/Body/BodyCreationSettings.h>

typedef struct JPH_CollisionGroup
{
        const struct JPH_GroupFilter *groupFilter;
        JPH_CollisionGroupId groupID;
        JPH_CollisionSubGroupId subGroupID;
} JPH_CollisionGroup;

JPH_CAPI void JPH_BodyCreationSettings_GetCollisionGroup(const JPH_BodyCreationSettings *settings,
                                                         JPH_CollisionGroup *result);
JPH_CAPI void JPH_BodyCreationSettings_SetCollisionGroup(JPH_BodyCreationSettings *settings,
                                                         const JPH_CollisionGroup *value);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_COLLISIONGROUP_H
