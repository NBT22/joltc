//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/CollisionGroup.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Physics/Collision/GroupFilter.hpp>

static inline JPH::CollisionGroup ToJolt(const JPH_CollisionGroup *group)
{
    JPH::CollisionGroup result(AsGroupFilter(group->groupFilter), group->groupID, group->subGroupID);
    return result;
}

static inline void FromJolt(const JPH::CollisionGroup &jolt, JPH_CollisionGroup *result)
{
    result->groupFilter = ToGroupFilter(jolt.GetGroupFilter());
    result->groupID = jolt.GetGroupID();
    result->subGroupID = jolt.GetSubGroupID();
}
