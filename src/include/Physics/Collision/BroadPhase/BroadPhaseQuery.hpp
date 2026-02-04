//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>

static inline const JPH::BroadPhaseQuery &ToJolt(const JPH_BroadPhaseQuery *query)
{
    JPH_ASSERT(query);
    return *reinterpret_cast<const JPH::BroadPhaseQuery *>(query);
}

static inline void FromJolt(const JPH::BroadPhaseQuery *query, const JPH_BroadPhaseQuery *&result)
{
    result = reinterpret_cast<const JPH_BroadPhaseQuery *>(query);
}
