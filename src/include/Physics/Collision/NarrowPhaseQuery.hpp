//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>

static inline const JPH::NarrowPhaseQuery &ToJolt(const JPH_NarrowPhaseQuery *query)
{
    static constexpr JPH::NarrowPhaseQuery defaultQuery{};
    return query != nullptr ? *reinterpret_cast<const JPH::NarrowPhaseQuery *>(query) : defaultQuery;
}

static inline void FromJolt(const JPH::NarrowPhaseQuery *query, const JPH_NarrowPhaseQuery *&result)
{
    result = reinterpret_cast<const JPH_NarrowPhaseQuery *>(query);
}
