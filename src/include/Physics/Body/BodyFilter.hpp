//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/Physics/Body/BodyFilter.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyFilter.h>

static inline const JPH::BodyFilter &ToJolt(const JPH_BodyFilter *filter)
{
    static constexpr JPH::BodyFilter defaultFilter{};
    return filter != nullptr ? *reinterpret_cast<const JPH::BodyFilter *>(filter) : defaultFilter;
}

static inline void FromJolt(const JPH::BodyFilter *filter, const JPH_BodyFilter *&result)
{
    result = reinterpret_cast<const JPH_BodyFilter *>(filter);
}
