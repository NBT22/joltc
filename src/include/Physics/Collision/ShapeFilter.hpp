//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/ShapeFilter.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>

static inline const JPH::ShapeFilter &ToJolt(const JPH_ShapeFilter *filter)
{
    static const JPH::ShapeFilter defaultFilter{};
    return filter != nullptr ? *reinterpret_cast<const JPH::ShapeFilter *>(filter) : defaultFilter;
}
