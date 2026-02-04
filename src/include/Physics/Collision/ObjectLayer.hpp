//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/Physics/Collision/ObjectLayer.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

static inline JPH::ObjectLayer ToJolt(const JPH_ObjectLayer *layer)
{
    return *layer;
}

static inline void FromJolt(const JPH::ObjectLayer &layer, JPH_ObjectLayer &result)
{
    result = layer;
}

static inline const JPH::ObjectLayerFilter &ToJolt(const JPH_ObjectLayerFilter *filter)
{
    static constexpr JPH::ObjectLayerFilter defaultFilter{};
    return filter != nullptr ? *reinterpret_cast<const JPH::ObjectLayerFilter *>(filter) : defaultFilter;
}

static inline void FromJolt(const JPH::ObjectLayerFilter *filter, const JPH_ObjectLayerFilter *&result)
{
    result = reinterpret_cast<const JPH_ObjectLayerFilter *>(filter);
}
