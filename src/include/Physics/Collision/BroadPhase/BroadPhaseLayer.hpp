//
// Created by NBT22 on 2/3/26.
//

#pragma once

#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

static inline JPH::BroadPhaseLayer ToJolt(const JPH_BroadPhaseLayer *layer)
{
    return JPH::BroadPhaseLayer(*layer);
}

static inline void FromJolt(const JPH::BroadPhaseLayer &layer, JPH_BroadPhaseLayer &result)
{
    result = layer.GetValue();
}

static inline const JPH::BroadPhaseLayerFilter &ToJolt(const JPH_BroadPhaseLayerFilter *filter)
{
    static constexpr JPH::BroadPhaseLayerFilter defaultFilter{};
    return filter != nullptr ? *reinterpret_cast<const JPH::BroadPhaseLayerFilter *>(filter) : defaultFilter;
}

static inline void FromJolt(const JPH::BroadPhaseLayerFilter *filter, const JPH_BroadPhaseLayerFilter *&result)
{
    result = reinterpret_cast<const JPH_BroadPhaseLayerFilter *>(filter);
}
