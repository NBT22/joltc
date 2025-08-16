//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/GroupFilter.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/GroupFilter.h>

static inline const JPH::GroupFilter &AsGroupFilter(const JPH_GroupFilter &t)
{
    return reinterpret_cast<const JPH::GroupFilter &>(t);
}
static inline const JPH::GroupFilter *AsGroupFilter(const JPH_GroupFilter *t)
{
    return reinterpret_cast<const JPH::GroupFilter *>(t);
}
static inline JPH::GroupFilter &AsGroupFilter(JPH_GroupFilter &t)
{
    return reinterpret_cast<JPH::GroupFilter &>(t);
}
static inline JPH::GroupFilter *AsGroupFilter(JPH_GroupFilter *t)
{
    return reinterpret_cast<JPH::GroupFilter *>(t);
}
static inline const JPH_GroupFilter &ToGroupFilter(const JPH::GroupFilter &t)
{
    return reinterpret_cast<const JPH_GroupFilter &>(t);
}
static inline const JPH_GroupFilter *ToGroupFilter(const JPH::GroupFilter *t)
{
    return reinterpret_cast<const JPH_GroupFilter *>(t);
}
static inline JPH_GroupFilter &ToGroupFilter(JPH::GroupFilter &t)
{
    return reinterpret_cast<JPH_GroupFilter &>(t);
}
static inline JPH_GroupFilter *ToGroupFilter(JPH::GroupFilter *t)
{
    return reinterpret_cast<JPH_GroupFilter *>(t);
}
