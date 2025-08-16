//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

static inline const JPH::BodyCreationSettings &AsBodyCreationSettings(const JPH_BodyCreationSettings &t)
{
    return reinterpret_cast<const JPH::BodyCreationSettings &>(t);
}
static inline const JPH::BodyCreationSettings *AsBodyCreationSettings(const JPH_BodyCreationSettings *t)
{
    return reinterpret_cast<const JPH::BodyCreationSettings *>(t);
}
static inline JPH::BodyCreationSettings &AsBodyCreationSettings(JPH_BodyCreationSettings &t)
{
    return reinterpret_cast<JPH::BodyCreationSettings &>(t);
}
static inline JPH::BodyCreationSettings *AsBodyCreationSettings(JPH_BodyCreationSettings *t)
{
    return reinterpret_cast<JPH::BodyCreationSettings *>(t);
}
static inline const JPH_BodyCreationSettings &ToBodyCreationSettings(const JPH::BodyCreationSettings &t)
{
    return reinterpret_cast<const JPH_BodyCreationSettings &>(t);
}
static inline const JPH_BodyCreationSettings *ToBodyCreationSettings(const JPH::BodyCreationSettings *t)
{
    return reinterpret_cast<const JPH_BodyCreationSettings *>(t);
}
static inline JPH_BodyCreationSettings &ToBodyCreationSettings(JPH::BodyCreationSettings &t)
{
    return reinterpret_cast<JPH_BodyCreationSettings &>(t);
}
static inline JPH_BodyCreationSettings *ToBodyCreationSettings(JPH::BodyCreationSettings *t)
{
    return reinterpret_cast<JPH_BodyCreationSettings *>(t);
}
