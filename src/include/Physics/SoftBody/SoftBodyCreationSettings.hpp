//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>

static inline const JPH::SoftBodyCreationSettings &AsSoftBodyCreationSettings(const JPH_SoftBodyCreationSettings &t)
{
    return reinterpret_cast<const JPH::SoftBodyCreationSettings &>(t);
}
static inline const JPH::SoftBodyCreationSettings *AsSoftBodyCreationSettings(const JPH_SoftBodyCreationSettings *t)
{
    return reinterpret_cast<const JPH::SoftBodyCreationSettings *>(t);
}
static inline JPH::SoftBodyCreationSettings &AsSoftBodyCreationSettings(JPH_SoftBodyCreationSettings &t)
{
    return reinterpret_cast<JPH::SoftBodyCreationSettings &>(t);
}
static inline JPH::SoftBodyCreationSettings *AsSoftBodyCreationSettings(JPH_SoftBodyCreationSettings *t)
{
    return reinterpret_cast<JPH::SoftBodyCreationSettings *>(t);
}
static inline const JPH_SoftBodyCreationSettings &ToSoftBodyCreationSettings(const JPH::SoftBodyCreationSettings &t)
{
    return reinterpret_cast<const JPH_SoftBodyCreationSettings &>(t);
}
static inline const JPH_SoftBodyCreationSettings *ToSoftBodyCreationSettings(const JPH::SoftBodyCreationSettings *t)
{
    return reinterpret_cast<const JPH_SoftBodyCreationSettings *>(t);
}
static inline JPH_SoftBodyCreationSettings &ToSoftBodyCreationSettings(JPH::SoftBodyCreationSettings &t)
{
    return reinterpret_cast<JPH_SoftBodyCreationSettings &>(t);
}
static inline JPH_SoftBodyCreationSettings *ToSoftBodyCreationSettings(JPH::SoftBodyCreationSettings *t)
{
    return reinterpret_cast<JPH_SoftBodyCreationSettings *>(t);
}
