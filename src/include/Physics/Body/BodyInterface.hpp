//
// Created by NBT22 on 8/14/25.
//

#pragma once

#include <joltc/Physics/Body/BodyInterface.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>

static inline const JPH::BodyInterface &AsBodyInterface(const JPH_BodyInterface &t)
{
    return reinterpret_cast<const JPH::BodyInterface &>(t);
}
static inline const JPH::BodyInterface *AsBodyInterface(const JPH_BodyInterface *t)
{
    return reinterpret_cast<const JPH::BodyInterface *>(t);
}
static inline JPH::BodyInterface &AsBodyInterface(JPH_BodyInterface &t)
{
    return reinterpret_cast<JPH::BodyInterface &>(t);
}
static inline JPH::BodyInterface *AsBodyInterface(JPH_BodyInterface *t)
{
    return reinterpret_cast<JPH::BodyInterface *>(t);
}
static inline const JPH_BodyInterface &ToBodyInterface(const JPH::BodyInterface &t)
{
    return reinterpret_cast<const JPH_BodyInterface &>(t);
}
static inline const JPH_BodyInterface *ToBodyInterface(const JPH::BodyInterface *t)
{
    return reinterpret_cast<const JPH_BodyInterface *>(t);
}
static inline JPH_BodyInterface &ToBodyInterface(JPH::BodyInterface &t)
{
    return reinterpret_cast<JPH_BodyInterface &>(t);
}
static inline JPH_BodyInterface *ToBodyInterface(JPH::BodyInterface *t)
{
    return reinterpret_cast<JPH_BodyInterface *>(t);
}
