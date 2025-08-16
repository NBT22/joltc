//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Body/Body.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>

static inline const JPH::Body &AsBody(const JPH_Body &t)
{
    return reinterpret_cast<const JPH::Body &>(t);
}
static inline const JPH::Body *AsBody(const JPH_Body *t)
{
    return reinterpret_cast<const JPH::Body *>(t);
}
static inline JPH::Body &AsBody(JPH_Body &t)
{
    return reinterpret_cast<JPH::Body &>(t);
}
static inline JPH::Body *AsBody(JPH_Body *t)
{
    return reinterpret_cast<JPH::Body *>(t);
}
static inline const JPH_Body &ToBody(const JPH::Body &t)
{
    return reinterpret_cast<const JPH_Body &>(t);
}
static inline const JPH_Body *ToBody(const JPH::Body *t)
{
    return reinterpret_cast<const JPH_Body *>(t);
}
static inline JPH_Body &ToBody(JPH::Body &t)
{
    return reinterpret_cast<JPH_Body &>(t);
}
static inline JPH_Body *ToBody(JPH::Body *t)
{
    return reinterpret_cast<JPH_Body *>(t);
}
