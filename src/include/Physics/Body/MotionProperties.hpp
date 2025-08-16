//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Body/MotionProperties.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/MotionProperties.h>

static inline const JPH::MotionProperties &AsMotionProperties(const JPH_MotionProperties &t)
{
    return reinterpret_cast<const JPH::MotionProperties &>(t);
}
static inline const JPH::MotionProperties *AsMotionProperties(const JPH_MotionProperties *t)
{
    return reinterpret_cast<const JPH::MotionProperties *>(t);
}
static inline JPH::MotionProperties &AsMotionProperties(JPH_MotionProperties &t)
{
    return reinterpret_cast<JPH::MotionProperties &>(t);
}
static inline JPH::MotionProperties *AsMotionProperties(JPH_MotionProperties *t)
{
    return reinterpret_cast<JPH::MotionProperties *>(t);
}
static inline const JPH_MotionProperties &ToMotionProperties(const JPH::MotionProperties &t)
{
    return reinterpret_cast<const JPH_MotionProperties &>(t);
}
static inline const JPH_MotionProperties *ToMotionProperties(const JPH::MotionProperties *t)
{
    return reinterpret_cast<const JPH_MotionProperties *>(t);
}
static inline JPH_MotionProperties &ToMotionProperties(JPH::MotionProperties &t)
{
    return reinterpret_cast<JPH_MotionProperties &>(t);
}
static inline JPH_MotionProperties *ToMotionProperties(JPH::MotionProperties *t)
{
    return reinterpret_cast<JPH_MotionProperties *>(t);
}
