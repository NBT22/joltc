//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>

static inline const JPH::PhysicsMaterial &AsPhysicsMaterial(const JPH_PhysicsMaterial &t)
{
    return reinterpret_cast<const JPH::PhysicsMaterial &>(t);
}

static inline const JPH::PhysicsMaterial *AsPhysicsMaterial(const JPH_PhysicsMaterial *t)
{
    return reinterpret_cast<const JPH::PhysicsMaterial *>(t);
}

static inline JPH::PhysicsMaterial &AsPhysicsMaterial(JPH_PhysicsMaterial &t)
{
    return reinterpret_cast<JPH::PhysicsMaterial &>(t);
}

static inline JPH::PhysicsMaterial *AsPhysicsMaterial(JPH_PhysicsMaterial *t)
{
    return reinterpret_cast<JPH::PhysicsMaterial *>(t);
}

static inline const JPH_PhysicsMaterial &ToPhysicsMaterial(const JPH::PhysicsMaterial &t)
{
    return reinterpret_cast<const JPH_PhysicsMaterial &>(t);
}

static inline const JPH_PhysicsMaterial *ToPhysicsMaterial(const JPH::PhysicsMaterial *t)
{
    return reinterpret_cast<const JPH_PhysicsMaterial *>(t);
}

static inline JPH_PhysicsMaterial &ToPhysicsMaterial(JPH::PhysicsMaterial &t)
{
    return reinterpret_cast<JPH_PhysicsMaterial &>(t);
}

static inline JPH_PhysicsMaterial *ToPhysicsMaterial(JPH::PhysicsMaterial *t)
{
    return reinterpret_cast<JPH_PhysicsMaterial *>(t);
}
