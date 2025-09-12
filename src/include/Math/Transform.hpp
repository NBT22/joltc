//
// Created by NBT22 on 8/14/25.
//

#pragma once

#include <joltc/Math/Transform.h>
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
#include <Math/Quat.hpp>
#include <Math/Vector3.hpp>

static inline void ToJolt(const Transform *transform, JPH::Vec3 &position, JPH::Quat &rotation)
{
    position = ToJolt(transform->position);
    rotation = ToJolt(transform->rotation);
}
