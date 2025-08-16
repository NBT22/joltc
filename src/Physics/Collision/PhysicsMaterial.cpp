//
// Created by NBT22 on 8/15/25.
//

#include <cstdint>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/Color.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Physics/Collision/PhysicsMaterial.hpp>

JPH_PhysicsMaterial *JPH_PhysicsMaterial_Create(const char *name, const uint32_t color)
{
    JPH::PhysicsMaterialSimple *const material = new JPH::PhysicsMaterialSimple(name, JPH::Color(color));
    material->AddRef();

    return ToPhysicsMaterial(material);
}

void JPH_PhysicsMaterial_Destroy(JPH_PhysicsMaterial *material)
{
    if (material != nullptr)
    {
        AsPhysicsMaterial(material)->Release();
    }
}

const char *JPH_PhysicsMaterial_GetDebugName(const JPH_PhysicsMaterial *material)
{
    return AsPhysicsMaterial(material)->GetDebugName();
}

uint32_t JPH_PhysicsMaterial_GetDebugColor(const JPH_PhysicsMaterial *material)
{
    return AsPhysicsMaterial(material)->GetDebugColor().GetUInt32();
}
