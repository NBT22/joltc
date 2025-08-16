//
// Created by NBT22 on 8/15/25.
//

#include <joltc/Physics/Body/MassProperties.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Math/Mat44.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/MassProperties.hpp>

bool JPH_MassProperties_DecomposePrincipalMomentsOfInertia(const JPH_MassProperties *properties,
                                                           JPH_Mat44 *rotation,
                                                           Vector3 *diagonal)
{
    JPH::Mat44 joltRotation{};
    JPH::Vec3 joltDiagonal{};
    const JPH::MassProperties joltProperties = ToJolt(properties);
    if (!joltProperties.DecomposePrincipalMomentsOfInertia(joltRotation, joltDiagonal))
    {
        return false;
    }
    FromJolt(joltRotation, rotation);
    FromJolt(joltDiagonal, diagonal);
    return true;
}

void JPH_MassProperties_ScaleToMass(JPH_MassProperties *properties, const float mass)
{
    JPH::MassProperties joltProperties = ToJolt(properties);
    joltProperties.ScaleToMass(mass);
    properties->mass = joltProperties.mMass;
    FromJolt(joltProperties.mInertia, &properties->inertia);
}

void JPH_MassProperties_GetEquivalentSolidBoxSize(const float mass, const Vector3 *inertiaDiagonal, Vector3 *result)
{
    FromJolt(JPH::MassProperties::sGetEquivalentSolidBoxSize(mass, ToJolt(inertiaDiagonal)), result);
}
