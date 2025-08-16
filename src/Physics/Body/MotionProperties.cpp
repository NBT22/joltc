//
// Created by NBT22 on 8/15/25.
//

#include <joltc/enums.h>
#include <joltc/Physics/Body/MassProperties.h>
#include <Math/Quat.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/MassProperties.hpp>
#include <Physics/Body/MotionProperties.hpp>

JPH_AllowedDOFs JPH_MotionProperties_GetAllowedDOFs(const JPH_MotionProperties *properties)
{
    return static_cast<JPH_AllowedDOFs>(reinterpret_cast<const JPH::MotionProperties *>(properties)->GetAllowedDOFs());
}

void JPH_MotionProperties_SetLinearDamping(JPH_MotionProperties *properties, const float damping)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetLinearDamping(damping);
}

float JPH_MotionProperties_GetLinearDamping(const JPH_MotionProperties *properties)
{
    return reinterpret_cast<const JPH::MotionProperties *>(properties)->GetLinearDamping();
}

void JPH_MotionProperties_SetAngularDamping(JPH_MotionProperties *properties, const float damping)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetAngularDamping(damping);
}

float JPH_MotionProperties_GetAngularDamping(const JPH_MotionProperties *properties)
{
    return reinterpret_cast<const JPH::MotionProperties *>(properties)->GetAngularDamping();
}

void JPH_MotionProperties_SetMassProperties(JPH_MotionProperties *properties,
                                            JPH_AllowedDOFs allowedDOFs,
                                            const JPH_MassProperties *massProperties)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)
            ->SetMassProperties(static_cast<JPH::EAllowedDOFs>(allowedDOFs), ToJolt(massProperties));
}

float JPH_MotionProperties_GetInverseMassUnchecked(JPH_MotionProperties *properties)
{
    return reinterpret_cast<JPH::MotionProperties *>(properties)->GetInverseMassUnchecked();
}

void JPH_MotionProperties_SetInverseMass(JPH_MotionProperties *properties, const float inverseMass)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetInverseMass(inverseMass);
}

void JPH_MotionProperties_GetInverseInertiaDiagonal(JPH_MotionProperties *properties, Vector3 *result)
{
    FromJolt(reinterpret_cast<JPH::MotionProperties *>(properties)->GetInverseInertiaDiagonal(), result);
}

void JPH_MotionProperties_GetInertiaRotation(JPH_MotionProperties *properties, JPH_Quat *result)
{
    FromJolt(reinterpret_cast<JPH::MotionProperties *>(properties)->GetInertiaRotation(), result);
}

void JPH_MotionProperties_SetInverseInertia(JPH_MotionProperties *properties,
                                            const Vector3 *diagonal,
                                            const JPH_Quat *rot)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->SetInverseInertia(ToJolt(diagonal), ToJolt(rot));
}

void JPH_MotionProperties_ScaleToMass(JPH_MotionProperties *properties, const float mass)
{
    reinterpret_cast<JPH::MotionProperties *>(properties)->ScaleToMass(mass);
}
