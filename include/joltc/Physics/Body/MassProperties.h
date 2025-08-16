//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_MASSPROPERTIES_H
#define JOLTC_MASSPROPERTIES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Mat44.h>
#include <joltc/Math/Vector3.h>

typedef struct JPH_MassProperties
{
        float mass;
        JPH_Mat44 inertia;
} JPH_MassProperties;

JPH_CAPI bool JPH_MassProperties_DecomposePrincipalMomentsOfInertia(const JPH_MassProperties *properties,
                                                                    JPH_Mat44 *rotation,
                                                                    Vector3 *diagonal);
JPH_CAPI void JPH_MassProperties_ScaleToMass(JPH_MassProperties *properties, float mass);
JPH_CAPI void JPH_MassProperties_GetEquivalentSolidBoxSize(float mass, const Vector3 *inertiaDiagonal, Vector3 *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_MASSPROPERTIES_H
