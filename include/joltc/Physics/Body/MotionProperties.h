//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_MOTIONPROPERTIES_H
#define JOLTC_MOTIONPROPERTIES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/MassProperties.h>

typedef struct JPH_MotionProperties JPH_MotionProperties;

JPH_CAPI JPH_AllowedDOFs JPH_MotionProperties_GetAllowedDOFs(const JPH_MotionProperties *properties);
JPH_CAPI void JPH_MotionProperties_SetLinearDamping(JPH_MotionProperties *properties, float damping);
JPH_CAPI float JPH_MotionProperties_GetLinearDamping(const JPH_MotionProperties *properties);
JPH_CAPI void JPH_MotionProperties_SetAngularDamping(JPH_MotionProperties *properties, float damping);
JPH_CAPI float JPH_MotionProperties_GetAngularDamping(const JPH_MotionProperties *properties);
JPH_CAPI void JPH_MotionProperties_SetMassProperties(JPH_MotionProperties *properties,
                                                     JPH_AllowedDOFs allowedDOFs,
                                                     const JPH_MassProperties *massProperties);
JPH_CAPI float JPH_MotionProperties_GetInverseMassUnchecked(JPH_MotionProperties *properties);
JPH_CAPI void JPH_MotionProperties_SetInverseMass(JPH_MotionProperties *properties, float inverseMass);
JPH_CAPI void JPH_MotionProperties_GetInverseInertiaDiagonal(JPH_MotionProperties *properties, Vector3 *result);
JPH_CAPI void JPH_MotionProperties_GetInertiaRotation(JPH_MotionProperties *properties, JPH_Quat *result);
JPH_CAPI void JPH_MotionProperties_SetInverseInertia(JPH_MotionProperties *properties,
                                                     const Vector3 *diagonal,
                                                     const JPH_Quat *rot);
JPH_CAPI void JPH_MotionProperties_ScaleToMass(JPH_MotionProperties *properties, float mass);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_MOTIONPROPERTIES_H
