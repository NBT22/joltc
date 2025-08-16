//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_RMAT44_H
#define JOLTC_RMAT44_H

#include <joltc/Math/Quat.h>
#include <joltc/Math/Vector3.h>
#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Mat44.h>

#ifdef JPH_DOUBLE_PRECISION
typedef struct JPH_RMat44
{
        float m11, m12, m13, m14;
        float m21, m22, m23, m24;
        float m31, m32, m33, m34;
        double m41, m42, m43, m44;
} JPH_RMat44;
#else
typedef JPH_Mat44 JPH_RMat44;
#endif

JPH_CAPI void JPH_RMat44_Add(const JPH_RMat44 *m1, const JPH_RMat44 *m2, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_Subtract(const JPH_RMat44 *m1, const JPH_RMat44 *m2, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_Multiply(const JPH_RMat44 *m1, const JPH_RMat44 *m2, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_MultiplyScalar(const JPH_RMat44 *matrix, float scalar, JPH_RMat44 *result);

JPH_CAPI void JPH_RMat44_Rotation(const JPH_Quat *rotation, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_Translation(const Vector3 *translation, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_RotationTranslation(const JPH_Quat *rotation, const Vector3 *translation, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_InverseRotationTranslation(const JPH_Quat *rotation,
                                                    const Vector3 *translation,
                                                    JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_Scale(const Vector3 *scale, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_Inversed(const JPH_RMat44 *matrix, JPH_RMat44 *result);
JPH_CAPI void JPH_RMat44_Transposed(const JPH_RMat44 *matrix, JPH_RMat44 *result);

JPH_CAPI void JPH_RMat44_GetAxisX(const JPH_RMat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_RMat44_GetAxisY(const JPH_RMat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_RMat44_GetAxisZ(const JPH_RMat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_RMat44_GetTranslation(const JPH_RMat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_RMat44_GetQuaternion(const JPH_RMat44 *matrix, JPH_Quat *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_RMAT44_H
