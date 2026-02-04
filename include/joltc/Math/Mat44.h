//
// Created by NBT22 on 8/14/25.
//

#ifndef JOLTC_MAT44_H
#define JOLTC_MAT44_H

#ifdef __cplusplus
extern "C"
{
#endif

#if !defined(__cpp_constexpr) && __STDC_VERSION__ < 202311L
#define constexpr const
#endif

#include <joltc/Math/Quat.h>
#include <joltc/Math/Vector3.h>

typedef struct JPH_Mat44
{
        float m11, m12, m13, m14;
        float m21, m22, m23, m24;
        float m31, m32, m33, m34;
        float m41, m42, m43, m44;
} JPH_Mat44;

static constexpr JPH_Mat44 JPH_Mat44_Zero = {};
static constexpr JPH_Mat44 JPH_Mat44_Identity = {
    .m11 = 1.0f,
    .m22 = 1.0f,
    .m33 = 1.0f,
    .m44 = 1.0f,
};

JPH_CAPI void JPH_Mat44_Add(const JPH_Mat44 *m1, const JPH_Mat44 *m2, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_Subtract(const JPH_Mat44 *m1, const JPH_Mat44 *m2, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_Multiply(const JPH_Mat44 *m1, const JPH_Mat44 *m2, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_MultiplyScalar(const JPH_Mat44 *matrix, float scalar, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_MultiplyVector3(const JPH_Mat44 *left, const Vector3 *right, Vector3 *result);

JPH_CAPI void JPH_Mat44_Rotation(const JPH_Quat *rotation, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat4_RotationAxisAngle(const Vector3 *axis, float angle, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_Translation(const Vector3 *translation, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_RotationTranslation(const JPH_Quat *rotation, const Vector3 *translation, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_InverseRotationTranslation(const JPH_Quat *rotation,
                                                   const Vector3 *translation,
                                                   JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_Scale(const Vector3 *scale, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_Inversed(const JPH_Mat44 *matrix, JPH_Mat44 *result);
JPH_CAPI void JPH_Mat44_Transposed(const JPH_Mat44 *matrix, JPH_Mat44 *result);

JPH_CAPI void JPH_Mat44_GetAxisX(const JPH_Mat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_Mat44_GetAxisY(const JPH_Mat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_Mat44_GetAxisZ(const JPH_Mat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_Mat44_GetTranslation(const JPH_Mat44 *matrix, Vector3 *result);
JPH_CAPI void JPH_Mat44_GetQuaternion(const JPH_Mat44 *matrix, JPH_Quat *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_MAT44_H
