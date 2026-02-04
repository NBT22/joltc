//
// Created by NBT22 on 8/14/25.
//

#ifndef JOLTC_QUAT_H
#define JOLTC_QUAT_H

#ifdef __cplusplus
extern "C"
{
#endif

#if !defined(__cpp_constexpr) && __STDC_VERSION__ < 202311L
#define constexpr const
#endif

#include <joltc/Math/Vector3.h>

typedef struct JPH_Quat
{
        float x;
        float y;
        float z;
        float w;
} JPH_Quat;

static constexpr JPH_Quat JPH_Quat_Zero = {0.0f, 0.0f, 0.0f, 0.0f};
static constexpr JPH_Quat JPH_Quat_Identity = {0.0f, 0.0f, 0.0f, 1.0f};

JPH_CAPI void JPH_Quat_FromTo(const Vector3 *from, const Vector3 *to, JPH_Quat *result);
JPH_CAPI void JPH_Quat_GetAxisAngle(const JPH_Quat *quat, Vector3 *outAxis, float *outAngle);
JPH_CAPI void JPH_Quat_GetEulerAngles(const JPH_Quat *quat, Vector3 *result);
JPH_CAPI void JPH_Quat_RotateAxisX(const JPH_Quat *quat, Vector3 *result);
JPH_CAPI void JPH_Quat_RotateAxisY(const JPH_Quat *quat, Vector3 *result);
JPH_CAPI void JPH_Quat_RotateAxisZ(const JPH_Quat *quat, Vector3 *result);
JPH_CAPI void JPH_Quat_Inversed(const JPH_Quat *quat, JPH_Quat *result);
JPH_CAPI void JPH_Quat_GetPerpendicular(const JPH_Quat *quat, JPH_Quat *result);
JPH_CAPI float JPH_Quat_GetRotationAngle(const JPH_Quat *quat, const Vector3 *axis);
JPH_CAPI void JPH_Quat_Rotation(const Vector3 *inAxis, float inAngle, JPH_Quat *result);
JPH_CAPI void JPH_Quat_FromEulerAngles(const Vector3 *angles, JPH_Quat *result);

JPH_CAPI void JPH_Quat_Add(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result);
JPH_CAPI void JPH_Quat_Subtract(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result);
JPH_CAPI void JPH_Quat_Multiply(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result);
JPH_CAPI void JPH_Quat_MultiplyScalar(const JPH_Quat *quat, float scalar, JPH_Quat *result);
JPH_CAPI void JPH_Quat_DivideScalar(const JPH_Quat *quat, float scalar, JPH_Quat *result);
JPH_CAPI void JPH_Quat_Dot(const JPH_Quat *q1, const JPH_Quat *q2, float *result);

JPH_CAPI void JPH_Quat_Conjugated(const JPH_Quat *quat, JPH_Quat *result);
JPH_CAPI void JPH_Quat_GetTwist(const JPH_Quat *quat, const Vector3 *axis, JPH_Quat *result);
JPH_CAPI void JPH_Quat_GetSwingTwist(const JPH_Quat *quat, JPH_Quat *outSwing, JPH_Quat *outTwist);
JPH_CAPI void JPH_Quat_Lerp(const JPH_Quat *from, const JPH_Quat *to, float fraction, JPH_Quat *result);
JPH_CAPI void JPH_Quat_Slerp(const JPH_Quat *from, const JPH_Quat *to, float fraction, JPH_Quat *result);
JPH_CAPI void JPH_Quat_Rotate(const JPH_Quat *quat, const Vector3 *vector, Vector3 *result);
JPH_CAPI void JPH_Quat_InverseRotate(const JPH_Quat *quat, const Vector3 *vector, Vector3 *result);

JPH_CAPI float JPH_Quat_LengthSq(const JPH_Quat *quat);
JPH_CAPI float JPH_Quat_Length(const JPH_Quat *quat);
JPH_CAPI void JPH_Quat_Normalized(const JPH_Quat *quat, JPH_Quat *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_QUAT_H
