//
// Created by NBT22 on 8/14/25.
//

#ifndef JOLTC_VECTOR3_H
#define JOLTC_VECTOR3_H

#ifdef __cplusplus
extern "C"
{
#endif

#if !defined(__cpp_constexpr) && __STDC_VERSION__ < 202311L
#define constexpr const
#endif

#include <math.h>
#include <stdbool.h>

typedef struct Vector3
{
        float x;
        float y;
        float z;
} Vector3;

static constexpr Vector3 Vector3_Zero = {0.0f, 0.0f, 0.0f};
static constexpr Vector3 Vector3_One = {1.0f, 1.0f, 1.0f};
static constexpr Vector3 Vector3_NaN = {NAN, NAN, NAN};
static constexpr Vector3 Vector3_AxisX = {1.0f, 0.0f, 0.0f};
static constexpr Vector3 Vector3_AxisY = {0.0f, 1.0f, 0.0f};
static constexpr Vector3 Vector3_AxisZ = {0.0f, 0.0f, 1.0f};
static constexpr Vector3 Vector3_Forward = {0.0f, 0.0f, -1.0f};

JPH_CAPI bool Vector3_IsClose(const Vector3 *v1, const Vector3 *v2, float maxDistanceSquared);
/**
 * Test if vector is near zero
 * @param vector The vector to check
 * @param maxDistSq Defaults to 1.0e-12f in Jolt
 * @return True if the vector has a magnitude which is nearly zero
 */
JPH_CAPI bool Vector3_IsNearZero(const Vector3 *vector, float maxDistSq);
JPH_CAPI bool Vector3_IsNormalized(const Vector3 *vector, float tolerance);
JPH_CAPI bool Vector3_IsNaN(const Vector3 *vector);

JPH_CAPI void Vector3_Negate(const Vector3 *vector, Vector3 *result);
JPH_CAPI void Vector3_Normalized(const Vector3 *vector, Vector3 *result);
JPH_CAPI void Vector3_Cross(const Vector3 *v1, const Vector3 *v2, Vector3 *result);
JPH_CAPI void Vector3_Abs(const Vector3 *vector, Vector3 *result);

JPH_CAPI float Vector3_Length(const Vector3 *vector);
JPH_CAPI float Vector3_LengthSquared(const Vector3 *vector);

JPH_CAPI void Vector3_DotProduct(const Vector3 *v1, const Vector3 *v2, float *result);

JPH_CAPI void Vector3_Add(const Vector3 *v1, const Vector3 *v2, Vector3 *result);
JPH_CAPI void Vector3_Subtract(const Vector3 *v1, const Vector3 *v2, Vector3 *result);
JPH_CAPI void Vector3_Multiply(const Vector3 *v1, const Vector3 *v2, Vector3 *result);
JPH_CAPI void Vector3_MultiplyScalar(const Vector3 *vector, float scalar, Vector3 *result);
JPH_CAPI void Vector3_Divide(const Vector3 *v1, const Vector3 *v2, Vector3 *result);
JPH_CAPI void Vector3_DivideScalar(const Vector3 *vector, float scalar, Vector3 *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_VECTOR3_H
