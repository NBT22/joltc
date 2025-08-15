//
// Created by NBT22 on 8/14/25.
//

#ifndef JOLTC_RVEC3_H
#define JOLTC_RVEC3_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Vector3.h>

#ifdef JPH_DOUBLE_PRECISION
#include <math.h>

typedef struct JPH_RVec3
{
        double x;
        double y;
        double z;
} JPH_RVec3;

static const JPH_RVec3 JPH_RVec3_Zero = {0.0, 0.0, 0.0};
static const JPH_RVec3 JPH_RVec3_One = {1.0, 1.0, 1.0};
static const JPH_RVec3 JPH_RVec3_NaN = {nan(), nan(), nan()};
static const JPH_RVec3 JPH_RVec3_AxisX = {1.0f, 0.0f, 0.0f};
static const JPH_RVec3 JPH_RVec3_AxisY = {0.0f, 1.0f, 0.0f};
static const JPH_RVec3 JPH_RVec3_AxisZ = {0.0f, 0.0f, 1.0f};
static const JPH_RVec3 JPH_RVec3_Forward = {0.0f, 0.0f, -1.0f};
#else
typedef Vector3 JPH_RVec3;

static const JPH_RVec3 JPH_RVec3_Zero = Vector3_Zero;
static const JPH_RVec3 JPH_RVec3_One = Vector3_One;
static const JPH_RVec3 JPH_RVec3_NaN = Vector3_NaN;
static const JPH_RVec3 JPH_RVec3_AxisX = Vector3_AxisX;
static const JPH_RVec3 JPH_RVec3_AxisY = Vector3_AxisY;
static const JPH_RVec3 JPH_RVec3_AxisZ = Vector3_AxisZ;
static const JPH_RVec3 JPH_RVec3_Forward = Vector3_Forward;
#endif

JPH_CAPI bool JPH_RVec3_IsClose(const JPH_RVec3 *v1, const JPH_RVec3 *v2, float maxDistanceSquared);
/**
 * Test if vector is near zero
 * @param vector The vector to check
 * @param maxDistSq Defaults to 1.0e-12f in Jolt
 * @return True if the vector has a magnitude which is nearly zero
 */
JPH_CAPI bool JPH_RVec3_IsNearZero(const JPH_RVec3 *vector, float maxDistSq);
JPH_CAPI bool JPH_RVec3_IsNormalized(const JPH_RVec3 *vector, float tolerance);
JPH_CAPI bool JPH_RVec3_IsNaN(const JPH_RVec3 *vector);

JPH_CAPI void JPH_RVec3_Negate(const JPH_RVec3 *vector, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_Normalized(const JPH_RVec3 *vector, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_Cross(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_Abs(const JPH_RVec3 *vector, JPH_RVec3 *result);

JPH_CAPI float JPH_RVec3_Length(const JPH_RVec3 *vector);
JPH_CAPI float JPH_RVec3_LengthSquared(const JPH_RVec3 *vector);

JPH_CAPI void JPH_RVec3_DotProduct(const JPH_RVec3 *v1, const JPH_RVec3 *v2, float *result);
JPH_CAPI void JPH_RVec3_Normalize(const JPH_RVec3 *vector, JPH_RVec3 *result);

JPH_CAPI void JPH_RVec3_Add(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_Subtract(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_Multiply(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_MultiplyScalar(const JPH_RVec3 *vector, float scalar, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_Divide(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result);
JPH_CAPI void JPH_RVec3_DivideScalar(const JPH_RVec3 *vector, float scalar, JPH_RVec3 *result);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_RVEC3_H
