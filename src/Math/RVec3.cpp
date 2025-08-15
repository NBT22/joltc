//
// Created by NBT22 on 8/14/25.
//

#include <joltc/Math/RVec3.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Math/RVec3.hpp>

bool JPH_RVec3_IsClose(const JPH_RVec3 *v1, const JPH_RVec3 *v2, const float maxDistanceSquared)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);

    return ToJolt(v1).IsClose(ToJolt(v2), maxDistanceSquared);
}

bool JPH_RVec3_IsNearZero(const JPH_RVec3 *vector, const float maxDistSq)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).IsNearZero(maxDistSq);
}

bool JPH_RVec3_IsNormalized(const JPH_RVec3 *vector, const float tolerance)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).IsNormalized(tolerance);
}

bool JPH_RVec3_IsNaN(const JPH_RVec3 *vector)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).IsNaN();
}

void JPH_RVec3_Negate(const JPH_RVec3 *vector, JPH_RVec3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(-ToJolt(vector), result);
}

void JPH_RVec3_Normalized(const JPH_RVec3 *vector, JPH_RVec3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector).Normalized(), result);
}

void JPH_RVec3_Cross(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1).Cross(ToJolt(v2)), result);
}

void JPH_RVec3_Abs(const JPH_RVec3 *vector, JPH_RVec3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector).Abs(), result);
}

float JPH_RVec3_Length(const JPH_RVec3 *vector)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).Length();
}

float JPH_RVec3_LengthSquared(const JPH_RVec3 *vector)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).LengthSq();
}

void JPH_RVec3_Multiply(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) * ToJolt(v2), result);
}

void JPH_RVec3_MultiplyScalar(const JPH_RVec3 *vector, const float scalar, JPH_RVec3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector) * scalar, result);
}

void JPH_RVec3_Divide(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) / ToJolt(v2), result);
}

void JPH_RVec3_DivideScalar(const JPH_RVec3 *vector, const float scalar, JPH_RVec3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(scalar != 0.0f);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector) / scalar, result);
}

void JPH_RVec3_DotProduct(const JPH_RVec3 *v1, const JPH_RVec3 *v2, float *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    *result = ToJolt(v1).Dot(ToJolt(v2));
}

void JPH_RVec3_Normalize(const JPH_RVec3 *vector, JPH_RVec3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector).Normalized(), result);
}

void JPH_RVec3_Add(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) + ToJolt(v2), result);
}

void JPH_RVec3_Subtract(const JPH_RVec3 *v1, const JPH_RVec3 *v2, JPH_RVec3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) - ToJolt(v2), result);
}
