//
// Created by NBT22 on 8/14/25.
//

#include <joltc/Math/Vector3.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Math/Vector3.hpp>

bool Vector3_IsClose(const Vector3 *v1, const Vector3 *v2, const float maxDistanceSquared)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);

    return ToJolt(v1).IsClose(ToJolt(v2), maxDistanceSquared);
}

bool Vector3_IsNearZero(const Vector3 *vector, const float maxDistSq)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).IsNearZero(maxDistSq);
}

bool Vector3_IsNormalized(const Vector3 *vector, const float tolerance)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).IsNormalized(tolerance);
}

bool Vector3_IsNaN(const Vector3 *vector)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).IsNaN();
}

void Vector3_Negate(const Vector3 *vector, Vector3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(-ToJolt(vector), result);
}

void Vector3_Normalized(const Vector3 *vector, Vector3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector).Normalized(), result);
}

void Vector3_Cross(const Vector3 *v1, const Vector3 *v2, Vector3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1).Cross(ToJolt(v2)), result);
}

void Vector3_Abs(const Vector3 *vector, Vector3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector).Abs(), result);
}

float Vector3_Length(const Vector3 *vector)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).Length();
}

float Vector3_LengthSquared(const Vector3 *vector)
{
    JPH_ASSERT(vector);

    return ToJolt(vector).LengthSq();
}

void Vector3_Multiply(const Vector3 *v1, const Vector3 *v2, Vector3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) * ToJolt(v2), result);
}

void Vector3_MultiplyScalar(const Vector3 *vector, const float scalar, Vector3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector) * scalar, result);
}

void Vector3_Divide(const Vector3 *v1, const Vector3 *v2, Vector3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) / ToJolt(v2), result);
}

void Vector3_DivideScalar(const Vector3 *vector, const float scalar, Vector3 *result)
{
    JPH_ASSERT(vector);
    JPH_ASSERT(scalar != 0.0f);
    JPH_ASSERT(result);

    FromJolt(ToJolt(vector) / scalar, result);
}

void Vector3_DotProduct(const Vector3 *v1, const Vector3 *v2, float *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    *result = ToJolt(v1).Dot(ToJolt(v2));
}

void Vector3_Add(const Vector3 *v1, const Vector3 *v2, Vector3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) + ToJolt(v2), result);
}

void Vector3_Subtract(const Vector3 *v1, const Vector3 *v2, Vector3 *result)
{
    JPH_ASSERT(v1);
    JPH_ASSERT(v2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(v1) - ToJolt(v2), result);
}
