//
// Created by NBT22 on 8/14/25.
//

#include <joltc/Math/Mat44.h>
#include <joltc/Math/Quat.h>
#include <joltc/Math/Vector3.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Math/Mat44.hpp>
#include <Math/Quat.hpp>
#include <Math/Vector3.hpp>

void JPH_Mat44_Add(const JPH_Mat44 *m1, const JPH_Mat44 *m2, JPH_Mat44 *result)
{
    JPH_ASSERT(m1);
    JPH_ASSERT(m2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(m1) + ToJolt(m2), result);
}

void JPH_Mat44_Subtract(const JPH_Mat44 *m1, const JPH_Mat44 *m2, JPH_Mat44 *result)
{
    JPH_ASSERT(m1);
    JPH_ASSERT(m2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(m1) - ToJolt(m2), result);
}

void JPH_Mat44_Multiply(const JPH_Mat44 *m1, const JPH_Mat44 *m2, JPH_Mat44 *result)
{
    JPH_ASSERT(m1);
    JPH_ASSERT(m2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(m1) * ToJolt(m2), result);
}

void JPH_Mat44_MultiplyScalar(const JPH_Mat44 *matrix, float scalar, JPH_Mat44 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix) * scalar, result);
}

void JPH_Mat44_MultiplyVector3(const JPH_Mat44 *left, const Vector3 *right, Vector3 *result)
{
    JPH_ASSERT(left);
    JPH_ASSERT(right);
    JPH_ASSERT(result);

    FromJolt(ToJolt(left) * ToJolt(right), result);
}

void JPH_Mat44_Rotation(const JPH_Quat *rotation, JPH_Mat44 *result)
{
    JPH_ASSERT(rotation);
    JPH_ASSERT(result);

    FromJolt(JPH::Mat44::sRotation(ToJolt(rotation)), result);
}

void JPH_Mat4_RotationAxisAngle(const Vector3 *axis, float angle, JPH_Mat44 *result)
{
    JPH_ASSERT(axis);
    JPH_ASSERT(result);

    FromJolt(JPH::Mat44::sRotation(ToJolt(axis), angle), result);
}

void JPH_Mat44_Translation(const Vector3 *translation, JPH_Mat44 *result)
{
    JPH_ASSERT(translation);
    JPH_ASSERT(result);

    FromJolt(JPH::Mat44::sTranslation(ToJolt(translation)), result);
}

void JPH_Mat44_RotationTranslation(const JPH_Quat *rotation, const Vector3 *translation, JPH_Mat44 *result)
{
    JPH_ASSERT(rotation);
    JPH_ASSERT(translation);
    JPH_ASSERT(result);

    FromJolt(JPH::Mat44::sRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_Mat44_InverseRotationTranslation(const JPH_Quat *rotation, const Vector3 *translation, JPH_Mat44 *result)
{
    JPH_ASSERT(rotation);
    JPH_ASSERT(translation);
    JPH_ASSERT(result);

    FromJolt(JPH::Mat44::sInverseRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_Mat44_Scale(const Vector3 *scale, JPH_Mat44 *result)
{
    JPH_ASSERT(scale);
    JPH_ASSERT(result);

    FromJolt(JPH::Mat44::sScale(ToJolt(scale)), result);
}

void JPH_Mat44_Transposed(const JPH_Mat44 *matrix, JPH_Mat44 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).Transposed(), result);
}

void JPH_Mat44_Inversed(const JPH_Mat44 *matrix, JPH_Mat44 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).Inversed(), result);
}

void JPH_Mat44_GetAxisX(const JPH_Mat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetAxisX(), result);
}

void JPH_Mat44_GetAxisY(const JPH_Mat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetAxisY(), result);
}

void JPH_Mat44_GetAxisZ(const JPH_Mat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetAxisZ(), result);
}

void JPH_Mat44_GetTranslation(const JPH_Mat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetTranslation(), result);
}

void JPH_Mat44_GetQuaternion(const JPH_Mat44 *matrix, JPH_Quat *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetQuaternion(), result);
}
