//
// Created by NBT22 on 8/15/25.
//

#include <joltc/Math/Quat.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/Vector3.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Math/Real.h>
#include <Math/Quat.hpp>
#include <Math/RMat44.hpp>
#include <Math/Vector3.hpp>

void JPH_RMat44_Add(const JPH_RMat44 *m1, const JPH_RMat44 *m2, JPH_RMat44 *result)
{
    JPH_ASSERT(m1);
    JPH_ASSERT(m2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(m1) + ToJolt(m2), result);
}

void JPH_RMat44_Subtract(const JPH_RMat44 *m1, const JPH_RMat44 *m2, JPH_RMat44 *result)
{
    JPH_ASSERT(m1);
    JPH_ASSERT(m2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(m1) - ToJolt(m2), result);
}

void JPH_RMat44_Multiply(const JPH_RMat44 *m1, const JPH_RMat44 *m2, JPH_RMat44 *result)
{
    JPH_ASSERT(m1);
    JPH_ASSERT(m2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(m1) * ToJolt(m2), result);
}

void JPH_RMat44_MultiplyScalar(const JPH_RMat44 *matrix, float scalar, JPH_RMat44 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix) * scalar, result);
}

void JPH_RMat44_Rotation(const JPH_Quat *rotation, JPH_RMat44 *result)
{
    JPH_ASSERT(rotation);
    JPH_ASSERT(result);

    FromJolt(JPH::RMat44::sRotation(ToJolt(rotation)), result);
}

void JPH_RMat44_Translation(const Vector3 *translation, JPH_RMat44 *result)
{
    JPH_ASSERT(translation);
    JPH_ASSERT(result);

    FromJolt(JPH::RMat44::sTranslation(ToJolt(translation)), result);
}

void JPH_RMat44_RotationTranslation(const JPH_Quat *rotation, const Vector3 *translation, JPH_RMat44 *result)
{
    JPH_ASSERT(rotation);
    JPH_ASSERT(translation);
    JPH_ASSERT(result);

    FromJolt(JPH::RMat44::sRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_RMat44_InverseRotationTranslation(const JPH_Quat *rotation, const Vector3 *translation, JPH_RMat44 *result)
{
    JPH_ASSERT(rotation);
    JPH_ASSERT(translation);
    JPH_ASSERT(result);

    FromJolt(JPH::RMat44::sInverseRotationTranslation(ToJolt(rotation), ToJolt(translation)), result);
}

void JPH_RMat44_Scale(const Vector3 *scale, JPH_RMat44 *result)
{
    JPH_ASSERT(scale);
    JPH_ASSERT(result);

    FromJolt(JPH::RMat44::sScale(ToJolt(scale)), result);
}

void JPH_RMat44_Transposed(const JPH_RMat44 *matrix, JPH_RMat44 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).Transposed(), result);
}

void JPH_RMat44_Inversed(const JPH_RMat44 *matrix, JPH_RMat44 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).Inversed(), result);
}

void JPH_RMat44_GetAxisX(const JPH_RMat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetAxisX(), result);
}

void JPH_RMat44_GetAxisY(const JPH_RMat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetAxisY(), result);
}

void JPH_RMat44_GetAxisZ(const JPH_RMat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetAxisZ(), result);
}

void JPH_RMat44_GetTranslation(const JPH_RMat44 *matrix, Vector3 *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetTranslation(), result);
}

void JPH_RMat44_GetQuaternion(const JPH_RMat44 *matrix, JPH_Quat *result)
{
    JPH_ASSERT(matrix);
    JPH_ASSERT(result);

    FromJolt(ToJolt(matrix).GetQuaternion(), result);
}
