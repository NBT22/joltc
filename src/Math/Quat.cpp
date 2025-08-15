//
// Created by NBT22 on 8/14/25.
//

#include <joltc/Math/Quat.h>
#include <joltc/Math/Vector3.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
#include <Math/Quat.hpp>
#include <Math/Vector3.hpp>

void JPH_Quat_FromTo(const Vector3 *from, const Vector3 *to, JPH_Quat *result)
{
    JPH_ASSERT(from);
    JPH_ASSERT(to);
    JPH_ASSERT(result);

    FromJolt(JPH::Quat::sFromTo(ToJolt(from), ToJolt(to)), result);
}

void JPH_Quat_GetAxisAngle(const JPH_Quat *quat, Vector3 *outAxis, float *outAngle)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(outAxis);
    JPH_ASSERT(outAngle);

    JPH::Vec3 joltAxis{};
    const JPH::Quat joltQuat = ToJolt(quat);
    joltQuat.GetAxisAngle(joltAxis, *outAngle);
    FromJolt(joltAxis, outAxis);
}

void JPH_Quat_GetEulerAngles(const JPH_Quat *quat, Vector3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).GetEulerAngles(), result);
}

void JPH_Quat_RotateAxisX(const JPH_Quat *quat, Vector3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).RotateAxisX(), result);
}

void JPH_Quat_RotateAxisY(const JPH_Quat *quat, Vector3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).RotateAxisY(), result);
}

void JPH_Quat_RotateAxisZ(const JPH_Quat *quat, Vector3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).RotateAxisZ(), result);
}

void JPH_Quat_Inversed(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).Inversed(), result);
}

void JPH_Quat_GetPerpendicular(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).GetPerpendicular(), result);
}

float JPH_Quat_GetRotationAngle(const JPH_Quat *quat, const Vector3 *axis)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(axis);

    return ToJolt(quat).GetRotationAngle(ToJolt(axis));
}

void JPH_Quat_Rotation(const Vector3 *inAxis, float inAngle, JPH_Quat *result)
{
    JPH_ASSERT(inAxis);
    JPH_ASSERT(result);

    FromJolt(JPH::Quat::sRotation(ToJolt(inAxis), inAngle), result);
}

void JPH_Quat_FromEulerAngles(const Vector3 *angles, JPH_Quat *result)
{
    JPH_ASSERT(angles);
    JPH_ASSERT(result);

    FromJolt(JPH::Quat::sEulerAngles(ToJolt(angles)), result);
}
void JPH_Quat_Add(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result)
{
    JPH_ASSERT(q1);
    JPH_ASSERT(q2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(q1) + ToJolt(q2), result);
}

void JPH_Quat_Subtract(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result)
{
    JPH_ASSERT(q1);
    JPH_ASSERT(q2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(q1) - ToJolt(q2), result);
}

void JPH_Quat_Multiply(const JPH_Quat *q1, const JPH_Quat *q2, JPH_Quat *result)
{
    JPH_ASSERT(q1);
    JPH_ASSERT(q2);
    JPH_ASSERT(result);

    FromJolt(ToJolt(q1) * ToJolt(q2), result);
}

void JPH_Quat_MultiplyScalar(const JPH_Quat *quat, float scalar, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat) * scalar, result);
}

void JPH_Quat_DivideScalar(const JPH_Quat *quat, float scalar, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(scalar != 0.0f);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat) / scalar, result);
}

void JPH_Quat_Dot(const JPH_Quat *q1, const JPH_Quat *q2, float *result)
{
    JPH_ASSERT(q1);
    JPH_ASSERT(q2);
    JPH_ASSERT(result);

    *result = ToJolt(q1).Dot(ToJolt(q2));
}

void JPH_Quat_Conjugated(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).Conjugated(), result);
}

void JPH_Quat_GetTwist(const JPH_Quat *quat, const Vector3 *axis, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(axis);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).GetTwist(ToJolt(axis)), result);
}

void JPH_Quat_GetSwingTwist(const JPH_Quat *quat, JPH_Quat *outSwing, JPH_Quat *outTwist)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(outSwing);
    JPH_ASSERT(outTwist);

    JPH::Quat swing{};
    JPH::Quat twist{};
    ToJolt(quat).GetSwingTwist(swing, twist);
    FromJolt(swing, outSwing);
    FromJolt(twist, outTwist);
}

void JPH_Quat_Lerp(const JPH_Quat *from, const JPH_Quat *to, float fraction, JPH_Quat *result)
{
    JPH_ASSERT(from);
    JPH_ASSERT(to);
    JPH_ASSERT(result);

    FromJolt(ToJolt(from).LERP(ToJolt(to), fraction), result);
}

void JPH_Quat_Slerp(const JPH_Quat *from, const JPH_Quat *to, float fraction, JPH_Quat *result)
{
    JPH_ASSERT(from);
    JPH_ASSERT(to);
    JPH_ASSERT(result);

    FromJolt(ToJolt(from).SLERP(ToJolt(to), fraction), result);
}

void JPH_Quat_Rotate(const JPH_Quat *quat, const Vector3 *vector, Vector3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat) * ToJolt(vector), result);
}

void JPH_Quat_InverseRotate(const JPH_Quat *quat, const Vector3 *vector, Vector3 *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(vector);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).InverseRotate(ToJolt(vector)), result);
}

float JPH_Quat_LengthSq(const JPH_Quat *quat)
{
    JPH_ASSERT(quat);

    return ToJolt(quat).LengthSq();
}

float JPH_Quat_Length(const JPH_Quat *quat)
{
    JPH_ASSERT(quat);

    return ToJolt(quat).Length();
}

void JPH_Quat_Normalized(const JPH_Quat *quat, JPH_Quat *result)
{
    JPH_ASSERT(quat);
    JPH_ASSERT(result);

    FromJolt(ToJolt(quat).Normalized(), result);
}
