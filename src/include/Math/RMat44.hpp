//
// Created by NBT22 on 8/15/25.
//

#pragma once

#ifdef JPH_DOUBLE_PRECISION
#include <joltc/Math/RMat44.h>
#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

static inline JPH::RMat44 ToJolt(const JPH_RMat44 *matrix)
{
    JPH::RMat44 result{};
    result.SetColumn4(0, JPH::Vec4(matrix->m11, matrix->m12, matrix->m13, matrix->m14));
    result.SetColumn4(1, JPH::Vec4(matrix->m21, matrix->m22, matrix->m23, matrix->m24));
    result.SetColumn4(2, JPH::Vec4(matrix->m31, matrix->m32, matrix->m33, matrix->m34));
    result.SetColumn4(3, JPH::Vec4(matrix->m41, matrix->m42, matrix->m43, matrix->m44));
    return result;
}

static inline void FromJolt(const JPH::RMat44 &matrix, JPH_RMat44 *result)
{
    const JPH::Vec4 column0 = matrix.GetColumn4(0);
    const JPH::Vec4 column1 = matrix.GetColumn4(1);
    const JPH::Vec4 column2 = matrix.GetColumn4(2);
    const JPH::Vec3 translation = matrix.GetTranslation();

    result->m11 = column0.GetX();
    result->m12 = column0.GetY();
    result->m13 = column0.GetZ();
    result->m14 = column0.GetW();

    result->m21 = column1.GetX();
    result->m22 = column1.GetY();
    result->m23 = column1.GetZ();
    result->m24 = column1.GetW();

    result->m31 = column2.GetX();
    result->m32 = column2.GetY();
    result->m33 = column2.GetZ();
    result->m34 = column2.GetW();

    result->m41 = translation.GetX();
    result->m42 = translation.GetY();
    result->m43 = translation.GetZ();
    result->m44 = 1.0;
}
#else
#include <Math/Mat44.hpp> // NOLINT(*-include-cleaner)
#endif
