//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/Shape/Shape.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>


static inline const JPH::ShapeSettings &AsShapeSettings(const JPH_ShapeSettings &t)
{
    return reinterpret_cast<const JPH::ShapeSettings &>(t);
}
static inline const JPH::ShapeSettings *AsShapeSettings(const JPH_ShapeSettings *t)
{
    return reinterpret_cast<const JPH::ShapeSettings *>(t);
}
static inline JPH::ShapeSettings &AsShapeSettings(JPH_ShapeSettings &t)
{
    return reinterpret_cast<JPH::ShapeSettings &>(t);
}
static inline JPH::ShapeSettings *AsShapeSettings(JPH_ShapeSettings *t)
{
    return reinterpret_cast<JPH::ShapeSettings *>(t);
}
static inline const JPH_ShapeSettings &ToShapeSettings(const JPH::ShapeSettings &t)
{
    return reinterpret_cast<const JPH_ShapeSettings &>(t);
}
static inline const JPH_ShapeSettings *ToShapeSettings(const JPH::ShapeSettings *t)
{
    return reinterpret_cast<const JPH_ShapeSettings *>(t);
}
static inline JPH_ShapeSettings &ToShapeSettings(JPH::ShapeSettings &t)
{
    return reinterpret_cast<JPH_ShapeSettings &>(t);
}
static inline JPH_ShapeSettings *ToShapeSettings(JPH::ShapeSettings *t)
{
    return reinterpret_cast<JPH_ShapeSettings *>(t);
}

static inline const JPH::Shape &AsShape(const JPH_Shape &t)
{
    return reinterpret_cast<const JPH::Shape &>(t);
}
static inline const JPH::Shape *AsShape(const JPH_Shape *t)
{
    return reinterpret_cast<const JPH::Shape *>(t);
}
static inline JPH::Shape &AsShape(JPH_Shape &t)
{
    return reinterpret_cast<JPH::Shape &>(t);
}
static inline JPH::Shape *AsShape(JPH_Shape *t)
{
    return reinterpret_cast<JPH::Shape *>(t);
}
static inline const JPH_Shape &ToShape(const JPH::Shape &t)
{
    return reinterpret_cast<const JPH_Shape &>(t);
}
static inline const JPH_Shape *ToShape(const JPH::Shape *t)
{
    return reinterpret_cast<const JPH_Shape *>(t);
}
static inline JPH_Shape &ToShape(JPH::Shape &t)
{
    return reinterpret_cast<JPH_Shape &>(t);
}
static inline JPH_Shape *ToShape(JPH::Shape *t)
{
    return reinterpret_cast<JPH_Shape *>(t);
}
