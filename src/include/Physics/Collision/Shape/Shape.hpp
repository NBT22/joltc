//
// Created by NBT22 on 8/15/25.
//

#pragma once

#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Physics/Collision/CollideShape.hpp>

class CollideShapeCollectorCallback final: public JPH::CollideShapeCollector
{
    public:
        CollideShapeCollectorCallback(JPH_CollideShapeCollectorCallback *proc, void *userData):
            proc(proc),
            userData(userData)
        {}

        void AddHit(const JPH::CollideShapeResult &result) override
        {
            const JPH_CollideShapeResult *hit = nullptr;
            FromJolt(&result, hit);

            const float fraction = proc(userData, hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CollideShapeCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
};

class CastShapeCollectorCallback final: public JPH::CastShapeCollector
{
    public:
        CastShapeCollectorCallback(JPH_CastShapeCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::ShapeCastResult &result) override
        {
            JPH_ShapeCastResult hit{};
            FromJolt(result.mContactPointOn1, &hit.contactPointOn1);
            FromJolt(result.mContactPointOn2, &hit.contactPointOn2);
            FromJolt(result.mPenetrationAxis, &hit.penetrationAxis);
            hit.penetrationDepth = result.mPenetrationDepth;
            hit.subShapeID1 = result.mSubShapeID1.GetValue();
            hit.subShapeID2 = result.mSubShapeID2.GetValue();
            hit.bodyID2 = result.mBodyID2.GetIndexAndSequenceNumber();
            hit.fraction = result.mFraction;
            hit.isBackFaceHit = result.mIsBackFaceHit;

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CastShapeCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
};


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
