//
// Created by NBT22 on 8/15/25.
//

#include <cstdint>
#include <joltc/enums.h>
#include <joltc/Geometry/AABox.h>
#include <joltc/Math/Mat44.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/MassProperties.h>
#include <joltc/Physics/Collision/CastResult.h>
#include <joltc/Physics/Collision/CollidePointResult.h>
#include <joltc/Physics/Collision/PhysicsMaterial.h>
#include <joltc/Physics/Collision/RayCast.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>
#include <joltc/Physics/Collision/ShapeFilter.h>
#include <Jolt/Jolt.h>
#include <Geometry/AABox.hpp>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Core/Result.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Math/Mat44.hpp>
#include <Math/RMat44.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/MassProperties.hpp>
#include <Physics/Collision/PhysicsMaterial.hpp>
#include <Physics/Collision/RayCast.hpp>
#include <Physics/Collision/Shape/Shape.hpp>
#include <Physics/Collision/ShapeFilter.hpp>

void JPH_ShapeSettings_Destroy(JPH_ShapeSettings *settings)
{
    if (settings != nullptr)
    {
        const JPH::ShapeSettings *const joltSettings = reinterpret_cast<JPH::ShapeSettings *>(settings);
        joltSettings->Release();
    }
}

uint64_t JPH_ShapeSettings_GetUserData(const JPH_ShapeSettings *settings)
{
    return reinterpret_cast<const JPH::ShapeSettings *>(settings)->mUserData;
}

void JPH_ShapeSettings_SetUserData(JPH_ShapeSettings *settings, const uint64_t userData)
{
    reinterpret_cast<JPH::ShapeSettings *>(settings)->mUserData = userData;
}


void JPH_Shape_Destroy(JPH_Shape *shape)
{
    AsShape(shape)->Release();
}

JPH_ShapeType JPH_Shape_GetType(const JPH_Shape *shape)
{
    return static_cast<JPH_ShapeType>(reinterpret_cast<const JPH::Shape *>(shape)->GetType());
}

JPH_ShapeSubType JPH_Shape_GetSubType(const JPH_Shape *shape)
{
    return static_cast<JPH_ShapeSubType>(reinterpret_cast<const JPH::Shape *>(shape)->GetSubType());
}

uint64_t JPH_Shape_GetUserData(const JPH_Shape *shape)
{
    return reinterpret_cast<const JPH::Shape *>(shape)->GetUserData();
}

void JPH_Shape_SetUserData(JPH_Shape *shape, const uint64_t userData)
{
    reinterpret_cast<JPH::Shape *>(shape)->SetUserData(userData);
}

bool JPH_Shape_MustBeStatic(const JPH_Shape *shape)
{
    return reinterpret_cast<const JPH::Shape *>(shape)->MustBeStatic();
}

void JPH_Shape_GetCenterOfMass(const JPH_Shape *shape, Vector3 *result)
{
    FromJolt(AsShape(shape)->GetCenterOfMass(), result);
}

void JPH_Shape_GetLocalBounds(const JPH_Shape *shape, JPH_AABox *result)
{
    FromJolt(AsShape(shape)->GetLocalBounds(), result);
}

uint32_t JPH_Shape_GetSubShapeIDBitsRecursive(const JPH_Shape *shape)
{
    return AsShape(shape)->GetSubShapeIDBitsRecursive();
}

void JPH_Shape_GetWorldSpaceBounds(const JPH_Shape *shape,
                                   const JPH_RMat44 *centerOfMassTransform,
                                   const Vector3 *scale,
                                   JPH_AABox *result)
{
    const JPH::AABox bounds = AsShape(shape)->GetWorldSpaceBounds(ToJolt(centerOfMassTransform), ToJolt(scale));
    FromJolt(bounds, result);
}

float JPH_Shape_GetInnerRadius(const JPH_Shape *shape)
{
    return AsShape(shape)->GetInnerRadius();
}

void JPH_Shape_GetMassProperties(const JPH_Shape *shape, JPH_MassProperties *result)
{
    FromJolt(AsShape(shape)->GetMassProperties(), result);
}

const JPH_Shape *JPH_Shape_GetLeafShape(const JPH_Shape *shape,
                                        const JPH_SubShapeID subShapeID,
                                        JPH_SubShapeID *remainder)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    JPH::SubShapeID joltRemainder = JPH::SubShapeID();
    const JPH::Shape *leaf = AsShape(shape)->GetLeafShape(joltSubShapeID, joltRemainder);
    *remainder = joltRemainder.GetValue();
    return reinterpret_cast<const JPH_Shape *>(leaf);
}

const JPH_PhysicsMaterial *JPH_Shape_GetMaterial(const JPH_Shape *shape, const JPH_SubShapeID subShapeID)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    return ToPhysicsMaterial(AsShape(shape)->GetMaterial(joltSubShapeID));
}

void JPH_Shape_GetSurfaceNormal(const JPH_Shape *shape,
                                const JPH_SubShapeID subShapeID,
                                const Vector3 *localPosition,
                                Vector3 *normal)
{
    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);
    const JPH::Vec3 joltNormal = AsShape(shape)->GetSurfaceNormal(joltSubShapeID, ToJolt(localPosition));
    FromJolt(joltNormal, normal);
}

void JPH_Shape_GetSupportingFace(const JPH_Shape *shape,
                                 const JPH_SubShapeID subShapeID,
                                 const Vector3 *direction,
                                 const Vector3 *scale,
                                 const JPH_Mat44 *centerOfMassTransform,
                                 JPH_SupportingFace *outVertices)
{
    JPH_ASSERT(shape);
    JPH_ASSERT(subShapeID);
    JPH_ASSERT(direction);
    JPH_ASSERT(scale);
    JPH_ASSERT(centerOfMassTransform);
    JPH_ASSERT(outVertices);

    JPH::SubShapeID joltSubShapeID = JPH::SubShapeID();
    joltSubShapeID.SetValue(subShapeID);

    JPH::Shape::SupportingFace joltFace;
    AsShape(shape)->GetSupportingFace(joltSubShapeID,
                                      ToJolt(direction),
                                      ToJolt(scale),
                                      ToJolt(centerOfMassTransform),
                                      joltFace);

    outVertices->count = joltFace.size();
    JPH_ASSERT(outVertices->count <= 32);

    for (uint32_t i = 0; i < outVertices->count && i < 32; ++i)
    {
        FromJolt(joltFace.at(i), &outVertices->vertices[i]);
    }
}

float JPH_Shape_GetVolume(const JPH_Shape *shape)
{
    return AsShape(shape)->GetVolume();
}

bool JPH_Shape_IsValidScale(const JPH_Shape *shape, const Vector3 *scale)
{
    return AsShape(shape)->IsValidScale(ToJolt(scale));
}

void JPH_Shape_MakeScaleValid(const JPH_Shape *shape, const Vector3 *scale, Vector3 *result)
{
    FromJolt(AsShape(shape)->MakeScaleValid(ToJolt(scale)), result);
}

JPH_Shape *JPH_Shape_ScaleShape(const JPH_Shape *shape, const Vector3 *scale)
{
    const JPH::Result<JPH::Ref<JPH::Shape>> shapeResult = AsShape(shape)->ScaleShape(ToJolt(scale));
    if (!shapeResult.IsValid())
    {
        return nullptr;
    }

    JPH::Shape *scaleShape = shapeResult.Get().GetPtr();
    scaleShape->AddRef();

    return ToShape(scaleShape);
}

bool JPH_Shape_CastRay(const JPH_Shape *shape, const Vector3 *origin, const Vector3 *direction, JPH_RayCastResult *hit)
{
    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    constexpr JPH::SubShapeIDCreator creator;
    JPH::RayCastResult result;

    const bool hadHit = AsShape(shape)->CastRay(ray, creator, result);

    if (hadHit)
    {
        hit->fraction = result.mFraction;
        hit->bodyID = result.mBodyID.GetIndexAndSequenceNumber();
        hit->subShapeID2 = result.mSubShapeID2.GetValue();
    }

    return hadHit;
}

bool JPH_Shape_CastRay2(const JPH_Shape *shape,
                        const Vector3 *origin,
                        const Vector3 *direction,
                        const JPH_RayCastSettings *rayCastSettings,
                        const JPH_CollisionCollectorType collectorType,
                        JPH_CastRayResultCallback *callback,
                        void *userData,
                        const JPH_ShapeFilter *shapeFilter)
{
    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RayCastSettings settings = ToJolt(rayCastSettings);

    constexpr JPH::SubShapeIDCreator creator;
    JPH_RayCastResult hitResult{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            AsShape(shape)->CastRay(ray, settings, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::RayCastResult &hit: collector.mHits)
                {
                    hitResult.fraction = hit.mFraction;
                    hitResult.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    hitResult.subShapeID2 = hit.mSubShapeID2.GetValue();
                    callback(userData, &hitResult);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
            AsShape(shape)->CastRay(ray, settings, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CastRayCollector> collector;
            AsShape(shape)->CastRay(ray, settings, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_Shape_CollidePoint(const JPH_Shape *shape, const Vector3 *point, const JPH_ShapeFilter *shapeFilter)
{
    constexpr JPH::SubShapeIDCreator creator;
    JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> collector;

    AsShape(shape)->CollidePoint(ToJolt(point), creator, collector, ToJolt(shapeFilter));
    return collector.HadHit();
}

bool JPH_Shape_CollidePoint2(const JPH_Shape *shape,
                             const Vector3 *point,
                             const JPH_CollisionCollectorType collectorType,
                             JPH_CollidePointResultCallback *callback,
                             void *userData,
                             const JPH_ShapeFilter *shapeFilter)
{
    const JPH::Vec3 joltPoint = ToJolt(point);
    constexpr JPH::SubShapeIDCreator creator;
    JPH_CollidePointResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsShape(shape)->CollidePoint(joltPoint, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::CollidePointResult &hit: collector.mHits)
                {
                    result.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    result.subShapeID2 = hit.mSubShapeID2.GetValue();
                    callback(userData, &result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsShape(shape)->CollidePoint(joltPoint, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> collector;
            AsShape(shape)->CollidePoint(joltPoint, creator, collector, ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                result.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}
