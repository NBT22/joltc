//
// Created by NBT22 on 2/3/26.
//

#include <cstdint>
#include <joltc/enums.h>
#include <joltc/Math/RMat44.h>
#include <joltc/Math/RVec3.h>
#include <joltc/Math/Transform.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Body/BodyFilter.h>
#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Body/BodyInterface.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/CastResult.h>
#include <joltc/Physics/Collision/CollidePointResult.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/NarrowPhaseQuery.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <joltc/Physics/Collision/RayCast.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Math/Mat44.hpp>
#include <Math/Quat.hpp>
#include <Math/RMat44.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Body/BodyFilter.hpp>
#include <Physics/Body/BodyInterface.hpp>
#include <Physics/Collision/BroadPhase/BroadPhaseLayer.hpp>
#include <Physics/Collision/CollideShape.hpp>
#include <Physics/Collision/NarrowPhaseQuery.hpp>
#include <Physics/Collision/ObjectLayer.hpp>
#include <Physics/Collision/RayCast.hpp>
#include <Physics/Collision/Shape/Shape.hpp>
#include <Physics/Collision/ShapeCast.hpp>
#include <Physics/Collision/ShapeFilter.hpp>

class CastRayCollectorCallback final: public JPH::CastRayCollector
{
    public:
        CastRayCollectorCallback(JPH_CastRayCollectorCallback *proc_, void *userData_): proc(proc_), userData(userData_)
        {}

        void AddHit(const JPH::RayCastResult &result) override
        {
            JPH_RayCastResult hit;
            hit.bodyID = result.mBodyID.GetIndexAndSequenceNumber();
            hit.fraction = result.mFraction;
            hit.subShapeID2 = result.mSubShapeID2.GetValue();

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CastRayCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

class CollidePointCollectorCallback final: public JPH::CollidePointCollector
{
    public:
        CollidePointCollectorCallback(JPH_CollidePointCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::CollidePointResult &result) override
        {
            JPH_CollidePointResult hit;
            hit.bodyID = result.mBodyID.GetIndexAndSequenceNumber();
            hit.subShapeID2 = result.mSubShapeID2.GetValue();

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CollidePointCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

bool JPH_NarrowPhaseQuery_CastRay_GAME(const JPH_NarrowPhaseQuery *query,
                                       const Transform *transform,
                                       const float maxDistance,
                                       JPH_RayCastResult *result,
                                       const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       const JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(transform);
    JPH_ASSERT(result);
    JPH_ASSERT(broadPhaseLayerFilter);
    JPH_ASSERT(objectLayerFilter);

    const JPH::RRayCast ray(ToJolt(transform->position),
                            ToJolt(transform->rotation) * -JPH::Vec3::sAxisZ() * maxDistance);
    constexpr JPH::RayCastSettings raySettings{};
    JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector{};
    const JPH::BodyFilter bodyFilter{};
    const JPH::ShapeFilter shapeFilter{};

    ToJolt(query).CastRay(ray,
                          raySettings,
                          collector,
                          ToJolt(broadPhaseLayerFilter),
                          ToJolt(objectLayerFilter),
                          bodyFilter,
                          shapeFilter);

    if (collector.HadHit())
    {
        result->fraction = collector.mHit.mFraction;
        result->bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
        result->subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
    }

    return collector.HadHit();
}

bool JPH_NarrowPhaseQuery_CastRay2_GAME(const JPH_NarrowPhaseQuery *query,
                                        const JPH_BodyInterface *bodyInterface,
                                        const JPH_BodyID bodyId,
                                        const float maxDistance,
                                        JPH_RayCastResult *result,
                                        Vector3 *hitPointOffset,
                                        const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                        const JPH_ObjectLayerFilter *objectLayerFilter,
                                        const JPH_BodyFilter *bodyFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(bodyInterface);
    JPH_ASSERT(bodyId != JPH_BodyId_InvalidBodyID);
    JPH_ASSERT(result);
    JPH_ASSERT(hitPointOffset);
    JPH_ASSERT(broadPhaseLayerFilter);
    JPH_ASSERT(objectLayerFilter);

    JPH::RVec3 position{};
    JPH::Quat rotation{};
    AsBodyInterface(bodyInterface)->GetPositionAndRotation(JPH::BodyID(bodyId), position, rotation);
    const JPH::RVec3 forward = rotation * -JPH::Vec3::sAxisZ();
    const JPH::RRayCast ray(position, forward * maxDistance);
    constexpr JPH::RayCastSettings raySettings{};
    JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector{};
    const JPH::ShapeFilter shapeFilter{};

    ToJolt(query).CastRay(ray,
                          raySettings,
                          collector,
                          ToJolt(broadPhaseLayerFilter),
                          ToJolt(objectLayerFilter),
                          ToJolt(bodyFilter),
                          shapeFilter);

    if (collector.HadHit())
    {
        result->fraction = collector.mHit.mFraction;
        result->bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
        result->subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
        FromJolt(-JPH::Vec3::sAxisZ() * collector.mHit.mFraction * maxDistance, hitPointOffset);
    }

    return collector.HadHit();
}

bool JPH_NarrowPhaseQuery_CastRay(const JPH_NarrowPhaseQuery *query,
                                  const JPH_RVec3 *origin,
                                  const Vector3 *direction,
                                  JPH_RayCastResult *hit,
                                  const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                  const JPH_ObjectLayerFilter *objectLayerFilter,
                                  const JPH_BodyFilter *bodyFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(origin);
    JPH_ASSERT(direction);
    JPH_ASSERT(hit);

    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    JPH::RayCastResult result;

    const bool hadHit = ToJolt(query).CastRay(ray,
                                              result,
                                              ToJolt(broadPhaseLayerFilter),
                                              ToJolt(objectLayerFilter),
                                              ToJolt(bodyFilter));

    if (hadHit)
    {
        hit->fraction = result.mFraction;
        hit->bodyID = result.mBodyID.GetIndexAndSequenceNumber();
        hit->subShapeID2 = result.mSubShapeID2.GetValue();
    }

    return hadHit;
}

bool JPH_NarrowPhaseQuery_CastRay2(const JPH_NarrowPhaseQuery *query,
                                   const JPH_RVec3 *origin,
                                   const Vector3 *direction,
                                   const JPH_RayCastSettings *rayCastSettings,
                                   JPH_CastRayCollectorCallback *callback,
                                   void *userData,
                                   const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                   const JPH_ObjectLayerFilter *objectLayerFilter,
                                   const JPH_BodyFilter *bodyFilter,
                                   const JPH_ShapeFilter *shapeFilter)
{
    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RayCastSettings raySettings = ToJolt(rayCastSettings);

    CastRayCollectorCallback collector(callback, userData);

    ToJolt(query).CastRay(ray,
                          raySettings,
                          collector,
                          ToJolt(broadPhaseLayerFilter),
                          ToJolt(objectLayerFilter),
                          ToJolt(bodyFilter),
                          ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CastRay3(const JPH_NarrowPhaseQuery *query,
                                   const JPH_RVec3 *origin,
                                   const Vector3 *direction,
                                   const JPH_RayCastSettings *rayCastSettings,
                                   const JPH_CollisionCollectorType collectorType,
                                   JPH_CastRayResultCallback *callback,
                                   void *userData,
                                   const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                   const JPH_ObjectLayerFilter *objectLayerFilter,
                                   const JPH_BodyFilter *bodyFilter,
                                   const JPH_ShapeFilter *shapeFilter)
{
    const JPH::RRayCast ray(ToJolt(origin), ToJolt(direction));
    const JPH::RayCastSettings raySettings = ToJolt(rayCastSettings);
    JPH_RayCastResult hitResult{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
            ToJolt(query).CastRay(ray,
                                  raySettings,
                                  collector,
                                  ToJolt(broadPhaseLayerFilter),
                                  ToJolt(objectLayerFilter),
                                  ToJolt(bodyFilter),
                                  ToJolt(shapeFilter));

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
            ToJolt(query).CastRay(ray,
                                  raySettings,
                                  collector,
                                  ToJolt(broadPhaseLayerFilter),
                                  ToJolt(objectLayerFilter),
                                  ToJolt(bodyFilter),
                                  ToJolt(shapeFilter));

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
            ToJolt(query).CastRay(ray,
                                  raySettings,
                                  collector,
                                  ToJolt(broadPhaseLayerFilter),
                                  ToJolt(objectLayerFilter),
                                  ToJolt(bodyFilter),
                                  ToJolt(shapeFilter));

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

bool JPH_NarrowPhaseQuery_CollidePoint(const JPH_NarrowPhaseQuery *query,
                                       const JPH_RVec3 *point,
                                       JPH_CollidePointCollectorCallback *callback,
                                       void *userData,
                                       const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       const JPH_ObjectLayerFilter *objectLayerFilter,
                                       const JPH_BodyFilter *bodyFilter,
                                       const JPH_ShapeFilter *shapeFilter)
{
    CollidePointCollectorCallback collector(callback, userData);
    ToJolt(query).CollidePoint(ToJolt(point),
                               collector,
                               ToJolt(broadPhaseLayerFilter),
                               ToJolt(objectLayerFilter),
                               ToJolt(bodyFilter),
                               ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CollidePoint2(const JPH_NarrowPhaseQuery *query,
                                        const JPH_RVec3 *point,
                                        const JPH_CollisionCollectorType collectorType,
                                        JPH_CollidePointResultCallback *callback,
                                        void *userData,
                                        const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                        const JPH_ObjectLayerFilter *objectLayerFilter,
                                        const JPH_BodyFilter *bodyFilter,
                                        const JPH_ShapeFilter *shapeFilter)
{
    const JPH::Vec3 joltPoint = ToJolt(point);
    JPH_CollidePointResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CollidePointCollector> collector;
            ToJolt(query).CollidePoint(joltPoint,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

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
            ToJolt(query).CollidePoint(joltPoint,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

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
            ToJolt(query).CollidePoint(joltPoint,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

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

bool JPH_NarrowPhaseQuery_CollideShape(const JPH_NarrowPhaseQuery *query,
                                       const JPH_Shape *shape,
                                       const Vector3 *scale,
                                       const JPH_RMat44 *centerOfMassTransform,
                                       const JPH_CollideShapeSettings *settings,
                                       JPH_RVec3 *baseOffset,
                                       JPH_CollideShapeCollectorCallback *callback,
                                       void *userData,
                                       const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       const JPH_ObjectLayerFilter *objectLayerFilter,
                                       const JPH_BodyFilter *bodyFilter,
                                       const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(shape);
    JPH_ASSERT(scale);
    JPH_ASSERT(centerOfMassTransform);
    JPH_ASSERT(callback);

    CollideShapeCollectorCallback collector(callback, userData);

    ToJolt(query).CollideShape(AsShape(shape),
                               ToJolt(scale),
                               ToJolt(centerOfMassTransform),
                               ToJolt(settings),
                               ToJolt(baseOffset),
                               collector,
                               ToJolt(broadPhaseLayerFilter),
                               ToJolt(objectLayerFilter),
                               ToJolt(bodyFilter),
                               ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CollideShape2(const JPH_NarrowPhaseQuery *query,
                                        const JPH_Shape *shape,
                                        const Vector3 *scale,
                                        const JPH_RMat44 *centerOfMassTransform,
                                        const JPH_CollideShapeSettings *settings,
                                        JPH_RVec3 *baseOffset,
                                        JPH_CollisionCollectorType collectorType,
                                        JPH_CollideShapeResultCallback *callback,
                                        void *userData,
                                        const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                        const JPH_ObjectLayerFilter *objectLayerFilter,
                                        const JPH_BodyFilter *bodyFilter,
                                        const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(shape);
    JPH_ASSERT(scale);
    JPH_ASSERT(centerOfMassTransform);
    JPH_ASSERT(callback);

    const JPH::Vec3 joltScale = ToJolt(scale);
    const JPH::Mat44 joltTransform = ToJolt(centerOfMassTransform);

    const JPH::CollideShapeSettings joltSettings = ToJolt(settings);
    const JPH::Vec3 joltBaseOffset = ToJolt(baseOffset);

    const JPH_CollideShapeResult *result = nullptr;

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
            ToJolt(query).CollideShape(AsShape(shape),
                                       joltScale,
                                       joltTransform,
                                       joltSettings,
                                       joltBaseOffset,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::CollideShapeResult &hit: collector.mHits)
                {
                    FromJolt(&hit, result);
                    callback(userData, result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> collector;
            ToJolt(query).CollideShape(AsShape(shape),
                                       joltScale,
                                       joltTransform,
                                       joltSettings,
                                       joltBaseOffset,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                FromJolt(&collector.mHit, result);
                callback(userData, result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
            ToJolt(query).CollideShape(AsShape(shape),
                                       joltScale,
                                       joltTransform,
                                       joltSettings,
                                       joltBaseOffset,
                                       collector,
                                       ToJolt(broadPhaseLayerFilter),
                                       ToJolt(objectLayerFilter),
                                       ToJolt(bodyFilter),
                                       ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                FromJolt(&collector.mHit, result);
                callback(userData, result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_NarrowPhaseQuery_CastShape(const JPH_NarrowPhaseQuery *query,
                                    const JPH_Shape *shape,
                                    const JPH_RMat44 *worldTransform,
                                    const Vector3 *direction,
                                    const JPH_ShapeCastSettings *settings,
                                    JPH_RVec3 *baseOffset,
                                    JPH_CastShapeCollectorCallback *callback,
                                    void *userData,
                                    const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                    const JPH_ObjectLayerFilter *objectLayerFilter,
                                    const JPH_BodyFilter *bodyFilter,
                                    const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(shape);
    JPH_ASSERT(worldTransform);
    JPH_ASSERT(direction);
    JPH_ASSERT(callback);

    CastShapeCollectorCallback collector(callback, userData);
    const JPH::RShapeCast shapeCast =
            JPH::RShapeCast::sFromWorldTransform(AsShape(shape),
                                                 JPH::Vec3(1.f,
                                                           1.f,
                                                           1.f), // scale can be embedded in worldTransform
                                                 ToJolt(worldTransform),
                                                 ToJolt(direction));
    ToJolt(query).CastShape(shapeCast,
                            ToJolt(settings),
                            ToJolt(baseOffset),
                            collector,
                            ToJolt(broadPhaseLayerFilter),
                            ToJolt(objectLayerFilter),
                            ToJolt(bodyFilter),
                            ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_NarrowPhaseQuery_CastShape2(const JPH_NarrowPhaseQuery *query,
                                     const JPH_Shape *shape,
                                     const JPH_RMat44 *worldTransform,
                                     const Vector3 *direction,
                                     const JPH_ShapeCastSettings *settings,
                                     JPH_RVec3 *baseOffset,
                                     JPH_CollisionCollectorType collectorType,
                                     JPH_CastShapeResultCallback *callback,
                                     void *userData,
                                     const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                     const JPH_ObjectLayerFilter *objectLayerFilter,
                                     const JPH_BodyFilter *bodyFilter,
                                     const JPH_ShapeFilter *shapeFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(shape);
    JPH_ASSERT(worldTransform);
    JPH_ASSERT(direction);
    JPH_ASSERT(callback);

    const JPH::RShapeCast shapeCast =
            JPH::RShapeCast::sFromWorldTransform(AsShape(shape),
                                                 JPH::Vec3(1.f,
                                                           1.f,
                                                           1.f), // scale can be embedded in worldTransform
                                                 ToJolt(worldTransform),
                                                 ToJolt(direction));

    const JPH::ShapeCastSettings joltSettings = ToJolt(settings);

    const JPH::Vec3 joltBaseOffset = ToJolt(baseOffset);

    JPH_ShapeCastResult result{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
            ToJolt(query).CastShape(shapeCast,
                                    joltSettings,
                                    joltBaseOffset,
                                    collector,
                                    ToJolt(broadPhaseLayerFilter),
                                    ToJolt(objectLayerFilter),
                                    ToJolt(bodyFilter),
                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::ShapeCastResult &hit: collector.mHits)
                {
                    FromJolt(hit.mContactPointOn1, &result.contactPointOn1);
                    FromJolt(hit.mContactPointOn2, &result.contactPointOn2);
                    FromJolt(hit.mPenetrationAxis, &result.penetrationAxis);
                    result.penetrationDepth = hit.mPenetrationDepth;
                    result.subShapeID1 = hit.mSubShapeID1.GetValue();
                    result.subShapeID2 = hit.mSubShapeID2.GetValue();
                    result.bodyID2 = hit.mBodyID2.GetIndexAndSequenceNumber();
                    result.fraction = hit.mFraction;
                    result.isBackFaceHit = hit.mIsBackFaceHit;
                    callback(userData, &result);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
            ToJolt(query).CastShape(shapeCast,
                                    joltSettings,
                                    joltBaseOffset,
                                    collector,
                                    ToJolt(broadPhaseLayerFilter),
                                    ToJolt(objectLayerFilter),
                                    ToJolt(bodyFilter),
                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                FromJolt(collector.mHit.mContactPointOn1, &result.contactPointOn1);
                FromJolt(collector.mHit.mContactPointOn2, &result.contactPointOn2);
                FromJolt(collector.mHit.mPenetrationAxis, &result.penetrationAxis);
                result.penetrationDepth = collector.mHit.mPenetrationDepth;
                result.subShapeID1 = collector.mHit.mSubShapeID1.GetValue();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                result.bodyID2 = collector.mHit.mBodyID2.GetIndexAndSequenceNumber();
                result.fraction = collector.mHit.mFraction;
                result.isBackFaceHit = collector.mHit.mIsBackFaceHit;
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::CastShapeCollector> collector;
            ToJolt(query).CastShape(shapeCast,
                                    joltSettings,
                                    joltBaseOffset,
                                    collector,
                                    ToJolt(broadPhaseLayerFilter),
                                    ToJolt(objectLayerFilter),
                                    ToJolt(bodyFilter),
                                    ToJolt(shapeFilter));

            if (collector.HadHit())
            {
                FromJolt(collector.mHit.mContactPointOn1, &result.contactPointOn1);
                FromJolt(collector.mHit.mContactPointOn2, &result.contactPointOn2);
                FromJolt(collector.mHit.mPenetrationAxis, &result.penetrationAxis);
                result.penetrationDepth = collector.mHit.mPenetrationDepth;
                result.subShapeID1 = collector.mHit.mSubShapeID1.GetValue();
                result.subShapeID2 = collector.mHit.mSubShapeID2.GetValue();
                result.bodyID2 = collector.mHit.mBodyID2.GetIndexAndSequenceNumber();
                result.fraction = collector.mHit.mFraction;
                result.isBackFaceHit = collector.mHit.mIsBackFaceHit;
                callback(userData, &result);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}
