//
// Created by NBT22 on 2/3/26.
//

#include <cstdint>
#include <joltc/enums.h>
#include <joltc/Geometry/AABox.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <joltc/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <joltc/Physics/Collision/CastResult.h>
#include <joltc/Physics/Collision/ObjectLayer.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Math/Vector3.hpp>
#include <Physics/Collision/BroadPhase/BroadPhaseLayer.hpp>
#include <Physics/Collision/BroadPhase/BroadPhaseQuery.hpp>
#include <Physics/Collision/ObjectLayer.hpp>

class RayCastBodyCollectorCallback final: public JPH::RayCastBodyCollector
{
    public:
        RayCastBodyCollectorCallback(JPH_RayCastBodyCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::BroadPhaseCastResult &result) override
        {
            JPH_BroadPhaseCastResult hit;
            hit.bodyID = result.mBodyID.GetIndexAndSequenceNumber();
            hit.fraction = result.mFraction;

            const float fraction = proc(userData, &hit);
            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_RayCastBodyCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

class CollideShapeBodyCollectorCallback final: public JPH::CollideShapeBodyCollector
{
    public:
        CollideShapeBodyCollectorCallback(JPH_CollideShapeBodyCollectorCallback *proc_, void *userData_):
            proc(proc_),
            userData(userData_)
        {}

        void AddHit(const JPH::BodyID &result) override
        {
            const float fraction = proc(userData, result.GetIndexAndSequenceNumber());

            UpdateEarlyOutFraction(fraction);
            hadHit = true;
        }

        JPH_CollideShapeBodyCollectorCallback *proc;
        void *userData;
        bool hadHit = false;
        uint32_t _padding{0};
};

bool JPH_BroadPhaseQuery_CastRay(const JPH_BroadPhaseQuery *query,
                                 const Vector3 *origin,
                                 const Vector3 *direction,
                                 JPH_RayCastBodyCollectorCallback *callback,
                                 void *userData,
                                 const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                 const JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(origin);
    JPH_ASSERT(direction);
    JPH_ASSERT(callback);

    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    RayCastBodyCollectorCallback collector(callback, userData);
    ToJolt(query).CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));
    return collector.hadHit;
}

bool JPH_BroadPhaseQuery_CastRay2(const JPH_BroadPhaseQuery *query,
                                  const Vector3 *origin,
                                  const Vector3 *direction,
                                  const JPH_CollisionCollectorType collectorType,
                                  JPH_RayCastBodyResultCallback *callback,
                                  void *userData,
                                  const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                  const JPH_ObjectLayerFilter *objectLayerFilter)
{
    const JPH::RayCast ray(ToJolt(origin), ToJolt(direction));
    JPH_BroadPhaseCastResult hitResult{};

    switch (collectorType)
    {
        case JPH_CollisionCollectorType_AllHit:
        case JPH_CollisionCollectorType_AllHitSorted:
        {
            JPH::AllHitCollisionCollector<JPH::RayCastBodyCollector> collector;
            ToJolt(query).CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));

            if (collector.HadHit())
            {
                if (collectorType == JPH_CollisionCollectorType_AllHitSorted)
                {
                    collector.Sort();
                }

                for (const JPH::BroadPhaseCastResult &hit: collector.mHits)
                {
                    hitResult.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
                    hitResult.fraction = hit.mFraction;
                    callback(userData, &hitResult);
                }
            }

            return collector.HadHit();
        }
        case JPH_CollisionCollectorType_ClosestHit:
        {
            JPH::ClosestHitCollisionCollector<JPH::RayCastBodyCollector> collector;
            ToJolt(query).CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));

            if (collector.HadHit())
            {
                hitResult.fraction = collector.mHit.mFraction;
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        case JPH_CollisionCollectorType_AnyHit:
        {
            JPH::AnyHitCollisionCollector<JPH::RayCastBodyCollector> collector;
            ToJolt(query).CastRay(ray, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));

            if (collector.HadHit())
            {
                hitResult.bodyID = collector.mHit.mBodyID.GetIndexAndSequenceNumber();
                hitResult.fraction = collector.mHit.mFraction;
                callback(userData, &hitResult);
            }

            return collector.HadHit();
        }

        default:
            return false;
    }
}

bool JPH_BroadPhaseQuery_CollideAABox(const JPH_BroadPhaseQuery *query,
                                      const JPH_AABox *box,
                                      JPH_CollideShapeBodyCollectorCallback *callback,
                                      void *userData,
                                      const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                      const JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(box);
    JPH_ASSERT(callback);

    const JPH::AABox joltBox(ToJolt(&box->min), ToJolt(&box->max));
    CollideShapeBodyCollectorCallback collector(callback, userData);
    ToJolt(query).CollideAABox(joltBox, collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));
    return collector.hadHit;
}

bool JPH_BroadPhaseQuery_CollideSphere(const JPH_BroadPhaseQuery *query,
                                       const Vector3 *center,
                                       float radius,
                                       JPH_CollideShapeBodyCollectorCallback *callback,
                                       void *userData,
                                       const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                       const JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(center);
    JPH_ASSERT(callback);

    CollideShapeBodyCollectorCallback collector(callback, userData);
    ToJolt(query).CollideSphere(ToJolt(center),
                                radius,
                                collector,
                                ToJolt(broadPhaseLayerFilter),
                                ToJolt(objectLayerFilter));
    return collector.hadHit;
}

bool JPH_BroadPhaseQuery_CollidePoint(const JPH_BroadPhaseQuery *query,
                                      const Vector3 *point,
                                      JPH_CollideShapeBodyCollectorCallback *callback,
                                      void *userData,
                                      const JPH_BroadPhaseLayerFilter *broadPhaseLayerFilter,
                                      const JPH_ObjectLayerFilter *objectLayerFilter)
{
    JPH_ASSERT(query);
    JPH_ASSERT(point);
    JPH_ASSERT(callback);

    CollideShapeBodyCollectorCallback collector(callback, userData);
    ToJolt(query).CollidePoint(ToJolt(point), collector, ToJolt(broadPhaseLayerFilter), ToJolt(objectLayerFilter));
    return collector.hadHit;
}
