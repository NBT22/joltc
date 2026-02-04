//
// Created by NBT22 on 2/3/26.
//

#include <joltc/Math/Mat44.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/CollisionDispatch.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Math/Mat44.hpp>
#include <Math/Vector3.hpp>
#include <Physics/Collision/CollideShape.hpp>
#include <Physics/Collision/Shape/Shape.hpp>
#include <Physics/Collision/ShapeCast.hpp>
#include <Physics/Collision/ShapeFilter.hpp>

bool JPH_CollisionDispatch_CollideShapeVsShape(const JPH_Shape *shape1,
                                               const JPH_Shape *shape2,
                                               const Vector3 *scale1,
                                               const Vector3 *scale2,
                                               const JPH_Mat44 *centerOfMassTransform1,
                                               const JPH_Mat44 *centerOfMassTransform2,
                                               const JPH_CollideShapeSettings *collideShapeSettings,
                                               JPH_CollideShapeCollectorCallback *callback,
                                               void *userData,
                                               const JPH_ShapeFilter *shapeFilter)
{
    CollideShapeCollectorCallback collector(callback, userData);

    JPH::CollisionDispatch::sCollideShapeVsShape(AsShape(shape1),
                                                 AsShape(shape2),
                                                 ToJolt(scale1),
                                                 ToJolt(scale2),
                                                 ToJolt(centerOfMassTransform1),
                                                 ToJolt(centerOfMassTransform2),
                                                 JPH::SubShapeIDCreator(),
                                                 JPH::SubShapeIDCreator(),
                                                 ToJolt(collideShapeSettings),
                                                 collector,
                                                 ToJolt(shapeFilter));

    return collector.hadHit;
}

bool JPH_CollisionDispatch_CastShapeVsShapeLocalSpace(const Vector3 *direction,
                                                      const JPH_Shape *shape1,
                                                      const JPH_Shape *shape2,
                                                      const Vector3 *scale1InShape2LocalSpace,
                                                      const Vector3 *scale2,
                                                      const JPH_Mat44 *centerOfMassTransform1InShape2LocalSpace,
                                                      const JPH_Mat44 *centerOfMassWorldTransform2,
                                                      const JPH_ShapeCastSettings *shapeCastSettings,
                                                      JPH_CastShapeCollectorCallback *callback,
                                                      void *userData,
                                                      const JPH_ShapeFilter *shapeFilter)
{
    const JPH::ShapeCast shapeCast(AsShape(shape1),
                                   ToJolt(scale1InShape2LocalSpace),
                                   ToJolt(centerOfMassTransform1InShape2LocalSpace),
                                   ToJolt(direction));

    CastShapeCollectorCallback collector(callback, userData);

    JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(shapeCast,
                                                        ToJolt(shapeCastSettings),
                                                        AsShape(shape2),
                                                        ToJolt(scale2),
                                                        ToJolt(shapeFilter),
                                                        ToJolt(centerOfMassWorldTransform2),
                                                        JPH::SubShapeIDCreator(),
                                                        JPH::SubShapeIDCreator(),
                                                        collector);

    return collector.hadHit;
}

bool JPH_CollisionDispatch_CastShapeVsShapeWorldSpace(const Vector3 *direction,
                                                      const JPH_Shape *shape1,
                                                      const JPH_Shape *shape2,
                                                      const Vector3 *scale1,
                                                      const Vector3 *scale2,
                                                      const JPH_Mat44 *centerOfMassWorldTransform1,
                                                      const JPH_Mat44 *centerOfMassWorldTransform2,
                                                      const JPH_ShapeCastSettings *shapeCastSettings,
                                                      JPH_CastShapeCollectorCallback *callback,
                                                      void *userData,
                                                      const JPH_ShapeFilter *shapeFilter)
{
    const JPH::ShapeCast shapeCast = JPH::ShapeCast::sFromWorldTransform(AsShape(shape1),
                                                                         ToJolt(scale1),
                                                                         ToJolt(centerOfMassWorldTransform1),
                                                                         ToJolt(direction));

    CastShapeCollectorCallback collector(callback, userData);

    JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(shapeCast,
                                                        ToJolt(shapeCastSettings),
                                                        AsShape(shape2),
                                                        ToJolt(scale2),
                                                        ToJolt(shapeFilter),
                                                        ToJolt(centerOfMassWorldTransform2),
                                                        JPH::SubShapeIDCreator(),
                                                        JPH::SubShapeIDCreator(),
                                                        collector);

    return collector.hadHit;
}
