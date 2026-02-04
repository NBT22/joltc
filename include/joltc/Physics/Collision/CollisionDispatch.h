//
// Created by NBT22 on 2/3/26.
//

#ifndef JOLTC_COLLISIONDISPATCH_H
#define JOLTC_COLLISIONDISPATCH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Math/Mat44.h>
#include <joltc/Math/Vector3.h>
#include <joltc/Physics/Collision/CollideShape.h>
#include <joltc/Physics/Collision/Shape/Shape.h>
#include <joltc/Physics/Collision/ShapeCast.h>
#include <joltc/Physics/Collision/ShapeFilter.h>

JPH_CAPI bool JPH_CollisionDispatch_CollideShapeVsShape(const JPH_Shape *shape1,
                                                        const JPH_Shape *shape2,
                                                        const Vector3 *scale1,
                                                        const Vector3 *scale2,
                                                        const JPH_Mat44 *centerOfMassTransform1,
                                                        const JPH_Mat44 *centerOfMassTransform2,
                                                        const JPH_CollideShapeSettings *collideShapeSettings,
                                                        JPH_CollideShapeCollectorCallback *callback,
                                                        void *userData,
                                                        const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_CollisionDispatch_CastShapeVsShapeLocalSpace(const Vector3 *direction,
                                                               const JPH_Shape *shape1,
                                                               const JPH_Shape *shape2,
                                                               const Vector3 *scale1InShape2LocalSpace,
                                                               const Vector3 *scale2,
                                                               const JPH_Mat44
                                                                       *centerOfMassTransform1InShape2LocalSpace,
                                                               const JPH_Mat44 *centerOfMassWorldTransform2,
                                                               const JPH_ShapeCastSettings *shapeCastSettings,
                                                               JPH_CastShapeCollectorCallback *callback,
                                                               void *userData,
                                                               const JPH_ShapeFilter *shapeFilter);

JPH_CAPI bool JPH_CollisionDispatch_CastShapeVsShapeWorldSpace(const Vector3 *direction,
                                                               const JPH_Shape *shape1,
                                                               const JPH_Shape *shape2,
                                                               const Vector3 *scale1,
                                                               const Vector3 *scale2,
                                                               const JPH_Mat44 *centerOfMassWorldTransform1,
                                                               const JPH_Mat44 *centerOfMassWorldTransform2,
                                                               const JPH_ShapeCastSettings *shapeCastSettings,
                                                               JPH_CastShapeCollectorCallback *callback,
                                                               void *userData,
                                                               const JPH_ShapeFilter *shapeFilter);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_COLLISIONDISPATCH_H
