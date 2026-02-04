//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_SHAPEFILTER_H
#define JOLTC_SHAPEFILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/Physics/Body/BodyID.h>
#include <joltc/Physics/Collision/Shape/SubShapeID.h>

typedef struct JPH_Shape JPH_Shape; // Forward declaration

typedef struct JPH_ShapeFilter JPH_ShapeFilter;

typedef struct JPH_ShapeFilter_Impl
{
        bool(JPH_API_CALL *ShouldCollide)(const JPH_Shape *inShape2, JPH_SubShapeID inSubShapeIDOfShape2);
        bool(JPH_API_CALL *ShouldCollide2)(const JPH_Shape *shape1,
                                           JPH_SubShapeID subShapeIDOfShape1,
                                           const JPH_Shape *shape2,
                                           JPH_SubShapeID subShapeIDOfShape2);
} JPH_ShapeFilter_Impl;

JPH_CAPI JPH_ShapeFilter *JPH_ShapeFilter_Create(const JPH_ShapeFilter_Impl *impl);
JPH_CAPI void JPH_ShapeFilter_Destroy(JPH_ShapeFilter *filter);
JPH_CAPI void JPH_ShapeFilter_SetImpl(JPH_ShapeFilter *filter, const JPH_ShapeFilter_Impl *impl);
JPH_CAPI JPH_BodyID JPH_ShapeFilter_GetBodyID2(JPH_ShapeFilter *filter);
JPH_CAPI void JPH_ShapeFilter_SetBodyID2(JPH_ShapeFilter *filter, JPH_BodyID id);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_SHAPEFILTER_H
