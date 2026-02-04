//
// Created by NBT22 on 8/15/25.
//

#include <joltc/Physics/Collision/ShapeFilter.h>
#include <joltc/types.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Physics/Collision/Shape/Shape.hpp>

class ManagedShapeFilter final: public JPH::ShapeFilter
{
    public:
        static const ManagedShapeFilter *ToManagedShapeFilter(const JPH_ShapeFilter *filter)
        {
            return reinterpret_cast<const ManagedShapeFilter *>(filter);
        }
        static ManagedShapeFilter *ToManagedShapeFilter(JPH_ShapeFilter *filter)
        {
            return reinterpret_cast<ManagedShapeFilter *>(filter);
        }
        static const JPH_ShapeFilter *FromManagedShapeFilter(const ManagedShapeFilter *filter)
        {
            return reinterpret_cast<const JPH_ShapeFilter *>(filter);
        }
        static JPH_ShapeFilter *FromManagedShapeFilter(ManagedShapeFilter *filter)
        {
            return reinterpret_cast<JPH_ShapeFilter *>(filter);
        }

        explicit ManagedShapeFilter(const JPH_ShapeFilter_Impl *impl): impl(impl) {}

        bool ShouldCollide(const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override
        {
            if (impl != nullptr && impl->ShouldCollide != nullptr)
            {
                return impl->ShouldCollide(ToShape(inShape2), inSubShapeIDOfShape2.GetValue());
            }

            return true;
        }

        bool ShouldCollide(const JPH::Shape *inShape1,
                           const JPH::SubShapeID &inSubShapeIDOfShape1,
                           const JPH::Shape *inShape2,
                           const JPH::SubShapeID &inSubShapeIDOfShape2) const override
        {
            if (impl != nullptr && impl->ShouldCollide2 != nullptr)
            {
                return impl->ShouldCollide2(ToShape(inShape1),
                                            inSubShapeIDOfShape1.GetValue(),
                                            ToShape(inShape2),
                                            inSubShapeIDOfShape2.GetValue());
            }

            return true;
        }

        const JPH_ShapeFilter_Impl *impl{};
};

JPH_ShapeFilter *JPH_ShapeFilter_Create(const JPH_ShapeFilter_Impl *impl)
{
    ManagedShapeFilter *const filter = new ManagedShapeFilter(impl);
    return reinterpret_cast<JPH_ShapeFilter *>(filter);
}

void JPH_ShapeFilter_Destroy(JPH_ShapeFilter *filter)
{
    delete reinterpret_cast<ManagedShapeFilter *>(filter);
}

void JPH_ShapeFilter_SetImpl(JPH_ShapeFilter *filter, const JPH_ShapeFilter_Impl *impl)
{
    JPH_ASSERT(filter);
    reinterpret_cast<ManagedShapeFilter *>(filter)->impl = impl;
}

JPH_BodyID JPH_ShapeFilter_GetBodyID2(JPH_ShapeFilter *filter)
{
    return reinterpret_cast<ManagedShapeFilter *>(filter)->mBodyID2.GetIndexAndSequenceNumber();
}

void JPH_ShapeFilter_SetBodyID2(JPH_ShapeFilter *filter, const JPH_BodyID id)
{
    reinterpret_cast<ManagedShapeFilter *>(filter)->mBodyID2 = JPH::BodyID(id);
}
