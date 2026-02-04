//
// Created by NBT22 on 2/3/26.
//

#include <joltc/Physics/Collision/ObjectLayer.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

class ManagedObjectLayerFilter final: public JPH::ObjectLayerFilter
{
    public:
        explicit ManagedObjectLayerFilter(const JPH_ObjectLayerFilter_Impl *impl): impl(impl) {}

        [[nodiscard]] bool ShouldCollide(const JPH::ObjectLayer inLayer) const override
        {
            if (impl != nullptr && impl->ShouldCollide != nullptr)
            {
                return impl->ShouldCollide(inLayer);
            }

            return true;
        }

        const JPH_ObjectLayerFilter_Impl *impl{};
};

JPH_ObjectLayerFilter *JPH_ObjectLayerFilter_Create(const JPH_ObjectLayerFilter_Impl *impl)
{
    ManagedObjectLayerFilter *const filter = new ManagedObjectLayerFilter(impl);
    return reinterpret_cast<JPH_ObjectLayerFilter *>(filter);
}

void JPH_ObjectLayerFilter_Destroy(JPH_ObjectLayerFilter *filter)
{
    delete reinterpret_cast<ManagedObjectLayerFilter *>(filter);
}

void JPH_ObjectLayerFilter_SetImpl(JPH_ObjectLayerFilter *filter, const JPH_ObjectLayerFilter_Impl *impl)
{
    JPH_ASSERT(filter);
    reinterpret_cast<ManagedObjectLayerFilter *>(filter)->impl = impl;
}
