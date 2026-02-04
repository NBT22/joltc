//
// Created by NBT22 on 2/3/26.
//

#include <joltc/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

class ManagedBroadPhaseLayerFilter final: public JPH::BroadPhaseLayerFilter
{
    public:
        explicit ManagedBroadPhaseLayerFilter(const JPH_BroadPhaseLayerFilter_Impl *impl): impl(impl) {}

        [[nodiscard]] bool ShouldCollide(const JPH::BroadPhaseLayer inLayer) const override
        {
            if (impl != nullptr && impl->ShouldCollide != nullptr)
            {
                return impl->ShouldCollide(static_cast<JPH_BroadPhaseLayer>(inLayer));
            }

            return true;
        }

        const JPH_BroadPhaseLayerFilter_Impl *impl;
};

JPH_BroadPhaseLayerFilter *JPH_BroadPhaseLayerFilter_Create(const JPH_BroadPhaseLayerFilter_Impl *impl)
{
    ManagedBroadPhaseLayerFilter *const filter = new ManagedBroadPhaseLayerFilter(impl);
    return reinterpret_cast<JPH_BroadPhaseLayerFilter *>(filter);
}

void JPH_BroadPhaseLayerFilter_Destroy(JPH_BroadPhaseLayerFilter *filter)
{
    if (filter != nullptr)
    {
        delete reinterpret_cast<ManagedBroadPhaseLayerFilter *>(filter);
    }
}

void JPH_BroadPhaseLayerFilter_SetImpl(JPH_BroadPhaseLayerFilter *filter, const JPH_BroadPhaseLayerFilter_Impl *impl)
{
    JPH_ASSERT(filter);
    reinterpret_cast<ManagedBroadPhaseLayerFilter *>(filter)->impl = impl;
}
